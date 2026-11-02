// Source-independent, backpressured WAV file input. All ports use clk.
// I_file_size is stable from the first byte until I_stream_end. Packet gaps
// are represented only by !I_byte_valid; they MUST NOT assert I_stream_end.
// I_stream_end is sticky and rises AFTER the final byte has been accepted.
module wav_stream_pcm_pipeline (
    input wire clk, rst, I_start,
    input wire [31:0] I_file_size,
    input wire [7:0] I_byte,
    input wire I_byte_valid, I_stream_end,
    output wire O_byte_ready,
    input wire I_sample_req,
    output wire pcm_valid,
    output wire signed [15:0] pcm_left, pcm_right,
    output reg wav_done, O_error, O_file_found,
    output reg [3:0] O_error_code,
    output wire [3:0] O_state
);
    localparam IDLE=0, WAIT_FILE=1, PARSER_START=2, HEADER=3,
               EX_START=4, DATA=5, DRAIN=6, FINISHED=7;
    reg [3:0] state;
    reg [31:0] file_size;
    wire parser_done, parser_error, extractor_done, extractor_error;
    wire [3:0] parser_ec, extractor_ec;
    wire [31:0] data_offset, data_size;
    wire ex_ready;
    wire header_feed = state==HEADER && !parser_done && !parser_error;
    wire data_feed = state==DATA && !extractor_done && !extractor_error;
    assign O_state = state;
    assign O_byte_ready = header_feed || (data_feed && ex_ready) || state==DRAIN;

    tf_wav_parser u_pars (
        .I_clk(clk), .I_rst(rst), .I_start(state==PARSER_START),
        .I_byte_valid(header_feed && I_byte_valid), .I_byte(I_byte),
        .I_stream_end(1'b0), .O_busy(), .O_done(parser_done),
        .O_error(parser_error), .O_error_code(parser_ec),
        .O_fmt(), .O_channels(), .O_sample_rate(), .O_bits(),
        .O_data_offset(data_offset), .O_data_size(data_size), .O_state()
    );
    wav_pcm_byte_extractor u_ex (
        .clk(clk), .rst(rst), .I_start(state==EX_START),
        .I_data_size(data_size), .I_byte(I_byte),
        .I_byte_valid(data_feed && ex_ready && I_byte_valid),
        .I_stream_end(I_stream_end), .O_byte_ready(ex_ready),
        .pcm_valid(pcm_valid), .pcm_left(pcm_left), .pcm_right(pcm_right),
        .I_sample_req(I_sample_req), .wav_done(extractor_done),
        .O_error(extractor_error), .O_error_code(extractor_ec), .O_state()
    );
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state<=IDLE; file_size<=0; wav_done<=0; O_error<=0;
            O_error_code<=0; O_file_found<=0;
        end else case (state)
            IDLE: if (I_start) begin
                state<=WAIT_FILE; wav_done<=0; O_error<=0;
                O_error_code<=0; O_file_found<=0;
            end
            WAIT_FILE: if (I_byte_valid) begin
                file_size<=I_file_size; O_file_found<=1; state<=PARSER_START;
            end
            PARSER_START: state<=HEADER;
            HEADER: begin
                if (parser_error) begin
                    O_error<=1; O_error_code<=parser_ec; state<=DRAIN;
                end else if (parser_done) begin
                    if (data_offset > file_size || data_size > file_size-data_offset) begin
                        O_error<=1; O_error_code<=7; state<=DRAIN;
                    end else state<=EX_START;
                end else if (I_stream_end) begin
                    O_error<=1; O_error_code<=7; state<=DRAIN;
                end
            end
            EX_START: state<=DATA;
            DATA: begin
                if (extractor_error) begin
                    O_error<=1; O_error_code<=extractor_ec; state<=DRAIN;
                end else if (extractor_done) state<=DRAIN;
            end
            DRAIN: if (I_stream_end) begin
                wav_done<=!O_error; state<=FINISHED;
            end
            FINISHED: state<=IDLE;
            default: state<=IDLE;
        endcase
    end
endmodule
