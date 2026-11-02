// RIFF/WAVE PCM parser: 48 kHz, stereo, signed 16-bit little endian.
// Accept arbitrary unknown chunks, including zero size and WORD padding.
// Errors: 1 RIFF, 2 WAVE, 3 encoding, 4 rate, 5 bits, 6 channels,
//         7 missing/truncated/invalid layout. Results persist until I_start.
module tf_wav_parser (
    input wire I_clk, I_rst, I_start, I_byte_valid,
    input wire [7:0] I_byte,
    input wire I_stream_end,
    output wire O_busy,
    output reg O_done, O_error,
    output reg [3:0] O_error_code,
    output reg [15:0] O_fmt, O_channels,
    output reg [31:0] O_sample_rate,
    output reg [15:0] O_bits,
    output reg [31:0] O_data_offset, O_data_size,
    output wire [3:0] O_state
);
    localparam S_IDLE=0, S_RIFF=1, S_CID=3, S_CSZ=4,
               S_SKIP=5, S_FMT=6, S_DONE=7, S_ERR=8;
    reg [3:0] st;
    reg [31:0] bidx, cid, csize, remain;
    reg [3:0] idx;
    reg fmt_seen;
    reg fmt_ok, channels_ok, bits_ok, align_ok, rate_ok, byte_rate_ok;
    wire [31:0] size_now = {I_byte,csize[31:8]};
    assign O_state=st;
    assign O_busy=(st==S_RIFF || st==S_CID || st==S_CSZ || st==S_SKIP || st==S_FMT);
    task fail;
        input [3:0] code;
        begin st<=S_ERR; O_error<=1; O_error_code<=code; end
    endtask
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            st<=S_IDLE; bidx<=0; cid<=0; csize<=0; remain<=0; idx<=0;
            fmt_seen<=0; fmt_ok<=0;channels_ok<=0;bits_ok<=0;align_ok<=0;rate_ok<=0;byte_rate_ok<=0;
            O_done<=0; O_error<=0; O_error_code<=0;
            O_fmt<=0; O_channels<=0; O_sample_rate<=0; O_bits<=0;
            O_data_offset<=0; O_data_size<=0;
        end else if (I_start && (st==S_IDLE || st==S_DONE || st==S_ERR)) begin
            st<=S_RIFF; bidx<=0; idx<=0; cid<=0; csize<=0; fmt_seen<=0;
            O_done<=0; O_error<=0; O_error_code<=0;
            O_fmt<=0; O_channels<=0; O_sample_rate<=0; O_bits<=0;
            O_data_offset<=0; O_data_size<=0;
        end else if (I_byte_valid && O_busy) begin
            bidx<=bidx+1'b1;
            case (st)
                S_RIFF: begin
                    cid<={cid[23:0],I_byte}; idx<=idx+1'b1;
                    if (idx==3 && {cid[23:0],I_byte}!=32'h52494646) fail(1);
                    if (idx==11) begin
                        if ({cid[23:0],I_byte}!=32'h57415645) fail(2);
                        else begin st<=S_CID; idx<=0; cid<=0; end
                    end
                end
                S_CID: begin
                    cid<={cid[23:0],I_byte}; idx<=idx+1'b1;
                    if (idx==3) begin st<=S_CSZ; idx<=0; csize<=0; end
                end
                S_CSZ: begin
                    csize<=size_now; idx<=idx+1'b1;
                    // Shifted LE assembly: final byte supplies bits 31:24.
                    if (idx==3) begin
                        idx<=0;
                        if (cid==32'h666d7420) begin
                            if (size_now<16 || fmt_seen) fail(7);
                            else begin
                                st<=S_FMT; csize<=size_now;
                                fmt_ok<=0;channels_ok<=0;bits_ok<=0;align_ok<=0;rate_ok<=0;byte_rate_ok<=0;
                            end
                        end else if (cid==32'h64617461) begin
                            if (!fmt_seen || size_now==0 || csize[9:8]!=0) fail(7);
                            else begin
                                O_data_offset<=bidx+1'b1;
                                O_data_size<=size_now; O_done<=1; st<=S_DONE;
                            end
                        end else if (size_now==32'hffffffff) fail(7);
                        else if (size_now==0) begin st<=S_CID; cid<=0; end
                        else begin
                            remain <= size_now + {31'd0,csize[8]}; st<=S_SKIP;
                        end
                    end
                end
                S_FMT: begin
                    idx<=idx+1'b1;
                    // Validate fixed PCM fields as their little-endian bytes arrive.
                    case (idx)
                        0:fmt_ok<=I_byte==1;1:fmt_ok<=fmt_ok && I_byte==0;
                        2:channels_ok<=I_byte==2;3:channels_ok<=channels_ok && I_byte==0;
                        4:rate_ok<=I_byte==8'h80;5:rate_ok<=rate_ok && I_byte==8'hbb;
                        6,7:rate_ok<=rate_ok && I_byte==0;
                        8:byte_rate_ok<=I_byte==8'h00;9:byte_rate_ok<=byte_rate_ok && I_byte==8'hee;
                        10:byte_rate_ok<=byte_rate_ok && I_byte==8'h02;11:byte_rate_ok<=byte_rate_ok && I_byte==0;
                        12:align_ok<=I_byte==4;13:align_ok<=align_ok && I_byte==0;
                        14:bits_ok<=I_byte==16;
                    endcase
                    if (idx==15) begin
                        if (!fmt_ok) fail(3);
                        else if (!rate_ok) fail(4);
                        else if ((!bits_ok || I_byte!=0)) fail(5);
                        else if (!channels_ok) fail(6);
                        else if (!align_ok || !byte_rate_ok || csize==32'hffffffff) fail(7);
                        else begin
                            fmt_seen<=1; O_fmt<=16'd1; O_channels<=16'd2;
                            O_sample_rate<=32'd48000; O_bits<=16'd16;
                            idx<=0; cid<=0;
                            if (csize==16) st<=S_CID;
                            else begin remain<=csize-16+{31'd0,csize[0]}; st<=S_SKIP; end
                        end
                    end
                end
                S_SKIP: begin
                    remain<=remain-1'b1;
                    if (remain==1) begin st<=S_CID; idx<=0; cid<=0; end
                end
            endcase
        end else if (I_stream_end && O_busy) fail(7);
    end
endmodule
