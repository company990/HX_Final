// WAV1 stop-and-wait file transport. One file per FPGA/audio reset.
// Datagram: "WAV1", session u32, offset u32, file_size u32, 1..1024
// original WAV bytes, CRC32 u32. Header/CRC integers are network byte order.
// CRC is standard reflected IEEE CRC32 over header + payload.
// Reply: "WACK", session, next_offset, status, CRC32 (20 bytes).
// Status 0=committed, 1=duplicate, 2=wrong offset, 3=other session,
// 4=WAV parser error. CRC/length-invalid packets are silently discarded.
// A packet is acknowledged ONLY after every payload byte has been accepted.
module udp_wav_receiver (
    input wire clk, rst,
    input wire [7:0] rx_data,
    input wire rx_valid,
    input wire [15:0] rx_length,
    input wire tx_ready, tx_ack,
    output wire tx_request, tx_valid,
    output wire [7:0] tx_data,
    output wire [15:0] tx_length,
    output wire [7:0] file_byte,
    output wire file_valid,
    input wire file_ready, file_error,
    output reg [31:0] file_size,
    output reg file_end,
    output reg [31:0] next_offset,
    output reg debug_rx_seen,
    output reg debug_wack_sent,
    output wire [3:0] debug_state,
    output reg [3:0] debug_checks,
    output wire [15:0] debug_length,debug_count
);
    localparam IDLE=0, CHECK=1, LOAD=2, STREAM=3, ACK_MAKE=4,
               ACK_CRC=5, ACK_WAIT=6, ACK_REQUEST=7, ACK_SEND=8;
    reg [3:0] state;
    reg [7:0] packet_ram[0:1023] /* fehdl force_ram=1, ram_style="bram" */;
    reg [7:0] read_byte;
    reg [9:0] rd_addr;
    reg receiving, capture;
    reg [15:0] rx_count, packet_length, gap_count;
    reg [31:0] magic, packet_session, packet_offset, packet_size;
    reg [31:0] crc_rx, crc_trailer;
    reg [10:0] payload_length;
    reg active;
    reg [31:0] session, status;
    reg [159:0] tx_shift;
    reg [31:0] ack_crc;
    reg [4:0] ack_index;
    wire [15:0] byte_index = receiving ? rx_count : 16'd0;
    wire [15:0] current_length = receiving ? packet_length : rx_length;
    wire can_capture = receiving ? capture :
        (state==IDLE && rx_length>=21 && rx_length<=1044);
    wire payload_write = rx_valid && can_capture && byte_index>=16 &&
                         byte_index < current_length-4;
    wire [9:0] wr_addr = byte_index[9:0]-10'd16;
    wire [7:0] ack_crc_byte = tx_shift[159:152];

    function [31:0] crc_byte;
        input [31:0] c; input [7:0] b;
        reg [31:0] v; integer k;
        begin
            v=c^{24'd0,b};
            for(k=0;k<8;k=k+1) v=v[0] ? (v>>1)^32'hedb88320 : v>>1;
            crc_byte=v;
        end
    endfunction
    // Build/check/send the response in one held buffer. No duplicate header
    // register and no payload reset mux; state prevents uninitialized output.
    always @(posedge clk)begin
        if(state==ACK_MAKE)tx_shift<={32'h5741434b,packet_session,next_offset,status,32'd0};
        else if(state==ACK_CRC)begin
            if(ack_index==15)tx_shift<={tx_shift[151:32],tx_shift[159:152],~crc_byte(ack_crc,ack_crc_byte)};
            else tx_shift<={tx_shift[151:32],tx_shift[159:152],32'd0};
        end else if(state==ACK_SEND)tx_shift<={tx_shift[151:0],8'd0};
    end
    // No reset on RAM or its read data register: retain BRAM inference.
    always @(posedge clk) begin
        if(payload_write) packet_ram[wr_addr]<=rx_data;
        if(state==LOAD) read_byte<=packet_ram[rd_addr];
    end
    // Observability only: the most recent CHECK result and live receiver state.
    assign debug_state=state;
    assign debug_length=packet_length;
    assign debug_count=rx_count;
    always @(posedge clk or posedge rst) begin
        if(rst) debug_checks<=0;
        else if(state==CHECK) debug_checks<={magic==32'h57415631,
            crc_trailer==~crc_rx,
            packet_size>=44 && packet_offset<=packet_size &&
            payload_length<=packet_size-packet_offset,1'b1};
    end
    assign file_byte=read_byte;
    assign file_valid=state==STREAM;
    assign tx_request=state==ACK_REQUEST;
    assign tx_valid=state==ACK_SEND;
    assign tx_data=tx_shift[159:152];
    assign tx_length=16'd20;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE; receiving<=0; capture<=0; rx_count<=0;
            packet_length<=0; gap_count<=0; magic<=0; packet_session<=0;
            packet_offset<=0; packet_size<=0; crc_rx<=32'hffffffff;
            crc_trailer<=0; payload_length<=0; active<=0; session<=0;
            file_size<=0; file_end<=0; next_offset<=0; rd_addr<=0;
            status<=0; ack_crc<=0; ack_index<=0;
            debug_rx_seen<=0; debug_wack_sent<=0;
        end else begin
            // Always track packet boundaries, including packets arriving while
            // busy. A retry must never overwrite the packet being streamed.
            if(rx_valid) begin
                // Sticky diagnostic: at least one UDP application byte reached
                // this receiver. Invalid WAV1 packets therefore remain at B.
                debug_rx_seen<=1'b1;
                gap_count<=0;
                if(!receiving) begin
                    packet_length<=rx_length; capture<=can_capture;
                end
                if(current_length==0 || byte_index==current_length-1) begin
                    receiving<=0; rx_count<=0; capture<=0;
                    if(can_capture) begin
                        payload_length<=current_length-20; state<=CHECK;
                    end
                end else begin receiving<=1; rx_count<=byte_index+1'b1; end
                if(can_capture) begin
                    if(byte_index==0) begin
                        magic<={24'd0,rx_data}; crc_rx<=crc_byte(32'hffffffff,rx_data);
                    end else begin
                        if(byte_index<4) magic<={magic[23:0],rx_data};
                        if(byte_index<current_length-4) crc_rx<=crc_byte(crc_rx,rx_data);
                    end
                    if(byte_index>=4 && byte_index<8)
                        packet_session<={packet_session[23:0],rx_data};
                    if(byte_index>=8 && byte_index<12)
                        packet_offset<={packet_offset[23:0],rx_data};
                    if(byte_index>=12 && byte_index<16)
                        packet_size<={packet_size[23:0],rx_data};
                    if(byte_index>=current_length-4)
                        crc_trailer<={crc_trailer[23:0],rx_data};
                end
            end else if(receiving) begin
                gap_count<=gap_count+1'b1;
                if(&gap_count) begin receiving<=0; capture<=0; rx_count<=0; end
            end
            case(state)
                IDLE: ;
                CHECK: begin
                    if(magic!=32'h57415631 || crc_trailer!=~crc_rx ||
                       packet_size<44 || packet_offset>packet_size ||
                       payload_length>packet_size-packet_offset) state<=IDLE;
                    else if(!active && packet_offset==0) begin
                        active<=1; session<=packet_session; file_size<=packet_size;
                        next_offset<=0; file_end<=0; rd_addr<=0; state<=LOAD;
                    end else if(!active) begin status<=2; state<=ACK_MAKE; end
                    else if(packet_session!=session || packet_size!=file_size) begin
                        status<=3; state<=ACK_MAKE;
                    end else if(packet_offset==next_offset && !file_end) begin
                        rd_addr<=0; state<=LOAD;
                    end else if(packet_offset<next_offset &&
                                payload_length<=next_offset-packet_offset) begin
                        status<=file_error ? 4 : 1; state<=ACK_MAKE;
                    end else begin status<=2; state<=ACK_MAKE; end
                end
                LOAD: state<=STREAM;
                STREAM: if(file_ready) begin
                    if({1'b0,rd_addr}+11'd1==payload_length) begin
                        next_offset<=next_offset+payload_length;
                        file_end<=next_offset+payload_length==file_size;
                        status<=file_error ? 4 : 0; state<=ACK_MAKE;
                    end else begin rd_addr<=rd_addr+1'b1; state<=LOAD; end
                end
                ACK_MAKE: begin
                    ack_crc<=32'hffffffff; ack_index<=0; state<=ACK_CRC;
                end
                ACK_CRC: begin
                    ack_crc<=crc_byte(ack_crc,ack_crc_byte);
                    if(ack_index==15) begin
                        state<=ACK_WAIT;
                    end else ack_index<=ack_index+1'b1;
                end
                ACK_WAIT: if(tx_ready) state<=ACK_REQUEST;
                ACK_REQUEST: if(tx_ack) begin ack_index<=0; state<=ACK_SEND; end
                ACK_SEND: begin
                    if(ack_index==19) begin
                        // Sticky diagnostic: the complete 20-byte WACK was
                        // handed to the UDP transmit interface.
                        debug_wack_sent<=1'b1;
                        state<=IDLE;
                    end
                    else ack_index<=ack_index+1'b1;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
