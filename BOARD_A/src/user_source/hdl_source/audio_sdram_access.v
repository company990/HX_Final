// Anlogic sdr_as_ram requires aligned BL4 requests. Store each logical PCM
// frame in four identical physical words (8192 frames occupy 128 KiB).
// v_pause stops new video bursts; v_busy covers an active burst and its tail.
module audio_sdram_access #(
    parameter ADDRW=21, DATAW=32, DMW=4, AADDRW=14,
    parameter [ADDRW-1:0] AUDIO_BASE=21'd655360,
    parameter TAGDEPTH=64, TAGW=6, TIME_EXT=0
)(
    output wire audio_idle,
    input wire clk,rst,
    output wire App_wr_en,
    output wire [ADDRW-1:0] App_wr_addr,
    output wire [DMW-1:0] App_wr_dm,
    output wire [DATAW-1:0] App_wr_din,
    output wire App_rd_en,
    output wire [ADDRW-1:0] App_rd_addr,
    input wire Sdr_rd_en,
    input wire [DATAW-1:0] Sdr_rd_dout,
    input wire Sdr_init_done,
    input wire Sdr_init_ref_vld,Sdr_busy,
    input wire v_wr_en,
    input wire [ADDRW-1:0] v_wr_addr,
    input wire [DMW-1:0] v_wr_dm,
    input wire [DATAW-1:0] v_wr_din,
    input wire v_rd_en,
    input wire [ADDRW-1:0] v_rd_addr,
    input wire v_busy,
    output wire v_pause,
    output wire v_rd_valid,
    output wire [DATAW-1:0] v_rd_data,
    input wire a_wr_req,
    input wire [AADDRW-1:0] a_wr_addr,
    input wire [DATAW-1:0] a_wr_data,
    output wire a_wr_ready,
    input wire a_rd_req,
    input wire [AADDRW-1:0] a_rd_addr,
    output wire a_rd_valid,
    output wire [DATAW-1:0] a_rd_data,
    input t_wr_req,input [15:0] t_wr_block,input [31:0] t_wr_data,
    output [1:0] t_wr_word,output t_wr_active,t_wr_ack,
    input t_rd_req,input [15:0] t_rd_block,output t_rd_valid,
    output [1:0] t_rd_word,output [31:0] t_rd_data,output reg t_fault
);
    localparam IDLE=0,CLAIM=1,WRITE=2,WTAIL=3,WACK=4,READ=5,RWAIT=6,FAIR=7,RECOVER=8;
    reg [3:0] state;
    reg read_pending,do_read;
    wire audio_response=(state==READ || state==RWAIT);
    reg timeshift,prefer_ts,t_pending;reg [15:0] t_pending_addr;
    reg [11:0] watchdog;
    wire t_request=TIME_EXT && (t_pending || t_wr_req);
    wire select_ts=t_request && (!(read_pending || a_wr_req) || prefer_ts);
    reg [5:0] delay_count;
    reg [1:0] beat,returned;
    assign audio_idle = (state==IDLE) && !read_pending && !a_wr_req && !a_rd_req && !t_request;

    reg [AADDRW-1:0] pending_addr;
    reg [ADDRW-1:0] burst_addr;
    reg [DATAW-1:0] write_data;
    wire owns_port=(state==WRITE || state==WTAIL || state==WACK || state==READ || state==RWAIT || state==RECOVER);
    assign v_pause=(state!=IDLE && state!=FAIR);
    assign App_wr_en=owns_port ? (state==WRITE) : v_wr_en;
    assign App_wr_addr=owns_port ? (burst_addr+beat) : v_wr_addr;
    // Retain data after App_wr_en falls, through the write pipeline tail.
    assign App_wr_din=owns_port ? (timeshift ? t_wr_data : write_data) : v_wr_din;
    assign App_wr_dm=owns_port ? {DMW{1'b0}} : v_wr_dm;
    assign App_rd_en=owns_port ? (state==READ) : v_rd_en;
    assign App_rd_addr=owns_port ? (burst_addr+beat) : v_rd_addr;
    assign a_wr_ready=(state==WACK) && !timeshift;
    assign t_wr_ack=TIME_EXT && timeshift && state==WACK;
    assign t_wr_word=beat;
    assign t_wr_active=TIME_EXT && timeshift && (state==WRITE || state==WTAIL || state==WACK);
    assign t_rd_valid=TIME_EXT && timeshift && audio_response && Sdr_rd_en;
    assign t_rd_word=returned;assign t_rd_data=Sdr_rd_dout;
    assign a_rd_valid=audio_response && !timeshift && Sdr_rd_en && returned==0;
    assign a_rd_data=Sdr_rd_dout;
    assign v_rd_valid=!audio_response && state!=RECOVER && Sdr_rd_en;
    assign v_rd_data=Sdr_rd_dout;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;delay_count<=0;beat<=0;returned<=0;
            read_pending<=0;pending_addr<=0;do_read<=0;
            burst_addr<=0;write_data<=0;timeshift<=0;prefer_ts<=1;t_pending<=0;t_pending_addr<=0;watchdog<=0;t_fault<=0;
        end else begin
            t_fault<=0;
            if(TIME_EXT && t_rd_req)begin t_pending<=1;t_pending_addr<=t_rd_block;end
            if(timeshift && (state==CLAIM || state==RWAIT))watchdog<=watchdog+1'b1;else watchdog<=0;
            if(a_rd_req) begin read_pending<=1;pending_addr<=a_rd_addr;end
            case(state)
                IDLE: if(read_pending || a_wr_req || t_request) begin
                    state<=CLAIM;delay_count<=0;timeshift<=select_ts;prefer_ts<=!select_ts;
                    do_read<=select_ts ? t_pending : read_pending;
                    burst_addr<=select_ts ? {3'b111,(t_pending ? t_pending_addr : t_wr_block),2'b00} : AUDIO_BASE +
                        (read_pending ? {{(ADDRW-AADDRW-2){1'b0}},pending_addr,2'b00} :
                                        {{(ADDRW-AADDRW-2){1'b0}},a_wr_addr,2'b00});
                    write_data<=a_wr_data;
                    if(select_ts)begin if(t_pending)t_pending<=0;end
                    else if(read_pending) read_pending<=0;
                end
                CLAIM: begin
                    if(v_busy || v_wr_en || v_rd_en || !Sdr_init_done ||
                       Sdr_init_ref_vld || Sdr_busy) delay_count<=0;
                    else if(delay_count==16) begin
                        state<=do_read ? READ : WRITE;beat<=0;returned<=0;
                    end else delay_count<=delay_count+1'b1;
                end
                WRITE: if(beat==3) begin state<=WTAIL;delay_count<=0;end
                       else beat<=beat+1'b1;
                WTAIL: if(delay_count==16) state<=WACK;
                       else delay_count<=delay_count+1'b1;
                WACK: begin state<=FAIR;delay_count<=0;end
                READ: if(beat==3) state<=RWAIT;else beat<=beat+1'b1;
                RWAIT: if(Sdr_rd_en) begin
                    returned<=returned+1'b1;
                    if(returned==3) begin state<=FAIR;delay_count<=0;end
                end
                RECOVER: begin
                    if(Sdr_rd_en || Sdr_busy)delay_count<=0;
                    else if(delay_count==63)begin state<=FAIR;delay_count<=0;end
                    else delay_count<=delay_count+1'b1;
                end
                // Give video an opportunity to start between audio operations.
                FAIR: if(delay_count==31) state<=IDLE;
                      else delay_count<=delay_count+1'b1;
                default: state<=IDLE;
            endcase
            if(TIME_EXT && timeshift && watchdog==4095 && (state==CLAIM || state==RWAIT))begin
                t_fault<=1;state<=state==RWAIT ? RECOVER : FAIR;delay_count<=0;watchdog<=0;
            end
        end
    end
endmodule
