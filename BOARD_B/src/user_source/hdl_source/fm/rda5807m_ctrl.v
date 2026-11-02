`timescale 1ns/1ps
// STM32 main.c / rda5807m.c initialization port. Frequency units are 100 kHz:
// 968 means 96.8 MHz. Sequential I2C address is 7'h10 (wire bytes 20/21).
// INIT_TABLE selects the supplied STM32 table 0..3; default 0 is RDA5807M.
// Commands are accepted only on cmd_valid && cmd_ready, in this clock domain.
module rda5807m_ctrl #(
    parameter integer CLK_HZ=25000000,
    parameter integer I2C_HZ=100000,
    parameter integer DEFAULT_FREQUENCY=968,
    parameter integer DEFAULT_VOLUME=15,
    parameter integer INIT_TABLE=0,
    parameter integer MS_CYCLES=CLK_HZ/1000
)(
    input wire clk,rst,
    input wire cmd_valid,
    output wire cmd_ready,
    input wire [2:0] cmd_op, // original 0..3; 4 seek up, 5 seek down, 6 power
    input wire [15:0] cmd_value,
    output wire busy,
    output reg init_done,comm_ok,tuned,
    output reg [3:0] error,
    output reg [15:0] frequency,
    output reg [3:0] volume,
    output reg mute,
    output reg powered,
    output reg [15:0] status_a,status_b,
    output wire [79:0] debug_status,
    inout wire fm_scl,fm_sda
);
    localparam PROBE=1,RESET_WRITE=2,RESET_WAIT=3,ZERO_WRITE=4,ZERO_WAIT=5,
               WAKE_WRITE=6,WAKE_WAIT=7,TABLE_WRITE=8,TABLE_WAIT=9,
               CONFIG=10,CONFIG_WRITE=11,CONFIG_WAIT=12,VOLUME_WRITE=13,
               TUNE_PREP=14,TUNE_WRITE=15,POLL_READ=16,POLL_CHECK=17,
               TUNE_FINISH=18,IDLE=19,CONTROL_WRITE=20,FAULT=21,
               SEEK_PREP=22,SEEK_WRITE=23,SEEK_STOP=24,POWER_WRITE=25,OFF=26,
               IO_WAIT=30,DELAY_WAIT=31;
    localparam MW=MS_CYCLES<2 ? 1 : $clog2(MS_CYCLES);
    reg [MW-1:0] ms_counter;
    wire ms_tick=ms_counter==MS_CYCLES-1;
    reg [7:0] state,resume_state;
    reg [9:0] delay_left;
    reg [12:0] tune_left;
    reg [7:0] status_wait;
    reg initializing,tuning;
    reg seeking,seek_up;
    reg force_mono,softmute_enabled,applied_mono,applied_softmute;
    reg [15:0] applied_frequency;
    reg [3:0] applied_volume;
    reg applied_mute;
    reg [15:0] reg02,reg03,reg04,reg05,reg06,reg07;
    reg [2:0] write_kind;
    reg io_start,io_read;
    reg [6:0] io_length;
    wire [6:0] byte_index;
    wire [6:0] read_index;
    wire [7:0] read_data,table_byte;
    wire [6:0] table_length;
    wire [95:0] table_regs;
    wire io_done,io_busy,read_valid;
    wire [2:0] io_error;
    reg [7:0] read_hi,write_data;
    reg [15:0] selected_reg;
    rda5807m_init_rom #(.TABLE(INIT_TABLE)) init_rom(
        .index(byte_index),.data(table_byte),.length(table_length),.first_regs(table_regs));
    always @* begin
        case(byte_index[3:1])
            0:selected_reg=reg02;1:selected_reg=reg03;2:selected_reg=reg04;
            3:selected_reg=reg05;4:selected_reg=reg06;default:selected_reg=reg07;
        endcase
        case(write_kind)
            0:write_data=byte_index==1 ? 8'h02 : 8'h00; // reset 0002,0000
            1:write_data=0;                            // reset release 0000,0000
            2:write_data=byte_index==0 ? 8'hc0 : 8'h01; // wake C001
            3:write_data=table_byte;
            default:write_data=byte_index[0] ? selected_reg[7:0] : selected_reg[15:8];
        endcase
    end
    fm_i2c_master #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ)) i2c(
        .clk(clk),.rst(rst),.start(io_start),.read_transfer(io_read),
        .address(7'h10),.length(io_length),.write_data(write_data),
        .byte_index(byte_index),.read_data(read_data),.read_index(read_index),.read_valid(read_valid),
        .busy(io_busy),.done(io_done),.error(io_error),.scl(fm_scl),.sda(fm_sda));
    assign cmd_ready=state==IDLE || state==FAULT || state==OFF;
    assign busy=!cmd_ready;
    // FLAGS, ERROR, STAGE, FREQ, VOL, MUTE, reserved byte, 0AH, 0BH.
    assign debug_status={{busy,init_done,comm_ok,tuned},error,state,applied_frequency,
                         applied_volume,{3'b0,applied_mute},{5'd0,applied_softmute,applied_mono,powered},status_a,status_b};
    task launch;
        input rd;
        input [6:0] bytes;
        input [2:0] kind;
        input [7:0] successor;
        begin io_start<=1;io_read<=rd;io_length<=bytes;write_kind<=kind;
              resume_state<=successor;state<=IO_WAIT;end
    endtask
    task delay_ms;
        input [9:0] milliseconds;
        input [7:0] successor;
        begin delay_left<=milliseconds;resume_state<=successor;state<=DELAY_WAIT;end
    endtask
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            ms_counter<=0;state<=DELAY_WAIT;resume_state<=PROBE;delay_left<=100;
            tune_left<=0;status_wait<=250;initializing<=1;tuning<=0;
            seeking<=0;seek_up<=1;powered<=0;applied_frequency<=0;applied_volume<=0;applied_mute<=1;
            force_mono<=0;softmute_enabled<=1;applied_mono<=0;applied_softmute<=1;
            io_start<=0;io_read<=0;io_length<=0;write_kind<=0;
            init_done<=0;comm_ok<=0;tuned<=0;error<=0;
            frequency<=DEFAULT_FREQUENCY;volume<=DEFAULT_VOLUME;mute<=0;
            status_a<=0;status_b<=0;read_hi<=0;
            reg02<=0;reg03<=0;reg04<=0;reg05<=0;reg06<=0;reg07<=0;
        end else begin
            io_start<=0;
            if(ms_tick) ms_counter<=0;else ms_counter<=ms_counter+1'b1;
            if(ms_tick && tuning && tune_left!=0) tune_left<=tune_left-1'b1;
            if(read_valid) begin
                if(!read_index[0]) read_hi<=read_data;
                else if(read_index==1) status_a<={read_hi,read_data};
                else if(read_index==3) status_b<={read_hi,read_data};
            end
            case(state)
                PROBE: launch(1,2,0,RESET_WRITE);
                RESET_WRITE: launch(0,4,0,RESET_WAIT);
                RESET_WAIT: delay_ms(30,ZERO_WRITE);
                ZERO_WRITE: launch(0,4,1,ZERO_WAIT);
                ZERO_WAIT: delay_ms(30,WAKE_WRITE);
                WAKE_WRITE: launch(0,2,2,WAKE_WAIT);
                WAKE_WAIT: delay_ms(600,TABLE_WRITE);
                TABLE_WRITE: launch(0,table_length,3,TABLE_WAIT);
                TABLE_WAIT: delay_ms(50,CONFIG);
                CONFIG: begin
                    // Exactly the STM32 main.c overrides: internal 32K crystal,
                    // stereo, no bass/RDS, 87..108 MHz at 100 kHz spacing.
                    reg02<=(table_regs[95:80] & 16'h8b83) | 16'h8001 |
                           (mute ? 16'h0000 : 16'h4000) | (force_mono ? 16'h2000 : 16'h0000);
                    reg03<=table_regs[79:64] & 16'hfff0;
                    reg04<=(table_regs[63:48] & 16'hfdff) | (softmute_enabled ? 16'h0200 : 16'h0000);
                    reg05<={table_regs[47:36],volume};
                    reg06<=table_regs[31:16];reg07<=table_regs[15:0];
                    state<=CONFIG_WRITE;
                end
                CONFIG_WRITE: launch(0,12,4,CONFIG_WAIT);
                CONFIG_WAIT: delay_ms(50,VOLUME_WRITE);
                VOLUME_WRITE: launch(0,12,4,TUNE_PREP);
                TUNE_PREP: begin
                    if(frequency<870 || frequency>1080) begin error<=6;state<=FAULT;end
                    else begin
                        seeking<=0;
                        reg03<=((frequency-16'd870)<<6) | 16'h0010;
                        reg02<=16'h8001 | (mute ? 16'h0000 : 16'h4000) | (force_mono ? 16'h2000 : 16'h0000);
                        tuned<=0;tuning<=1;state<=TUNE_WRITE;
                    end
                end
                TUNE_WRITE: begin tune_left<=800;launch(0,12,4,POLL_READ);end
                SEEK_PREP:begin
                    reg02<=(reg02 & 16'hfc7f) | 16'h0101 | (seek_up ? 16'h0200 : 16'h0000);
                    reg03[4]<=0;tuned<=0;tuning<=1;seeking<=1;state<=SEEK_WRITE;
                end
                SEEK_WRITE:begin tune_left<=5000;launch(0,12,4,POLL_READ);end
                SEEK_STOP:launch(0,12,4,FAULT);
                POWER_WRITE:launch(0,12,4,OFF);
                POLL_READ: begin
                    if(tune_left==0) begin error<=5;tuning<=0;reg03[4]<=0;reg02[8]<=0;state<=seeking ? SEEK_STOP : FAULT;end
                    else launch(1,4,0,POLL_CHECK);
                end
                POLL_CHECK: begin
                    if(status_a[14]) begin
                        // TUNE is automatically cleared by the chip. Mirror
                        // that in the shadow so volume writes do not retune.
                        reg03[4]<=0;reg02[8]<=0;tuning<=0;
                        if(seeking && status_a[13])begin error<=8;state<=SEEK_STOP;end
                        else if(status_a[9:0]>210)begin error<=6;state<=FAULT;end
                        else begin
                            if(seeking)begin frequency<=870+status_a[9:0];reg03<={status_a[9:0],6'd0};end
                            delay_ms(60,TUNE_FINISH);
                        end
                    end else delay_ms(10,POLL_READ);
                end
                TUNE_FINISH: begin
                    powered<=1;init_done<=1;initializing<=0;tuned<=1;error<=0;
                    seeking<=0;applied_frequency<=870+status_a[9:0];
                    status_wait<=250;state<=IDLE;
                end
                CONTROL_WRITE: launch(0,12,4,IDLE);
                IO_WAIT: if(io_done) begin
                    comm_ok<=io_error==0;
                    if(io_error!=0) begin
                        error<={1'b0,io_error};tuning<=0;reg03[4]<=0;reg02[8]<=0;
                        if(initializing) init_done<=0;
                        state<=resume_state==IDLE ? IDLE : FAULT;
                    end else begin
                        if(!io_read && write_kind==4)begin
                         applied_volume<=volume;applied_mute<=mute;
                         applied_mono<=reg02[13];applied_softmute<=reg04[9];
                        end
                        if(resume_state==OFF)begin powered<=0;status_a<=0;status_b<=0;end
                        state<=resume_state;
                    end
                end
                DELAY_WAIT: if(ms_tick) begin
                    if(delay_left<=1) state<=resume_state;
                    else delay_left<=delay_left-1'b1;
                end
                IDLE,FAULT,OFF: begin
                    if(cmd_valid) begin
                        error<=0;status_wait<=250;
                        case(cmd_op)
                            0: if(cmd_value>=870 && cmd_value<=1080) begin
                                if(init_done) begin frequency<=cmd_value;state<=TUNE_PREP;end
                                else error<=7;
                            end else error<=6;
                            1: if(cmd_value<=15) begin
                                if(init_done) begin volume<=cmd_value[3:0];reg05[3:0]<=cmd_value[3:0];state<=CONTROL_WRITE;end
                                else error<=7;
                            end else error<=6;
                            2: if(cmd_value<=1) begin
                                if(init_done) begin mute<=cmd_value[0];reg02[14]<=!cmd_value[0];state<=CONTROL_WRITE;end
                                else error<=7;
                            end else error<=6;
                            3: begin init_done<=0;comm_ok<=0;tuned<=0;initializing<=1;tuning<=0;seeking<=0;delay_ms(100,PROBE);end
                            4,5:if(init_done && powered)begin seek_up<=cmd_op==4;state<=SEEK_PREP;end else error<=7;
                            6:if(cmd_value<=1)begin
                                init_done<=0;tuned<=0;seeking<=0;tuning<=0;
                                if(cmd_value[0])begin initializing<=1;comm_ok<=0;delay_ms(100,PROBE);end
                                else begin reg02[0]<=0;reg02[8]<=0;reg03[4]<=0;state<=POWER_WRITE;end
                            end else error<=6;
                            // Scanner op7/value0 reads fresh 0A/0B; tagged 010x sets reception policy.
                            7:if(cmd_value>=16'h0100 && cmd_value<=16'h0103)begin
                                if(init_done && powered)begin
                                    force_mono<=cmd_value[0];softmute_enabled<=cmd_value[1];
                                    reg02[13]<=cmd_value[0];reg04[9]<=cmd_value[1];state<=CONTROL_WRITE;
                                end else error<=7;
                            end else if(cmd_value==0)begin
                                if(init_done && powered)launch(1,4,0,IDLE);else error<=7;
                            end else error<=6;
                            default:error<=6;
                        endcase
                    end else if(state==IDLE && ms_tick) begin
                        if(status_wait==0) begin status_wait<=250;launch(1,4,0,IDLE);end
                        else status_wait<=status_wait-1'b1;
                    end
                end
                default: begin error<=7;state<=FAULT;tuning<=0;end
            endcase
        end
    end
endmodule
