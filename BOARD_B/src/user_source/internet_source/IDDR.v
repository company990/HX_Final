`timescale 1ns / 1ps
//******************************************************************** 
// -------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>Copyright Notice<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
// ------------------------------------------------------------------- 
//             /\ --------------- 
//            /  \ ------------- 
//           / /\ \ -----------
//          / /  \ \ ---------
//         / /    \ \ ------- 
//        / /      \ \ ----- 
//       / /_ _ _   \ \ --- 
//      /_ _ _ _ _\  \_\ -
//********************************************************************
// Author: suluyang 
// Email:luyang.su@anlogic.com 
// Date:2020/10/16 
// Description: 
// 1.0:Date:2020/10/16 通用IDDR模块建立，支持PH1器件和EG4器件
// web：www.anlogic.com 
//-------------------------------------------------------------------- 
//*******************************************************************/
module IDDR (
        q1,
        q0,
        clk,
        d,
        rst
);
output           q1;
output           q0;
input            clk;
input            d;
input            rst;
parameter ASYNCRST      = "ENABLE";     //ENABLE, DISABLE
parameter PIPEMODE      = "PIPED";      //PIPED, NONE
parameter DEVICE 		= "PH1";//"PH1" "EG4" "PH1A400"
parameter BANK_TYPE     = "HD"; //"HD","HP"

generate 
if(DEVICE == "EG4")
begin
	EG_LOGIC_IDDR #(
		.ASYNCRST(ASYNCRST),     //ENABLE, DISABLE
		.PIPEMODE(PIPEMODE)      //PIPED, NONE
	)
	u_iddr
	(
		.q1	(q1	),
		.q0	(q0	),
		.clk(clk),
		.d	(d	),
		.rst(rst)
	);		
end
else if(DEVICE == "PH1" || DEVICE == "PH1A400" )
begin
	PH1_LOGIC_IDDR #(
		.ASYNCRST(ASYNCRST),     //ENABLE, DISABLE
		.PIPEMODE(PIPEMODE)      //PIPED, NONE
	)
	u_iddr
	(
		.q1	(q1	),
		.q0	(q0	),
		.clk(clk),
		.d	(d	),
		.rst(rst)
	);		
end

else if(DEVICE == "PH1P" )
begin 
  if (BANK_TYPE   == "HD")begin
  
	PH1P_LOGIC_HD_IDDR #(
		.ASYNCRST(ASYNCRST),     //ENABLE, DISABLE
		.MODE("SAME_EDGE_PIPE")      //PIPED, NONE
	)
	u_iddr
	(
		.q1	(q1	),
		.q0	(q0	),
		.clk(clk),
		.d	(d	),
		.rst(rst)
	);	
end	
else if (BANK_TYPE   == "HR")
  begin
	PH1P_LOGIC_IDDR #(
		.ASYNCRST(ASYNCRST),     //ENABLE, DISABLE
		.PIPEMODE(PIPEMODE)      //PIPED, NONE
	)
	u_iddr
	(
		.q1	(q1	),
		.q0	(q0	),
		.clk(clk),
		.d	(d	),
		.rst(rst)
	);	

end

end

else if(DEVICE == "DR1" )
begin 
  if (BANK_TYPE   == "HD")begin
	DR1_LOGIC_HD_IDDR #(
		.ASYNCRST(ASYNCRST),     //ENABLE, DISABLE
		.MODE("SAME_EDGE_PIPE")   // "SAME_EDGE_PIPE","SAME_EDGE","OPPOSITE_EDGE"
	)
	u_iddr
	(
		.q1	(q1	),
		.q0	(q0	),
		.clk(clk),
		.d	(d	),
		.rst(rst)
	);	
end
else  begin
  	DR1_LOGIC_IDDR #(
		.ASYNCRST(ASYNCRST),     //ENABLE, DISABLE
		.PIPEMODE(PIPEMODE)      //PIPED, NONE
	)
	u_iddr
	(
		.q1	(q1	),
		.q0	(q0	),
		.clk(clk),
		.d	(d	),
		.rst(rst)
	);
end

end
else if(DEVICE == "DR1P" )
begin
wire [9:0]  odata; 
   if (BANK_TYPE   == "HP")begin
       DR1P_LOGIC_HP_ISERDES #(
      .IDDRMODE  ( "DDRX1") 
      )
      u_iddr
      (
        .sclk(clk),
        .pclk(clk),
        .rst(rst),
        .idata(d),
        .odata(odata)
      );
end
   else begin
       DR1P_LOGIC_HR_ISERDES #(
      .IDDRMODE  ( "DDRX1") 
      )
      u_iddr
      (
        .sclk(clk),
        .pclk(clk),
        .rst(rst),
        .idata(d),
        .odata(odata )
      );
   end
assign q1=odata[1];
assign q0=odata[0];
	
end


else if(DEVICE == "PH2" )
begin
   if (BANK_TYPE   == "HD")begin
	PH2_LOGIC_HD_IDDR #(
		.ASYNCRST(ASYNCRST),     //ENABLE, DISABLE
		.MODE("SAME_EDGE_PIPE")      // "SAME_EDGE_PIPE","SAME_EDGE","OPPOSITE_EDGE"
	)
	u_iddr
	(
		.q1	(q1	),
		.q0	(q0	),
		.clk(clk),
		.d	(d	),
		.rst(rst)
	);	
end
  else if(BANK_TYPE   == "HP")begin
	PH2_LOGIC_HP_IDDR #(
	   	.MODE   ("SAME_EDGE_PIPE"),
        .RSTMODE("ASYNC"        ),
        .ACTMODE("LOW"          )
	)
	u_iddr
	(
		.q1	(q1	),
		.q0	(q0	),
		.clk(clk),
		.d	(d	),
		.sr(rst)
	);	
end
	
end
endgenerate

endmodule