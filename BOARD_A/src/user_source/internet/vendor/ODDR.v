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
// 1.0:Date:2020/10/16 通用ODDR模块建立，支持PH1器件和EG4器件
// web：www.anlogic.com 
//-------------------------------------------------------------------- 
//*******************************************************************/
module ODDR (
        q,
        clk,
        d1,
        d0,
        rst
);
output           q;
input            clk;
input            d1;
input            d0;
input            rst;
parameter ASYNCRST      = "ENABLE";     //ENABLE, DISABLE
parameter DEVICE 		= "PH2";//"PH1" "EG4" "PH1A400"
parameter BANK_TYPE     = "HD"; //"HD","HP"

generate 
if(DEVICE == "EG4")
begin
	EG_LOGIC_ODDR#(
		.ASYNCRST(ASYNCRST)
	) 
	u_oddr
	(
		.q	(q	),
		.clk(clk),
		.d1	(d1	),
		.d0	(d0	),
		.rst(rst)
	);	
end
else if(DEVICE == "PH1" || DEVICE == "PH1A400" )
begin
	PH1_LOGIC_ODDR#(
		.ASYNCRST(ASYNCRST)
	) 
	u_oddr
	(
		.q	(q	),
		.clk(clk),
		.d1	(d1	),
		.d0	(d0	),
		.rst(rst)
	);		
end

else if(DEVICE == "PH1P")
begin
   if (BANK_TYPE   == "HD")begin
	PH1P_LOGIC_HD_ODDR#(
		.ASYNCRST(ASYNCRST)
	) 
	u_oddr
	(
		.q	(q	),
		.clk(clk),
		.d1	(d1	),
		.d0	(d0	),
		.rst(rst)
	);	

end
else if (BANK_TYPE   == "HR")
 begin
	PH1P_LOGIC_ODDR#(
		.ASYNCRST(ASYNCRST)
	) 
	u_oddr
	(
		.q	(q	),
		.clk(clk),
		.d1	(d1	),
		.d0	(d0	),
		.rst(rst)
	);		
end
end

else if(DEVICE == "DR1")
begin
	DR1_LOGIC_ODDR#(
		.ASYNCRST(ASYNCRST)
	) 
	u_oddr
	(
		.q	(q	),
		.clk(clk),
		.d1	(d1	),
		.d0	(d0	),
		.rst(rst)
	);		
end

else if(DEVICE == "DR1P" )
begin
   if (BANK_TYPE   == "HP")begin
       DR1P_LOGIC_HP_OSERDES #(
      .ODDRMODE  ( "DDRX1") 
      )
      u_iddr
      (
        .sclk(clk),
        .pclk(clk),
        .rst(rst),
        .idata({8'd0,d1,d0}),
        .odata(odata)
      );
end
   else begin
       DR1P_LOGIC_HR_OSERDES #(
      .ODDRMODE  ( "DDRX1") 
      )
      u_iddr
      (
        .sclk(clk),
        .pclk(clk),
        .rst(rst),
        .idata({8'd0,d1,d0}),
        .odata(odata )
      );
   end

	
end



else if(DEVICE == "PH2")
begin
   if (BANK_TYPE   == "HD")begin
	   PH2_LOGIC_HD_ODDR#(
	   	.ASYNCRST(ASYNCRST)
	   ) 
	   u_oddr
	   (
	   	.q	(q	),
	   	.clk(clk),
	   	.d1	(d1	),
	   	.d0	(d0	),
	   	.rst(rst)
	   );	
end	
  else if(BANK_TYPE   == "HP")begin
      PH2_LOGIC_HP_ODDR#(
	   	.SRMODE   ("RESET"),
        .RSTMODE("ASYNC"        ),
        .ACTMODE("LOW"          )
	   ) 
	   u_oddr
	   (
	   	.q	(q	),
	   	.clk(clk),
	   	.d1	(d1	),
	   	.d0	(d0	),
	   	.sr(rst)
	   );

   end
end

endgenerate

endmodule