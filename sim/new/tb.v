//================================================
//  University  : UIT - www.uit.edu.vn
//  Course name : System-on-Chip Design
//  Lab name    : lab3
//  File name   : tb.v
//  Author      : Pham Thanh Hung
//  Date        : Oct 21, 2017
//  Version     : 1.0
//-------------------------------------------------
// Modification History
//
//================================================
`timescale 1ns/100ps
module top;
//clock gen
reg clk;
reg rst;
wire [31:0] pc_incr4;
wire[31:0] prg_addr;
wire[31:0] prg_data;
wire[31:0] raddr;
wire[31:0] waddr;
wire[31:0] wdata;
wire we;
wire re;
wire[31:0] rdata;
integer i;
pipeline_processor DUT(
//input
.clk(clk),
.rst(rst),
.prg_data(prg_data),
.dmem_rdata(rdata),
//output
.prg_addr(prg_addr),
.raddr_dmem(raddr),
.waddr_dmem (waddr),
.writedataOut (wdata),
.MemWriteOut_MEM (we),
.MemReadOut_MEM (re)
);
//insert your code here
//--data mem
ram ram(
//input
.wdata(wdata),
.we (we),
.re (re),
.waddr (waddr),
.raddr(raddr),
//output
.rdata(rdata));
//insert your code here
wire[4:0] rs, rt, rd;
wire[31:0] inst;
assign inst       = prg_data;
assign rs         = inst[25:21];
assign rt         = inst[20:16];
assign rd         = inst[15:11];
wire [31:0] r1, r2;
assign r1= DUT.regf[rs];
assign r2= DUT.regf[rt];
rom rom(
//input
.addr(prg_addr),
//output
.data(prg_data)
);
parameter CLK_LO = 10;
always #(CLK_LO) clk=~clk;
initial begin
    //insert your code here
    rst = 1;
    clk = 0;
    #100;
    rst =0;
end
initial begin
    $monitor (DUT.regf[8], DUT.regf[9]);
    $monitor (rd, r1, r2);
    for (i=0;i<32;i=i+1) DUT.regf[i]=0;  
    @(posedge clk);
    $display (DUT.regf[rt], r1+DUT.imm_signed);
    if(DUT.regf[rt] != r1+DUT.imm_signed) begin
       $display("ERROR.Command ADDI - EXP:%h, ACT:%h.",r1+DUT.imm_signed,DUT.regf[rt]);
       $display("End by testcase");
       $finish();
    end   
    @(posedge clk);
    if(DUT.regf[rd] != (r1+r2)) begin
       $display("ERROR.Command ADD - EXP:%h, ACT:%h.",r1+r2,DUT.regf[rd]);
       $display("End by testcase");
       $finish();
       end 
    $display("PASSED");
    #600;  
    $finish();
end
endmodule