//================================================
//  University  : UIT - www.uit.edu.vn
//  Course name : System-on-Chip Design
//  Lab name    : lab1
//  File name   : pipeline_processor.v
//  Author      : Pham Thanh Hung
//  Date        : Oct 21, 2017
//  Version     : 1.0
//-------------------------------------------------
// Modification History
//
//=================================================

module pipeline_processor(
//input
clk,rst,
prg_data,
dmem_rdata,
//output
prg_addr,
raddr_dmem,
waddr_dmem,
writedataOut,
MemWriteOut_MEM,
MemReadOut_MEM,
pc_incr4
);

parameter R_OP =  6'b00_0000;
parameter LW_OP =  6'b10_0011 ;
parameter SW_OP =  6'b10_1011;
parameter BEQ_OP =  6'b00_0100;
parameter ADDI_OP = 6'b00_1000;
parameter LUI_OP = 6'b00_1111;

parameter ADD_FUNC = 6'b10_0000;
parameter SUB_FUNC =  6'b10_0010;
parameter AND_FUNC =  6'b10_0100;
parameter OR_FUNC =  6'b10_0101;
parameter SLT_FUNC =  6'b10_1010;

//input
input clk; //main clock
input rst; //reset
input [31:0] prg_data;
input [31:0] dmem_rdata; //data memory


//output
output [31:0] prg_addr;
output [31:0] raddr_dmem;//data memory
output [31:0] waddr_dmem;//data memory
output [31:0] writedataOut;//data memory
output MemWriteOut_MEM;//data memory
output reg MemReadOut_MEM;//data memory
output [31:0] pc_incr4;

//User declaration
wire [31:0] inst; //instruction
//Register file declaration
reg [31:0] regf [0:31]; //32 registers

wire [4:0] raddr1_regf;
wire [4:0] raddr2_regf;
wire [4:0] waddr_regf;

wire [31:0] wdata_regf;
reg [31:0] regf_rdata1;
reg [31:0] regf_rdata2;

reg write_regf;

//Data Memory Controller
reg [31:0] dmem [1023:0];
wire [31:0] raddr_dmem;
wire [31:0] waddr_dmem;
reg [31:0] wdata_dmem;
wire [31:0] dmem_rdata;

reg write_dmem;
reg read_dmem;

//ALU declaration


wire [31:0] op1_alu;
wire [31:0] op2_alu;
reg [31:0] alu_result;
reg [31:0] branch;
reg Zero;
//Data path declaration
reg [31:0] pc;

wire [31:0] pc_temp;
wire [4:0] rs;
wire [4:0] rt;
wire [4:0] rd;
wire [15:0] imm;
wire [31:0] imm_signed;

//Control unit declaration
reg alu_src;
reg [2:0] alu_op;
reg reg_dst;
reg mem_to_reg;
wire pc_src;
reg is_branch;

//if_id pipeline
reg [31:0] pc_out;
reg [31:0] instruction_out;
//id_ex pipeline
reg [31:0] PCplus4out;
reg [31:0] ReadData1_out, ReadData2_out;
reg [31:0] SignExtendResult_out;
reg [4:0] rtOut, rdOut;
reg RegWriteOut, MemtoRegOut, MemWriteOut, MemReadOut, ALUSrcOut, RegDstOut, BranchOut;
reg [2:0] ALUOpOut;
//ex_mem pipeline
reg [31:0] RegWriteOut_MEM, MemWriteOut_MEM, MemReadOut_MEM;
reg [31:0] ALUresultOut;
reg [31:0] writedataOut;
reg Branch_final, MemtoRegOut_MEM;
reg [4:0] waddr_regf_out;
//mem_wb pipeline
reg RegWriteOut_Final, MemtoRegOut_Final;
reg [31:0] readDataOut, ALUresultOut_Final;
reg [4:0] writeRegOut_Final;
//---------------------------
//Register File
always @(*) begin
    if(RegWriteOut_Final) regf[writeRegOut_Final] <= wdata_regf; 
    regf_rdata1 <= regf[raddr1_regf];
    regf_rdata2 <= regf[raddr2_regf];
end
//---------------------------
//Data Mem
assign raddr_dmem = ALUresultOut;
assign waddr_dmem = ALUresultOut;
//assign wdata_dmem = regf_rdata2;

//---------------------------
//ALU operation
always @(*) begin
    branch = pc_out + (SignExtendResult_out << 2);
    end
assign pc_temp = branch;

// ALU
always @(*) begin
    if (ALUOpOut == 3'b000) begin
        alu_result = op1_alu & op2_alu;
    end else if (ALUOpOut == 3'b001) begin
        alu_result = op1_alu | op2_alu;
    end else if (ALUOpOut == 3'b010) begin
        alu_result = op1_alu + op2_alu;
    end else if (ALUOpOut == 3'b011) begin
        alu_result = op2_alu;
    end else if (ALUOpOut == 3'b110) begin
        alu_result = op1_alu - op2_alu;
    end else if (ALUOpOut == 3'b111) begin
        alu_result = (op1_alu < op2_alu) ? 1 : 0;
    end
assign Zero = (ALUresultOut == 0) ? 1 : 0;
end
//---------------------------
// Data Path

assign rs = instruction_out[25:21];
assign rt = instruction_out[20:16];
assign rd = instruction_out[15:11];
assign imm = instruction_out[15:0];
assign imm_signed = {{16{imm[15]}},imm[15:0]};

//To Reg File
assign raddr1_regf = rs;
assign raddr2_regf = rt;
assign waddr_regf = (RegDstOut == 1) ? rdOut : rtOut;
assign wdata_regf = (MemtoRegOut_Final == 1) ? readDataOut : ALUresultOut_Final;

// To ALU
assign op1_alu = ReadData1_out;
assign op2_alu = (ALUSrcOut == 1) ? SignExtendResult_out : ReadData2_out;

//PC
always @(posedge clk or negedge rst) begin
  if (rst) begin
    pc <= 0;
    reg_dst = 0;
    alu_src = 0;
    write_regf = 0;
    write_dmem = 0;
    read_dmem = 0;
    mem_to_reg = 0;
    alu_op = 0;
    is_branch = 0;
    alu_result = 0;
  end else if (pc_src) begin
    pc <= pc_temp;
    end else pc <= pc_incr4;
end

assign pc_incr4 = pc + 4;

// Control unit
always @(*) begin
    case (instruction_out[31:26])
        R_OP: begin
            case (instruction_out[5:0])
                ADD_FUNC: begin
                    reg_dst     = 1'b1;
                    write_regf  = 1'b1;
                    alu_src     = 1'b0;
                    alu_op      = 3'b010;
                    write_dmem  = 1'b0;
                    read_dmem   = 1'b0;
                    mem_to_reg  = 1'b0;
                    is_branch   = 1'b0;
                end
                SUB_FUNC:begin
                    reg_dst     = 1'b1;
                    write_regf  = 1'b1;
                    alu_src     = 1'b0;
                    alu_op      = 3'b110;
                    write_dmem  = 1'b0;
                    read_dmem   = 1'b0;
                    mem_to_reg  = 1'b0;
                    is_branch   = 1'b0;
                end
                AND_FUNC:begin
                    reg_dst     = 1'b1;
                    write_regf  = 1'b1;
                    alu_src     = 1'b0;
                    alu_op      = 3'b000;
                    write_dmem  = 1'b0;
                    read_dmem   = 1'b0;
                    mem_to_reg  = 1'b0;
                    is_branch   = 1'b0;
                end
                OR_FUNC:begin
                    reg_dst     = 1'b1;
                    write_regf  = 1'b1;
                    alu_src     = 1'b0;
                    alu_op      = 3'b001;
                    write_dmem  = 1'b0;
                    read_dmem   = 1'b0;
                    mem_to_reg  = 1'b0;
                    is_branch   = 1'b0;
                end
                SLT_FUNC:begin
                    reg_dst     = 1'b1;
                    write_regf  = 1'b1;
                    alu_src     = 1'b0;
                    alu_op      = 3'b111;
                    write_dmem  = 1'b0;
                    read_dmem   = 1'b0;
                    mem_to_reg  = 1'b0;
                    is_branch   = 1'b0;
                end
            endcase
        end
        LW_OP:begin
            reg_dst     = 1'b0;
            write_regf  = 1'b1;
            alu_src     = 1'b1;
            alu_op      = 3'b010;
            write_dmem  = 1'b0;
            read_dmem   = 1'b1;
            mem_to_reg  = 1'b1;
            is_branch   = 1'b0;
        end
        SW_OP:begin
            reg_dst     = 1'bx;
            write_regf  = 1'b0;
            alu_src     = 1'b1;
            alu_op      = 3'b010;
            write_dmem  = 1'b1;
            read_dmem   = 1'b0;
            mem_to_reg  = 1'bx;
            is_branch   = 1'b0;
        end
        BEQ_OP:begin
            reg_dst     = 1'bx;
            write_regf  = 1'b0;
            alu_src     = 1'b0;
            alu_op      = 3'b110;
            write_dmem  = 1'b0;
            read_dmem   = 1'b0;
            mem_to_reg  = 1'bx;
            is_branch   = 1'b1;
        end
        ADDI_OP:begin
            reg_dst     = 1'b0;
            write_regf  = 1'b1;
            alu_src     = 1'b1;
            alu_op      = 3'b010;
            write_dmem  = 1'b0;
            read_dmem   = 1'b0;
            mem_to_reg  = 1'b0;
            is_branch   = 1'b0;
        end
        LUI_OP:begin
            reg_dst     = 1'b0;
            write_regf  = 1'b1;
            alu_src     = 1'b1;
            alu_op      = 3'b011;
            write_dmem  = 1'b0;
            read_dmem   = 1'b0;
            mem_to_reg  = 1'b0;
            is_branch   = 1'b0;
        end
    default: begin
            reg_dst     = 1'bx;
            write_regf  = 1'bx;
            alu_src     = 1'bx;
            alu_op      = 3'bx;
            write_dmem  = 1'bx;
            read_dmem   = 1'bx;
            mem_to_reg  = 1'bx;
            is_branch   = 1'bx;
        end
    endcase 
end
assign pc_src = (Zero && Branch_final) ? 1'b1 : 1'b0;

assign prg_addr = pc;

//IF_ID pipeline
  always@(posedge clk) begin
       pc_out = pc_incr4;
       instruction_out = prg_data;
    end
//IDIF_EX pipeline
always @(posedge clk)
    begin
      PCplus4out <= pc_out;
      ReadData1_out <= regf_rdata1;
      ReadData2_out <= regf_rdata2;
      SignExtendResult_out <= imm_signed;
      rtOut <= rt;
      rdOut <= rd;
      RegWriteOut <= write_regf;
      MemtoRegOut <= mem_to_reg;
      MemWriteOut <= write_dmem;
      MemReadOut <= read_dmem;
      ALUSrcOut <= alu_src;
      ALUOpOut <= alu_op;
      RegDstOut <= reg_dst;
      BranchOut <= is_branch;
      wdata_dmem <= regf_rdata2;
end
//EX_MEM pipeline

always @(posedge clk) begin
      RegWriteOut_MEM <= RegWriteOut;
      MemtoRegOut_MEM <= MemtoRegOut;
      MemWriteOut_MEM <= MemWriteOut;
      MemReadOut_MEM <= MemReadOut;
      ALUresultOut <= alu_result;
      writedataOut <= wdata_dmem;
      Branch_final <= BranchOut;
      waddr_regf_out <= waddr_regf;
end
//MEM_WB pipeline

always@(posedge clk)
    begin
      RegWriteOut_Final <= RegWriteOut_MEM;
//      MemReadOut_Final<= MemReadOut_MEM;
      readDataOut <= dmem_rdata;
      ALUresultOut_Final <= ALUresultOut;
      writeRegOut_Final <= waddr_regf_out;
      MemtoRegOut_Final <= MemtoRegOut_MEM;
end
endmodule
