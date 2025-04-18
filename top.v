`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/16/2025 02:28:34 PM
// Design Name: 
// Module Name: final
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////



module update_pc(zero,jump_target,PC_plus_four,signExtendedImmediate,readDATA1,clk,rst,jump,jump_src,nextPC);
input zero;
input [25:0] jump_target;
input [31:0] PC_plus_four;
input [31:0] signExtendedImmediate;
input [31:0] readDATA1;
input clk,rst,jump,jump_src;        
output reg [31:0] nextPC;
    always@(posedge clk or rst) begin
        if(rst) begin
            nextPC<=32'd0;
        end
        else begin
            if(jump) begin
                if(jump_src) begin
                    nextPC<=readDATA1;
                end  
                else begin
                    nextPC<={PC_plus_four[31:28],2'b00,jump_target[25:0]};
                end  
            end
            else begin
                if(zero) begin   // if branch split
                    nextPC<=$signed(PC_plus_four)+$signed(signExtendedImmediate);
                end 
                else begin
                    nextPC<=PC_plus_four;
                end     
            end
        end        
    end
endmodule
module MUXDataToRegester(memToReg,regDst,readDATA,PC_plus_four,ALU_result,writedata);
    input memToReg;
    input [1:0]regDst;
    input [31:0]readDATA;
    input [31:0]PC_plus_four;
    input [31:0]ALU_result;
    output reg [31:0]writedata;
    always@(*) begin
        if(regDst==2'b10) begin
            writedata<=PC_plus_four;
        end
        else begin
            if(memToReg) begin
                writedata<=readDATA;
            end 
            else begin
                writedata<=ALU_result;
            end       

        end    
    end
endmodule
module MY_ALU(
    input [31:0] ALU_input1, ALU_input2,
    input [3:0] branch,
    input [4:0] shamt,
    input [3:0] ALU_control,
    output reg [31:0] ALU_result,
    output reg [31:0] HI, LO,
    output reg zero, 
    output carry_out, overflow
);      
    localparam [3:0] Not         = 4'd0;
    localparam [3:0] ADD         = 4'd1;
    localparam [3:0] SUB         = 4'd2;
    localparam [3:0] ADDU        = 4'd3;
    localparam [3:0] SUBU        = 4'd4;
    localparam [3:0] MUL         = 4'd5;
    localparam [3:0] MADD        = 4'd6;
    localparam [3:0] MADDU       = 4'd7;
    localparam [3:0] AND         = 4'd8;
    localparam [3:0] OR          = 4'd9;
    localparam [3:0] XOR         = 4'd10;
    localparam [3:0] SLL         = 4'd11;  // Shift left logical
    localparam [3:0] SRL         = 4'd12;  // Shift right logical
    localparam [3:0] SRA         = 4'd13;
    localparam [3:0] Slt         = 4'd14;
    localparam [3:0] Seq         = 4'd15;

    localparam [3:0] Branch_equal    = 4'd1;
    localparam [3:0] Branch_notEqual = 4'd2;
    localparam [3:0] Branch_bgt      = 4'd3;
    localparam [3:0] Branch_bgte     = 4'd4;
    localparam [3:0] Branch_ble      = 4'd5;
    localparam [3:0] Branch_bleq     = 4'd6;
    localparam [3:0] Branch_bleu     = 4'd7;
    localparam [3:0] Branch_bgtu     = 4'd8;


    reg [63:0] temp;
   always@(*)begin
    case(branch)
        Branch_equal: begin
        zero = ($signed(ALU_input1) == $signed(ALU_input2)); // beq
        end
        Branch_notEqual: begin
            zero = ($signed(ALU_input1) != $signed(ALU_input2)); // bne
        end
        Branch_bgt: begin
            zero = ($signed(ALU_input1) > $signed(ALU_input2));  // bgt (signed)
        end
        Branch_bgte: begin
            zero = ($signed(ALU_input1) >= $signed(ALU_input2)); // bge (signed)
        end
        Branch_ble: begin
            zero = ($signed(ALU_input1) < $signed(ALU_input2));  // blt (signed)
        end
        Branch_bleq: begin
            zero = ($signed(ALU_input1) <= $signed(ALU_input2)); // ble (signed)
        end
        Branch_bleu: begin
            zero = ($unsigned(ALU_input1) <= $unsigned(ALU_input2)); // bleu (unsigned)
        end
        Branch_bgtu: begin
            zero = ($unsigned(ALU_input1) > $unsigned(ALU_input2));  // bgtu (unsigned)
        end
        default: begin
            zero = 0;              
            case(ALU_control)
                ADD: begin
                    ALU_result=$signed(ALU_input1) + $signed(ALU_input2);
                end 
                SUB: begin
                    ALU_result=$signed(ALU_input1) -$signed(ALU_input2);
                end 
                ADDU: begin
                    ALU_result=$unsigned(ALU_input1) + $unsigned(ALU_input2);
                end 
                SUBU: begin
                    ALU_result=$unsigned(ALU_input1) - $unsigned(ALU_input2);
                end 
                //
                MUL: begin
                    temp = $unsigned(ALU_input1) * $unsigned(ALU_input2);
                    HI = temp[63:32];
                    LO = temp[31:0];
                end 
                MADD: begin
                    temp = $signed(ALU_input1) * $signed(ALU_input2);
                    temp = temp + {HI, LO};
                    HI = temp[63:32];
                    LO = temp[31:0];
                end 
                MADDU: begin
                    temp = $unsigned(ALU_input1) * $unsigned(ALU_input2);
                    temp = temp + {HI, LO};
                    HI = temp[63:32];
                    LO = temp[31:0];
                end 
                //
                AND: begin
                    ALU_result=ALU_input1 & ALU_input2;
                end 
                OR: begin
                    ALU_result=ALU_input1 | ALU_input2;
                end 
                XOR: begin
                    ALU_result=ALU_input1 ^ ALU_input2;
                end 
                Slt: begin
                    ALU_result=($signed(ALU_input1)<$signed(ALU_input2));
                end
                Seq: begin
                    ALU_result=($signed(ALU_input1)==$signed(ALU_input2));
                end
                SLL: begin // Shift Left Logical
                    ALU_result = ALU_input2 << shamt;
                end
                SRL: begin
                    // Shift Right Logical
                    ALU_result = ALU_input2 >> shamt;
                end
                SRA: begin
                    // Shift Right Arithmetic (preserves the sign bit for signed values)
                    ALU_result = $signed(ALU_input2) >>> shamt;
                end
                Not: begin
                    // Bitwise NOT
                    ALU_result = ~ALU_input1;
                end    
                default : begin
                    ALU_result=32'd0;
                end    
            endcase
        end    
    endcase
   end 

endmodule
// copied from yug 
module my_ALU_CTRL(
    input [3:0] aluOp,
    input [5:0] funct,
    output jump_src,
    output reg [3:0] aluControl
);  
    localparam [3:0] ALU_OP_RTYPE=4'd0;

    localparam [3:0] NOT =4'd0;
    localparam [3:0] ADD  = 4'd1;
    localparam [3:0] SUB  = 4'd2;
    localparam [3:0] ADDU  = 4'd3;
    localparam [3:0] SUBU  = 4'd4;
    localparam [3:0] MUL  = 4'd5;
    localparam [3:0] MADD   = 4'd6;
    localparam [3:0] MADDU  = 4'd7;
    localparam [3:0] AND   = 4'd8;
    localparam [3:0] OR  = 4'd9;
    localparam [3:0] XOR  = 4'd10;
    localparam [3:0] SLL  = 4'd11;  // Shift left logical
    localparam [3:0] SRL  = 4'd12;  // Shift right logical
    localparam [3:0] SRA  = 4'd13;
    localparam [3:0] SLT  = 4'd14;
    localparam [3:0] SEQ  = 4'd15;

    localparam [5:0] fun_ADD  = 6'd32;  // 100000
    localparam [5:0] fun_SUB  = 6'd34;  // 100010
    localparam [5:0] fun_ADDU = 6'd33;  // 100001
    localparam [5:0] fun_SUBU = 6'd35;  // 100011
    localparam [5:0] fun_MUL  = 6'd24;  // 011000
    localparam [5:0] fun_AND  = 6'd36;  // 100100
    localparam [5:0] fun_OR   = 6'd37;  // 100101
    localparam [5:0] fun_XOR  = 6'd38;  // 100110
    localparam [5:0] fun_SLL  = 6'd0;   // 000000
    localparam [5:0] fun_SRL  = 6'd2;   // 000010
    localparam [5:0] fun_SRA  = 6'd3;   // 000011
    localparam [5:0] fun_SLT  = 6'd42;  // 100000 
    localparam [5:0] fun_JR=6'd8;

    localparam [5:0] fun_SEQ=6'd48;
    localparam [5:0] fun_MADD=6'd49;
    localparam [5:0] fun_MADDU=6'd50;
    localparam [5:0] fun_NOT=6'd49;
    assign jump_src=    (funct==fun_JR);
    always@(*)begin
            case(aluOp)
            ALU_OP_RTYPE: begin
                case(funct)
                    fun_ADD: begin
                        aluControl<=ADD;
                    end
                    fun_SUB: begin
                        aluControl<=SUB;
                    end
                    fun_ADDU: begin
                        aluControl<=ADDU;
                    end
                    fun_SUBU: begin
                        aluControl<=SUBU;
                    end
                    fun_MUL: begin
                        aluControl<=MUL;
                    end
                    fun_AND: begin
                        aluControl<=AND;
                    end
                    fun_OR: begin
                        aluControl<=OR;
                    end
                    fun_XOR: begin
                        aluControl<=XOR;
                    end
                    fun_SLL: begin
                        aluControl<=SLL;
                    end
                    fun_SRL: begin
                        aluControl<=SRL;
                    end
                    fun_SRA: begin
                        aluControl<=SRA;
                    end
                    fun_SLT: begin
                        aluControl<=SLT;
                    end
                    fun_SEQ: begin
                        aluControl<=SEQ;
                    end
                    fun_NOT: begin
                        aluControl<=NOT;
                    end
                    fun_MADD: begin
                        aluControl<=MADD;
                    end    
                    fun_MADDU: begin
                        aluControl<=MADDU;
                    end
                    default:
                        aluControl<=ADD;      
                            
                endcase
            end
            default: aluControl <= aluOp;    
        endcase
    end    
endmodule
module ALU_Input_Mux(
    input [31:0] read1,read2, immediate,
    input alu_src,
    output reg [31:0] alu_input1,alu_input2
);
always @(*) begin
        alu_input1 = read1;
        if (alu_src) begin
            alu_input2 = immediate;
        end 
        else begin
            alu_input2 = read2;
        end
    end
endmodule
module registerFile (rst,readAddress1, readAddress2, writeAddress, writeData,
WE, dataOut1, dataOut2, clock,testdata1,testdata2,testdata3,testdata4);
    input rst;
    input [4:0] readAddress1, readAddress2, writeAddress;
    input [31:0] writeData;
    input WE, clock;
    output [31:0] dataOut1, dataOut2;
    output [31:0] testdata1;
    output [31:0] testdata2;
    output [31:0] testdata3;
    output [31:0] testdata4;
    reg [31:0] RF [31:0];
    assign dataOut1= RF[readAddress1];
    assign dataOut2= RF[readAddress2];
    assign testdata1=RF[9];
    assign testdata2=RF[10];
    assign testdata3=RF[writeAddress];
    assign testdata3=RF[12];
    integer i;
    always @(negedge clock or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1)
                RF[i] <= 32'b0;
        end
        else if (WE && writeAddress != 0) begin
            RF[writeAddress] = writeData;
        end
    end

endmodule
module MUXWriteAddress(rt,rd,regDst,write_address);
    input [4:0]rt,rd;
    input [1:0]regDst;
    output reg [4:0]write_address;
    always@(*) begin
        if(regDst==2'd0)begin
            write_address<=rt;
        end
        else if(regDst==2'd1) begin
            write_address<=rd;
        end
        else if(regDst==2'd2) begin
            write_address<=5'd31;
        end
        else begin
            write_address<=5'd0;
        end                
    end    
endmodule
module myControlUnit(opcode,regDst,regWrite,aluSrc,memRead,memWrite,memToReg,jump,branch,aluOp);
    input [5:0] opcode;
    output reg [1:0] regDst;
    output reg regWrite,aluSrc,memRead,memWrite,memToReg,jump;
    output reg [3:0] branch;
    output reg [3:0] aluOp;
    // Define local parameters for ALU operation codes
        localparam [3:0] ALU_OP_RTYPE=4'd0;
        localparam [3:0] ADD  = 4'd1;
        localparam [3:0] SUB  = 4'd2;
        localparam [3:0] ADDU  = 4'd3;
        localparam [3:0] SUBU  = 4'd4;
        localparam [3:0] MUL  = 4'd5;
        localparam [3:0] MADD   = 4'd6;
        localparam [3:0] MADDU  = 4'd7;
        localparam [3:0] AND   = 4'd8;
        localparam [3:0] OR  = 4'd9;
        localparam [3:0] XOR  = 4'd10;
        localparam [3:0] SLL  = 4'd11;  // Shift left logical
        localparam [3:0] SRL  = 4'd12;  // Shift right logical
        localparam [3:0] SRA  = 4'd13;
        localparam [3:0] Slt  = 4'd14;
        localparam [3:0] Seq  = 4'd15;    
         // Shift right arithmetic
        // localparam [3:0] SLT  = 4'd14;// Shift left logical
        // localparam [3:0] SRL  = 4'd15;  // Shift right logical
        // localparam [3:0] SRA  = 4'd16;  // Shift right arithmetic
        // localparam [3:0] SLT  = 4'd17; 
        // localparam [3:0] OR   = 4'd18;
        // localparam [3:0] XOR  = 4'd19;
        // localparam [3:0] SLL  = 4'd8;  // Shift left logical
        // localparam [3:0] SRL  = 4'd9;  // Shift right logical
        // localparam [3:0] SRA  = 4'd10;  // Shift right arithmetic
        // localparam [3:0] SLT  = 4'd11;  // Set on less than
        localparam [5:0] opCode_R_Type   = 6'd0;
        localparam [5:0] opCode_addIU_Type = 6'd9;
        localparam [5:0] opCode_ANDI_Type  = 6'd12;
        localparam [5:0] opCode_ORI_Type   = 6'd13;
        localparam [5:0] opCode_XORI_Type  = 6'd14;
        localparam [5:0] opCode_addI_Type  = 6'd8;
        localparam [5:0] opCode_load_Type  = 6'd35;
        localparam [5:0] opCode_SW_Type    = 6'd43;
        localparam [5:0] opCode_LUI_Type   = 6'd15;
        localparam [5:0] opCode_J_Type     = 6'd2;
        localparam [5:0] opCode_JAL_Type   = 6'd3;
        localparam [5:0] opCode_SLTI_Type  = 6'd10;
        localparam [5:0] opCode_beq_Type   = 6'd4;
        localparam [5:0] opCode_bne_Type   = 6'd5;
        

        // added instuction type
        localparam [5:0] opCode_bgt_Type=6'd32;  //32 bgte
        localparam [5:0] opCode_bgte_Type=6'd33;  //33 bgte
        localparam [5:0] opCode_ble_Type=6'd34;  //34 bgte
        localparam [5:0] opCode_bleq_Type=6'd36;  //35 bgte
        localparam [5:0] opCode_bleu_Type=6'd37;  //36 bleu
        localparam [5:0] opCode_bgtu_Type=6'd38;  //37 bgtu
        //localparam [5:0] opCode_bgtu_Type=6'b100000; //38 bgte
        localparam [5:0] opCode_SEQ_Type=6'b100110;  //38seq

        localparam [3:0] Branch_equal=4'd1;
        localparam [3:0] Branch_notEqual=4'd2;
        localparam [3:0] Branch_bgt=4'd3;
        localparam [3:0] Branch_bgte=4'd4;
        localparam [3:0] Branch_ble=4'd5;
        localparam [3:0] Branch_bleq=4'd6;
        localparam [3:0] Branch_bleu=4'd7;
        localparam [3:0] Branch_bgtu=4'd8;
    always @(*) begin
        

        regDst = 2'b00;
        regWrite = 0;
        aluSrc = 0;
        memRead = 0;
        memWrite = 0;
        memToReg = 0;
        jump = 0;
        branch = 4'b0000;
        aluOp = 4'b0000;
        // opcode decoding
        //6'b000000 R-type(add,addu,sub,subu,madd,maddu,mul,not and, or, xor, slt,seq,jr, sltu,sll,srl,sra,sla)
        case (opcode)
            opCode_R_Type: begin // R-type
                regDst = 2'b01;
                regWrite = 1;
                aluSrc = 0;
                memToReg = 0;
                aluOp = ALU_OP_RTYPE; // ALU operation defined by funct field
            end

            opCode_addI_Type: begin // addi
                regDst = 2'b00;
                regWrite = 1;
                aluSrc = 1;
                memToReg = 0;
                aluOp = ADD; // ALU operation for addi
            end

            opCode_addIU_Type: begin // addiu
                regDst = 2'b00;
                regWrite = 1;
                aluSrc = 1;
                memToReg = 0;
                aluOp = ADDU; // ALU operation for addi
            end
            opCode_ANDI_Type: begin // andi
                regDst = 2'b00;
                regWrite = 1;
                aluSrc = 1;
                memToReg = 0;
                aluOp = AND; // ALU operation for addi
            end
            opCode_ORI_Type: begin // ORI_TYPE
                regDst = 2'b00;
                regWrite = 1;
                aluSrc = 1;
                memToReg = 0;
                aluOp = OR; 
            end    
            opCode_XORI_Type: begin  //XORI
                regDst = 2'b00;
                regWrite = 1;
                aluSrc = 1;
                memToReg = 0;
                aluOp = XOR; 
            end
            opCode_load_Type: begin  //LW
                regDst = 2'b00;
                regWrite = 1;
                aluSrc = 1;
                memRead=1;
                memWrite=0;
                memToReg = 1;
                jump = 0;
                branch = 4'b0000;
                aluOp = ADD;    
            end
            opCode_SW_Type: begin  //SW
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 1;
                memRead=0;
                memWrite=1;
                memToReg = 1;
                jump = 0;
                branch = 4'b0000;
                aluOp = ADD;    
            end
            /// think again
            opCode_LUI_Type: begin  //LUI
                regDst = 2'b00;
                regWrite = 1;
                aluSrc = 1;
                memRead=0;
                memWrite=1;
                memToReg = 1;
                jump = 0;
                branch = 4'b0000;
                aluOp = ADD;    
            end
            opCode_beq_Type: begin  //Beq
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4'd1;
                aluOp = 4'd0;    
            end
            opCode_bne_Type: begin  //Bne
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4'd2;
                aluOp = 4'd0;    
            end
            opCode_bgt_Type: begin  //Bgt
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4;
                aluOp = 4'd3;    
            end
            opCode_bgte_Type: begin  //Bgte
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4'd4;
                aluOp = 4'd0;    
            end
            opCode_ble_Type: begin  //Ble
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4'd5;
                aluOp = 4'd0;    
            end
            opCode_bleq_Type: begin  //Bleq
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4'd6;
                aluOp = 4'd0;    
            end
            opCode_bleu_Type: begin  //Bleu
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4'd7;
                aluOp = 4'd0;    
            end
            opCode_bgt_Type: begin  //Bgtu
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4'd8;
                aluOp = 4'd0;    
            end
            opCode_bgtu_Type: begin  //Bgtu
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4'd8;
                aluOp = 4'd0;    
            end
            opCode_J_Type: begin  //Jump
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 1;
                branch = 4'd0;
                aluOp = 4'd0;    
            end
            opCode_JAL_Type: begin  //JAL
                regDst = 2'b10;
                regWrite = 1;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 1;
                branch = 4'd0;
                aluOp = 4'd0;    
            end
            opCode_SLTI_Type: begin  //SLTI
                regDst = 2'b0;
                regWrite = 1;
                aluSrc = 1;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4'd0;
                aluOp = Slt;    
            end
            opCode_SEQ_Type: begin  //SEQ
                regDst = 2'b00;
                regWrite = 1;
                aluSrc = 1;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4'd0;
                aluOp = Seq;    
            end
            default: begin
                regDst = 2'b00;
                regWrite = 0;
                aluSrc = 0;
                memRead=0;
                memWrite=0;
                memToReg = 0;
                jump = 0;
                branch = 4'd0;
                aluOp = 4'd0;
            end    
        endcase
    end
endmodule

 
module SignExtender(
    input [15:0] immediate,
    output  [31:0] sign_extended_immediate
);  
    assign sign_extended_immediate = {  {16{immediate[15]}} , immediate};
endmodule

// copied from yug change it 
module Decoder(
    input [31:0] instruction,
    output reg [5:0] opcode,funct,
    output reg [4:0] rs,rd,rt,shamt,
    output reg [15:0] immediate,
    output reg [25:0] jump_addr
);
    
    always @(*) begin
        opcode = instruction[31:26];
        funct = instruction[5:0];
        rs = instruction[25:21];
        rt = instruction[20:16];
        rd = instruction[15:11];
        shamt = instruction[10:6];
        immediate = instruction[15:0];
        jump_addr = instruction[25:0];
    end
endmodule



module main(
    clk ,rst,enable,
    inst_write_address,
    inst_write_data,
    mem_write_input_address,
    mem_write_input_data
    
);
    // inst_write_address
    input clk, rst,enable;
    input [9:0]inst_write_address;
    input [31:0]inst_write_data;
    // Program Counter
    input [9:0]mem_write_input_address;
    input [31:0]mem_write_input_data;
    
    wire mem_write_signal;
    wire [9:0]mem_write_address;
    wire [31:0]mem_write_data;
   
    
    
    
    wire [31:0] PC ; 
    wire [31:0] PC_plus_four;
    assign PC_plus_four = PC + 1;
    // Instruction
    wire [31:0] instruction;

    // Decoded Fields
    wire [5:0] opcode , funct;
    wire [4:0] rs , rt , rd , shamt;    
    wire [15:0] immediate;
    wire [31:0] sign_extended_immediate;
    wire [25:0] jump_address;

    // Data from register file
    wire [31:0] read_data1, read_data2, write_data;

    // ALU inputs and outputs
    wire [31:0] ALU_input1 , ALU_input2;
    wire [31:0] ALU_result;
    wire [3:0] ALU_op;
    wire [3:0] ALU_control;
    wire equal, less_than, greater_than;
    wire carry_out;
    //wire jump_src;
    wire overflow;

    // Data memory output
    wire [31:0] memory_data;

    // Multiplication registers
    wire [31:0] HI , LO;

    // Bramch and Jump
    wire [31:0] branch_target;
    wire [31:0] jump_target;

    wire[4:0] write_address;
    // mux control signals
    wire jump, reg_write, mem_read, mem_write, mem_to_reg, alu_src,  branch , jump_src;
    wire [1:0] reg_dst;
    // which branch
    wire [3:0] branches;
    wire [31:0] testdata1;
    wire [31:0] testdata2;
    wire [31:0] testdata3;
    wire [31:0] testdata4;
    
    assign mem_write_data = (enable) ? mem_write_input_data: read_data2;
    assign mem_write_address = (enable) ? mem_write_input_address: ALU_result[9:0];
    assign mem_write_signal = (enable) ? enable: mem_write;
    
     // fetch instruction 
    memory_wrapper inst_mem(
        .a(inst_write_address),
        .d(inst_write_data),
        .dpra(PC[9:0]),
        .clk(clk),
        .we(enable),
        .dpo(instruction)
    );
    
    Decoder decoder (
        .instruction(instruction),
        .opcode(opcode),
        .funct(funct),
        .rs(rs),
        .rt(rt),
        .rd(rd),
        .shamt(shamt),
        .immediate(immediate),
        .jump_addr(jump_address)
    );
    // Sign extender
    SignExtender sign_extender (
        .immediate(immediate),
        .sign_extended_immediate(sign_extended_immediate)
    );
    // data memory
    memory_wrapper data_mem(
        .a(mem_write_address),
        .d(mem_write_data),
        .dpra(ALU_result[9:0]),
        .clk(clk),
        .we(mem_write_signal),
        .dpo(memory_data)
    );
    // CU
    myControlUnit cu(
        .opcode(opcode),
        .regDst(reg_dst),
        .regWrite(reg_write),
        .aluSrc(alu_src),
        .memRead(mem_read),
        .memWrite(mem_write),
        .memToReg(mem_to_reg),
        .jump(jump),
        .branch(branches),
        .aluOp(ALU_op)
    );
    // Mux for writing address
    MUXWriteAddress inst_WriteAddress(
        .rt(rt),
        .rd(rd),
        .regDst(reg_dst),
        .write_address(write_address)
    );
    // regester file
    registerFile inst_RF (
        .rst(rst),
        .readAddress1(rs),
        .readAddress2(rt),
        .writeAddress(write_address),
        .writeData(write_data),               // fill after some time 
        .WE(reg_write),
        .clock(clk),
        .dataOut1(read_data1),
        .dataOut2(read_data2),
        .testdata1(testdata1),
        .testdata2(testdata2),
        .testdata3(testdata3),
        .testdata4(testdata4)
    );    
    /// selection of data for r type and i type inst
    ALU_Input_Mux alu_input_mux (
        .read1(read_data1),
        .read2(read_data2),
        .immediate(sign_extended_immediate),
        .alu_src(alu_src),
        .alu_input1(ALU_input1),
        .alu_input2(ALU_input2)
    ); 

    // ALU_ctrl 
    my_ALU_CTRL inst_ALU_CTRL(
        .aluOp(ALU_op),
        .funct(funct),
        .jump_src(jump_src),
        .aluControl(ALU_control)
    );

    // ALU 
    MY_ALU inst_MY_ALU(
        .ALU_input1(read_data1),
        .ALU_input2(ALU_input2),
        .branch(branches),
        .shamt(shamt),
        .ALU_control(ALU_control),
        .ALU_result(ALU_result),
        .HI(HI),
        .LO(LO),
        .zero(zero),
        .carry_out(carry_out),
        .overflow(overflow)
    );
    // MUX for selecting data for regester writedata
    MUXDataToRegester inst_MUXDataToRegester(
        .memToReg(mem_to_reg),
        .regDst(reg_dst),
        .readDATA(memory_data),
        .PC_plus_four(PC_plus_four),
        .ALU_result(ALU_result),
        .writedata(write_data)
    );
    update_pc inst_update_pc(
        .zero(zero),
        .jump_target(instruction[25:0]),
        .PC_plus_four(PC_plus_four),
        .signExtendedImmediate(sign_extended_immediate),
        .readDATA1(read_data1),
        .clk(clk),
        .rst(rst),
        .jump(jump),
        .jump_src(jump_src),
        .nextPC(PC)
    ) ;
    initial begin
        // Print selected signals when they change
        $monitor("Time = %0t|clk=%b | PC = %d|inst=%d |nextpc = %d|d1=%d|d2=%d|al1=%d|AL2=%d |rs=%d|rt=%d|rd=%d |wd=%d|regdata=%b|memwrite=%b|jump=%b|jumpSRC=%b", $time,clk,PC,+ instruction,$signed(PC_plus_four)+ $signed(sign_extended_immediate),testdata1,testdata2,$signed(read_data1),ALU_input2,rs,rt,rd,write_data,sign_extended_immediate,zero,jump,jump_src);
//             );
    end  
endmodule



// instruction format for bgte {6'd33,rs,rt,offset}
// instruction format for ble {6'd34,rs,rt,offset} ble
// instruction for  bleq  {6'd36,rs,rt,offset}