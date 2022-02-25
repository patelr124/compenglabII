// D-Type
`define LDUR_op 11'b11111000010
`define STUR_op 11'b11111000000

// B-Type
`define CBZ_op  11'b10110100???

// R-Type
`define ADD_op  11'b10001011000
`define SUB_op  11'b11001011000
`define AND_op  11'b10001010000
`define ORR_op  11'b10101010000
`define LSL_op  11'b11010011011
`define LSR_op  11'b11010011010
`define B_op    11'b000101?????

module Control(op_code, Reg2Loc, Branch, UnconBranch, MemRead, Mem2Reg, ALU_op, MemWrite, ALUSrc, RegWrite);
input [10:0] op_code;                        //11bit OPCode zijn de 11 MSB van de instruction Code

output [1:0] ALU_op;                         //2bit ALUOP dit zijn de 2 MSB van de 11 bit OPCode
reg    [1:0] ALU_op;

output Reg2Loc;                           
output Branch;
output MemRead;
output Mem2Reg;
output MemWrite;
output ALUSrc;
output RegWrite;
output UnconBranch;

reg Reg2Loc, Branch, UnconBranch, MemRead, Mem2Reg, MemWrite, ALUSrc, RegWrite;

always @ (*)
begin
    case(op_code)
        `LDUR_op:                        // ALUOP[00] D-type (Load & Store)
            begin
                Reg2Loc     =  1'b0;    
                ALUSrc      =  1'b1;
                Mem2Reg     =  1'b1;
                RegWrite    =  1'b1;          
                MemRead     =  1'b1;
                MemWrite    =  1'b0;
                Branch      =  1'b0;
                UnconBranch =  1'b0;
                ALU_op       =  2'b00;                       
            end
	    `STUR_op: 
            begin
                Reg2Loc     =  1'b1;    
                ALUSrc      =  1'b1;
                Mem2Reg     =  1'b0;
                RegWrite    =  1'b0;          
                MemRead     =  1'b0;
                MemWrite    =  1'b1;
                Branch      =  1'b0;
                UnconBranch =  1'b0;
                ALU_op       =  2'b00;       
            end
        `CBZ_op:                         // ALUOP[01] B-type (CBZ)
            begin
                Reg2Loc     =  1'b1;    
                ALUSrc      =  1'b0;
                Mem2Reg     =  1'b0;
                RegWrite    =  1'b1;          
                MemRead     =  1'b0;
                MemWrite    =  1'b0;
                Branch      =  1'b1;
                UnconBranch =  1'b0;
                ALU_op       =  2'b01;
            end
	    `ADD_op:                         // ALUOP[10] R-types (AND & OR & ...)  
            begin
                Reg2Loc     =  1'b0;    
                ALUSrc      =  1'b0;
                Mem2Reg     =  1'b0;
                RegWrite    =  1'b1;          
                MemRead     =  1'b0;
                MemWrite    =  1'b0;
                Branch      =  1'b0;
                UnconBranch =  1'b0;
                ALU_op       =  2'b10;
            end
	    `SUB_op: 
            begin
                Reg2Loc     =  1'b0;   
                ALUSrc      =  1'b0;
                Mem2Reg     =  1'b0;
                RegWrite    =  1'b1;           
                MemRead     =  1'b0;
                MemWrite    =  1'b0;
                Branch      =  1'b0;
                UnconBranch =  1'b0;
                ALU_op       =  2'b10;
            end
	    `AND_op: 
            begin
                Reg2Loc     =  1'b0;    
                ALUSrc      =  1'b0;
                Mem2Reg     =  1'b0;
                RegWrite    =  1'b1;           
                MemRead     =  1'b0;
                MemWrite    =  1'b0;
                Branch      =  1'b0;
                UnconBranch =  1'b0;
                ALU_op       =  2'b10;
            end
	    `ORR_op: 
            begin
                Reg2Loc     =  1'b0;    
                ALUSrc      =  1'b0;
                Mem2Reg     =  1'b0;
                RegWrite    =  1'b1;           
                MemRead     =  1'b0;
                MemWrite    =  1'b0;
                Branch      =  1'b0;
                UnconBranch =  1'b0;
                ALU_op       =  2'b10;
            end
	    `B_op: 
            begin
                Reg2Loc     =  1'b0;    
                ALUSrc      =  1'b0;
                Mem2Reg     =  1'b0;
                RegWrite    =  1'b1;           
                MemRead     =  1'b0;
                MemWrite    =  1'b0;
                Branch      =  1'b0;
                UnconBranch =  1'b1;
                ALU_op       =  2'b10;
            end 
	    `LSR_op: 
            begin
                Reg2Loc     =  1'b0;    
                ALUSrc      =  1'b0;
                Mem2Reg     =  1'b0;
                RegWrite    =  1'b1;           
                MemRead     =  1'b0;
                MemWrite    =  1'b0;
                Branch      =  1'b0;
                UnconBranch =  1'b0;
                ALU_op       =  2'b10;
            end 
        `LSL_op: 
            begin
                Reg2Loc     =  1'b0;    
                ALUSrc      =  1'b0;
                Mem2Reg     =  1'b0;
                RegWrite    =  1'b1;           
                MemRead     =  1'b0;
                MemWrite    =  1'b0;
                Branch      =  1'b0;
                UnconBranch =  1'b0;
                ALU_op       =  2'b10;
            end   
        default:                                //RESET! alles op X
            begin
                Reg2Loc     =  1'b0;    
                ALUSrc      =  1'b0;
                Mem2Reg     =  1'b0;
                RegWrite    =  1'b0;           
                MemRead     =  1'b0;
                MemWrite    =  1'b0;
                Branch      =  1'b0;
                UnconBranch =  1'b0;
                ALU_op       =  2'b00;
           end
        endcase       
    end
endmodule