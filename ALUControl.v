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

module ALUControl(type, ALU_op, opcode);

    input[1:0] ALU_op;
    input[1:0] opcode;
    output reg[3:0] type;
    
    always @(*)
    begin
        type = 4'b1;
        if(ALU_op == 2'b00)
            type = 4'b0010;                 // D-type
        else if(ALU_op == 2'b01)
            type = 4'b0010;                 // B-type
        else if(ALU_op == 2'b10)            // R-type
        begin
            if (opcode == `ADD_op)      		
			    type =  4'b0010;
            else if (opcode == `SUB_op)      		
                type =  4'b0110;
            else if (opcode == `AND_op)      
                type =  4'b0000;
            else if (opcode == `ORR_op)     
                type =  4'b0001;    
            else if (opcode == `LSL_op)      
                type =  4'b0011;
            else if (opcode == `LSR_op)      
                type =  4'b0111;
            else if (opcode == `B_op)      
                type = 4'b1111;	
        end
    end

endmodule