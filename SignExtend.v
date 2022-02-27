// D-Type
`define LDUR_op 11'b11111000010
`define STUR_op 11'b11111000000

// B-Type
`define CBZ_op  11'b10110100???

// R-Type
`define LSL_op  11'b11010011011
`define LSR_op  11'b11010011010
`define B_op    11'b000101?????

module SignExtend(instruction, extended);

input [31:0] instruction;
output reg [63:0] extended;

always @ (*)
	begin
		if (instruction[31:26] == `B_op)				
			extended = {{38{instruction[25]}}, instruction[25:0]};

		else if(instruction[31:24] == `CBZ_op)
			extended = {{45{instruction[23]}}, instruction[23:5]};

		else if(instruction[31:21] == `LDUR_op || instruction[31:21] == `STUR_op)
			extended = {{55{instruction[20]}}, instruction[20:12]};

		else if(instruction[31:21] == `LSL_op || `LSR_op)
			extended = {{58{1'b0}}, instruction[15:10]};

		else
			extended = {{32{instruction[31]}},instruction};
	end
endmodule