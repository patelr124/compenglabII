
module Registers(r1, r2, r_write, data_in, write_E, r1_data, r2_data, clk, reset);
	input clk;						
	input reset;
	input [4:0] r1, r2;				
	input [4:0] r_write;				// register to write to
	input [63:0] data_in;				// data to write
	input write_E;						// write enable control line
	
	output [63:0] r1_data, r2_data;		
	reg [63:0] registers [31:0]; 

	integer i;

	assign #2 r1_data = registers[r1];
	assign #2 r2_data = registers[r2];

	initial
	begin
		registers[31] = 64'b0; 	// 64 bit binary literals (64'b0)
	end
    
	// if write_E is HIGH and r_write is 5 bits then may write the data to r_write register	

	always @ (posedge clk)
	begin
		if(reset)
		for(i = 0; i < 64; i=i+1)
			registers[i] = 64'b0;
		else
		begin
			if(write_E && r_write != 5'b11111)
			registers[r_write] = data_in;
		end
	end
	
	always @ (*)
	begin
		registers[31] = {64{1'b0}}; // set register 31 back to 0
	end
endmodule