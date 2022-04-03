`timescale 1ns / 1ps

/*
	Group Members: Ralph Quinto and Warren Seto

	Lab Name: ARM LEGv8 CPU Testbench (Pipelined, Hazard Detection, and Forwarding Unit) 
*/

module CPU_TEST;
  
  /* CPU Signals */
  reg reset;
  reg clk;
  
  /* Connect CPU to Instruction Memory */
  wire [63:0] PC_wire;
  wire [31:0] IC_wire;
  
  /* Connect CPU to Data Memory */
  wire [63:0] mem_address;
  wire [63:0] mem_data_in;
  wire c_memWrite;
  wire c_memRead;
  wire [63:0] mem_data_o;
  
  ARM_CPU core (reset, clk, IC_wire, mem_data_o, PC_wire, mem_address, mem_data_in, c_memWrite, c_memRead);
  IC mem1 (PC_wire, IC_wire);
  Data_Memory mem2 (mem_address, mem_data_in, c_memWrite, c_memRead, mem_data_o);
  
  /* Setup the clock */
  initial begin
    clk = 1'b0;
    reset = 1'b1;
    #30 $finish;
  end
  
  /* Toggle the clock */
  always begin
    #1 clk = ~clk; reset = 1'b0;
  end
  
endmodule