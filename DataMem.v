module DataMemory(data_in , address , data_write , mem_read , mem_write , clk);
   input [63:0] data_write;
   input [63:0] address;

   input mem_read, mem_write;
   input clk;

   output reg [63:0] data_in;
          reg [7:0]  memory[1023:0];   
          // 16 different memory blocks of 64bits --> 1024 or 128bit

    // initialization task: dividing the memory banks
   task init;                           
      input [63:0] addr;
      input [63:0] data;

        begin
            memory[addr]   =  data[63:56]; 
            memory[addr+1] =  data[55:48];
            memory[addr+2] =  data[47:40];
            memory[addr+3] =  data[39:32];
            memory[addr+4] =  data[31:24];
            memory[addr+5] =  data[23:16];
            memory[addr+6] =  data[15:8];
            memory[addr+7] =  data[7:0];
        end
    endtask

initial
     begin
    	// Set data used in instruction memory
    	init( 64'h0,  64'h1);  			// Counter variable
    	init( 64'h8,  64'ha);  			// part of the mask
    	init( 64'h10, 64'h5);  			// other part of mask
    	init( 64'h18, 64'h0ffbea7deadbeeff); 	// constant
    	init( 64'h20, 64'h0); 			// delete
     end

    // Read and retrieve data
   always @(posedge clk)
     begin
    	if(mem_read)
    	    begin
                data_in[63:56]    = memory[address];
                data_in[55:48]    = memory[address+1];
                data_in[47:40]    = memory[address+2];
                data_in[39:32]    = memory[address+3];
                data_in[31:24]    = memory[address+4];
                data_in[23:16]    = memory[address+5];
                data_in[15:8]     = memory[address+6];
                data_in[7:0]      = memory[address+7];
    	    end
    end

   // Write and save data
   always @ (posedge clk)
     begin
    	if(mem_write)
    	    begin
                memory[address]   = data_write[63:56];
                memory[address+1] = data_write[55:48];
                memory[address+2] = data_write[47:40];
                memory[address+3] = data_write[39:32];
                memory[address+4] = data_write[31:24];
                memory[address+5] = data_write[23:16];
                memory[address+6] = data_write[15:8];
                memory[address+7] = data_write[7:0];
    	    end
    end
endmodule