`timescale 1ns / 1ps

module memory( //BITS
    input clk,
    input reset,
    input [63:0] addressRead,
    input fetch_new_ins,
    output reg [23:0] reference
    );
    
    reg [23:0] memory [59999:0];//57962
    reg [23:0] input_t;
    integer i;
    integer File;
    initial 
    begin
        i = 0;
        //Initialize memory File 
        File = $fopen("TRACE1.DAT", "rb");
        while (!$feof(File))
        begin
            input_t = ($fgetc(File)) | ($fgetc(File) << 8) | ($fgetc(File) << 16);
            memory[i] = input_t;
            i=i+1;
        end
        $fclose(File);
    end

    always @(*)
    begin
        if (reset)
        begin
            
        end
        else
        begin
            reference <= memory[addressRead];
        end
    end
    
    

endmodule
