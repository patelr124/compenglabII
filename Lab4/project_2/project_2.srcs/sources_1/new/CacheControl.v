`timescale 1ns / 1ps

module CacheControl
    (
    input clk,
    input reset,
    input [23:0] reference,
    input  hit,
    output [19:0] tag,
    output reg [63:0] addressRead,
    output index,
    output reg write_E,
    output reg read_E,
    output reg fetch_new_ins
    );
    
    reg [63:0] hits;
    reg [63:0] misses;
    reg [63:0] references_count;
    
    wire [2:0] offset;
    
    assign tag = reference[23:4]; 
    assign index = reference[3]; 
    assign offset = reference[2:0]; 
    
    always @(posedge clk)
    begin
    //Drive addressRead
    //Output:
        //addressRead
        if (reset)
        begin
            addressRead <= -1;
        end
        else
        begin
            if (fetch_new_ins)
            begin
                addressRead <= addressRead + 1;
            end
        end
    end
    
    always @(posedge clk)
    begin
    //Output: 
        //fetch_new_ins
        if (reset)
        begin
            fetch_new_ins <= 1;
            read_E <= 0;
            write_E <= 0;
            hits <= 0;
            misses <= 0;
            references_count <= 0;
        end
        else
        begin
            if (fetch_new_ins) 
            begin
                references_count <= references_count + 1;
                fetch_new_ins <= 0;
                read_E <= 1;
            end
            else if (read_E)
            begin
                if (hit == 0)
                begin //MISS
                    write_E <= 1;
                    misses <= misses + 1;
                    read_E <= 0;
                end
                else if (hit  == 1)
                begin //HIT
                    hits <= hits + 1;
                    fetch_new_ins <= 1;
                    read_E <= 0;
                end
            end
            else if (write_E)
            begin
                write_E <= 0;
                fetch_new_ins <= 1;
            end
            
        end
    end
    
    
endmodule
