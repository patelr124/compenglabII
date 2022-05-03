`timescale 1ns / 1ps

`define L  (8) //Num Bytes per line
`define K  (2) //2, 4, 8, 16 //Number lines per set
`define KN (256) //64, 256 //#Num Sets
`define Sets (128) //64, 256 //#Num Sets in cache
`define DataLength (57961) //Num datapoints Trace1
 //parameter DataLength = 59856; //Num datapoints Trace2\

module testbench();

    reg clk;
    
    //Generate Clock
    initial
    begin
        clk = 0;
        while(1)
        begin
            #5 clk = ~clk;
        end
    end

    //Reset
    reg reset; //Active Low
    initial
    //Initize reset
    begin
        reset = 1;
        #20 reset = 0;
    end

    wire [23:0] reference;
    wire [19:0] tag;
    wire index;
    wire fetch_new_ins;

    
    //memory
    wire [63:0] addressRead;
    memory u_memory(
        .clk(clk),
        .reset(reset),
        .reference(reference),
        .addressRead(addressRead),
        .fetch_new_ins(fetch_new_ins)
    );

    //Cache Control
    //Produces hit/Miss
    wire write_E;
    wire read_E;
    wire hit;
    
    CacheControl u_CacheControl(
        .clk(clk),
        .reset(reset),
        .reference(reference),
        .tag(tag),
        .index(index),
        .write_E(write_E),
        .read_E(read_E),
        .addressRead(addressRead),
        .fetch_new_ins(fetch_new_ins),
        .hit(hit)
    );
   
    //CacheMem

    
    Cache u_Cache(
        .clk(clk),
        .index(index),
        .tag(tag),
        .hit(hit),
        .write_E(write_E),
        .read_E(read_E),
        .fetch_new_ins(fetch_new_ins)
    );
    
endmodule
