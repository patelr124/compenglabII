`timescale 1ns / 1ps

module Cache(
    input clk,
    input index,
    input [19:0] tag,
    input write_E,
    input read_E,
    input fetch_new_ins,
    output reg hit //1 = miss, 2 = hit
    );
    reg [6:0] wr_ind1;
    reg [6:0] wr_ind0;
    reg [6:0] LRU_p1 [127:0];
    reg [6:0] LRU_p0 [127:0];
    
    reg v_ind1 [127:0]; //128 1 bits
    reg v_ind0 [127:0]; //128 1 bits
    reg [19:0] tag_b1 [127:0]; //128 20 bits 
    reg [19:0] tag_b0 [127:0]; //128 20 bits
    integer i;
    
    initial 
    begin
        wr_ind1 = 0;
        wr_ind0 = 0;
        hit = 0;
        for (i=0;i<128;i=i+1) begin
            v_ind1[i] = 0; 
            v_ind0[i] = 0; 
            tag_b1[i] = 0; 
            tag_b0[i] = 0; 
            LRU_p1[i] = 0;
            LRU_p0[i] = 0;
        end
    end
    
    wire write_EPosedgePulse;
    wire write_ENegedgePulse;
    reg write_E_p1;
    always @(posedge clk)
    begin
        write_E_p1 <= write_E;
    end
    assign write_EPosedgePulse = write_E && ~write_E_p1;
    assign write_ENegedgePulse = ~write_E && write_E_p1;
    
    reg [6:0] hitindex;   

    integer j;
    integer k;
    integer m;
    always @(*)
    begin
        ///////////////////////////////////////////////////////////////////////////////
        //Sets the write index as the lowest value in the replacement priority array
        wr_ind1 = 0;
        wr_ind0 = 0;
        for (j = 1; j < 128; j=j+1)
        begin
            if (LRU_p1[j]< LRU_p1[wr_ind1])
            begin
                wr_ind1 = j;
            end
            if (LRU_p0[j]< LRU_p0[wr_ind0])
            begin
                wr_ind0 = j;
            end
        end
    end

    always @(posedge clk)
    begin
        ///////////////////////////////////////////////////////////////////////////////
        //Decrement all values that are greater than the new index
        if (write_ENegedgePulse)
        begin
            //if (j == 128)
            //begin
                for (m = 0; m < 128; m = m + 1)
                begin
                    if (index == 1)
                    begin
                        if (LRU_p1[m] > LRU_p1[wr_ind1])
                        begin
                            LRU_p1[m] <= LRU_p1[m] - 1;
                        end
                    end
                    else if (index == 0)
                    begin
                        if (LRU_p0[m] > LRU_p0[wr_ind0])
                        begin
                            LRU_p0[m] <= LRU_p0[m] - 1;
                        end
                    end
                end
                if (m == 128)
                begin
                    if (index == 1)
                    begin
                        tag_b1[wr_ind1] <= tag;
                        v_ind1[wr_ind1] <= 1;
                        LRU_p1[wr_ind1] <= 127;
                    end
                    else if (index == 0)
                    begin
                        tag_b0[wr_ind0] <= tag;
                        v_ind0[wr_ind0] <= 1;
                        LRU_p0[wr_ind0] <= 127;
                    end
                end
            //end
        end      
        if (hit == 1)
        begin
            for (k = 0; k < 128; k=k+1)
            begin
                if (index == 1)
                begin
                    if (LRU_p1[k] > LRU_p1[hitindex])
                    begin
                        LRU_p1[k] <= LRU_p1[k] - 1;
                    end                
                end
                else if (index == 0)    
                begin
                    if (LRU_p0[k] > LRU_p0[hitindex])
                    begin
                        LRU_p0[k] <= LRU_p0[k] - 1;
                    end
                end                
            end
            if (k == 128)
            begin
                if (index == 1)
                begin
                    LRU_p1[hitindex] <= 127;
                end
                else if (index == 0)
                begin
                    LRU_p0[hitindex] <= 127;
                end                
            end
        end
    end

    reg [19:0] DebugtagBitsAtI;
    reg [19:0] DebugtagBitsAt0;

        
    always @(*)
    begin
    //Replace tag and Set Valid
        if (read_E)
        begin
            hit = 0;
            if (index == 1)
            begin
                for (i=0;i<128;i=i+1)
                begin
                    if (tag_b1[i] == tag)
                    begin
                        hitindex = i;
                        DebugtagBitsAtI = tag_b1[i];
                        hit = 1;
                    end
                end
            end 
            else if (index  == 0)
            begin
                for (i=0;i<128;i=i+1)
                begin
                    if (tag_b0[i] == tag)
                    begin
                        hitindex = i;
                        DebugtagBitsAt0 = tag_b0[i];
                        hit = 1;
                    end
                end
            end       
        end 
        else if (fetch_new_ins)
        begin
            hit = 0;
            hitindex = 0;
        end 
    end 
    
endmodule
