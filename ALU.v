module ALU(out, r1, r2, tag, zero);

    input[64:0] r1, r2;
    input[3:0] tag;
    output[63:0] out;
    reg[63:0] out;
    
    output wire zero;
    
    always @(*)
    begin
        case(tag)
            4'b0000: begin // AND
            out = r1 & r2;
            end
            4'b0001: begin // ORR
            out = r1 | r2;
            end
            4'b0010: begin // ADD
            out = r1 + r2;
            end
            4'b0011: begin // LSL
            out = r1 << r2;
            end
            4'b0100: begin // LSR
            out = r1 >> r2;
            end
            4'b0111: begin // pass through
            out = r2;
            end
         endcase  
    end
    assign zero = (out == 64'b0);
endmodule
    