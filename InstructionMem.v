/*
 * 2 NOPs are needed to prevent Data hazards for R-types 3 are needed for loads"
 * End result should be the following => 11111000010 000101000 00 11111 01010
*/

`define NOP 32'h8B1F03FF 						// opcode for no operation aka stall instruction for pipelining

module InstructionMemory(instruction, address);
    input [63:0] address;            			//64 bit address-code

    output [31:0] instruction;       			//32 bit instructiecode uitvoer

    reg [31:0] instruction;
    
    always @ (address) begin
        case(address)
    	64'h000: instruction = 32'hF84083EA; 	//LDUR X10, [XZR, 0x8]
    	64'h004: instruction = 32'hF84103EB; 	//LDUR X11, [XZR, 0x10]
    	64'h008: instruction = 32'hF84183EC; 	//LDUR X12, [XZR, 0x18]
		64'h00c: instruction = 32'hF84003E9; 	//LDUR X9, [XZR, 0x0]
		64'h010: instruction = 32'hF84203ED; 	//LDUR X13, [XZR, 0x20]
		64'h014: instruction = 32'hAA0B014A; 	//ORR X10, X10, X11
		64'h018: instruction = `NOP; 			//Stall
		64'h01c: instruction = `NOP; 			//Stall dependency on X10
		64'h020: instruction = 32'h8A0A018C; 	//AND X12, X12, X10
		64'h024: instruction = `NOP; 			//Stall
		64'h028: instruction = `NOP; 			//Stall dependency on X12
		64'h02c: instruction = 32'hB400014C; 	//CBZ X12, end ; loop
		64'h030: instruction = `NOP; 			//Stall
		64'h034: instruction = `NOP; 			//Stall
		64'h038: instruction = `NOP; 			//Stall Don't want to run the next instruction if we are branching
		64'h03c: instruction = 32'h8B0901AD; 	//ADD X13, X13, X9
		64'h040: instruction = 32'hCB09018C; 	//SUB X12, X12, X9
		64'h044: instruction = 32'h17FFFFFA; 	//B loop
		64'h048: instruction = `NOP; 			//Stall
		64'h04c: instruction = `NOP; 			//Stall
		64'h050: instruction = `NOP; 			//Stall Don't want to run the next instruction when we are branching
		64'h054: instruction = 32'hF80203ED; 	//STUR X13, [XZR, 0x20] ; end
		64'h058: instruction = 32'hF84203ED;  	//One last load to place stored value on memdbus for test checking.; LDUR X13,[XZR, 0x20]
		64'h05c: instruction = `NOP;
		64'h060: instruction = `NOP;
		64'h064: instruction = `NOP;
		64'h068: instruction = `NOP; 			//Add stalls to wait for the value to be placed on the MemBus in the WriteBack stage
		63'h06c: instruction = 32'h8B1F03E9; 	// start of tests
		63'h070: instruction = `NOP;
		63'h074: instruction = `NOP;
		63'h078: instruction = 32'hB2048D29; 	// 0x123
		63'h07c: instruction = `NOP;
		63'h080: instruction = `NOP;
		63'h084: instruction = 32'hD37F3129; 	// shift
		63'h088: instruction = `NOP;
		63'h08c: instruction = `NOP;
		63'h090: instruction = 32'hB2115929; 	// 0x456
		63'h094: instruction = `NOP;
		63'h098: instruction = `NOP;
		63'h09c: instruction = 32'hD37F3129; 	// shift
		63'h0a0: instruction = `NOP;
		63'h0a4: instruction = `NOP;
		63'h0a8: instruction = 32'hB21E2529; 	// 0x789
		63'h0ac: instruction = `NOP;
		63'h0b0: instruction = `NOP;
		63'h0b4: instruction = 32'hD37F3129; 	// shift
		63'h0b8: instruction = `NOP;
		63'h0bc: instruction = `NOP;
		63'h0c0: instruction = 32'hB22AF129; 	// 0xabc
		63'h0c4: instruction = `NOP;
		63'h0c8: instruction = `NOP;
		63'h0cc: instruction = 32'hD37F3129; 	// shift
		63'h0d0: instruction = `NOP;
		63'h0d4: instruction = `NOP;
		63'h0d8: instruction = 32'hB237BD29; 	// 0xdef
		63'h0dc: instruction = `NOP;
		63'h0e0: instruction = `NOP;
		63'h0e4: instruction = 32'hD37F1129; 	// shift 4
		63'h0e8: instruction = `NOP;
		63'h0ec: instruction = `NOP;
		63'h0f0: instruction = 32'hF80283E9; 	// stur
		63'h0f4: instruction = `NOP;
		63'h0f8: instruction = `NOP;
		63'h0fc: instruction = `NOP;
		63'h100: instruction = 32'hF84283EA; 	// ldur
		default: instruction = `NOP;
        endcase
    end
endmodule   