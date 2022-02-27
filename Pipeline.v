module Pipeline(clk, reset, PC_start, PC_fetch, mem_out);
    input wire clk;
	input wire reset;
	input wire [63:0] PC_start;

    /* Outputs - Nachecken als deze kloppen bij het testen */
    output wire [63:0] PC_fetch;
	output wire [63:0] mem_out; 

	/* Stage 1 - Instruction Fetch (IF) */
    reg [63:0] current_PC;
    wire [63:0] PC_next;
	wire [63:0] current_PC_4;
	wire [31:0] instruction;
	
	/* Stage 1/2 - IF/ID Registers */
	reg [63:0] current_PC2;
	reg [31:0] instruction2;
	
	/* Stage 2 - Instruction Decode (ID) connections */
	wire [63:0] data;
	wire [63:0] r1_data;
	wire [63:0] r2_data;

    wire [4:0] rb;
	wire [4:0] reg_write;

	wire [63:0] sign_ext;
	wire [1:0] ALU_op;

    wire ALUSrc, Mem2Reg, RegWrite, MemRead, MemWrite, Branch, Unconbranch;

    /* voor signalen te hergebruiken, maken we een dummywire */
    wire dummyWire; 
	
	/* Stage 2/3 - ID/EX Registers */
	reg ALUSrc3, Mem2Reg3, RegWrite3, MemRead3, MemWrite3, Branch3, Unconbranch3;
	reg [1:0] ALU_op3;
	reg [63:0] current_PC3;
	reg [63:0] r1_data_3;
	reg [63:0] r2_data_3;
	reg [63:0] sign_ext3;
	reg [10:0] Opcode3;
	reg [4:0] Rd3; 
	
	/* Stage 3 - EX Connections */
	wire [63:0] shiftedImm;
	wire [63:0] branchAddr;
	wire [63:0] ALUBusB;
	wire [3:0] ALUControlBits;
	wire [63:0] ALUBusW;
	wire ALUZero;
	
	/* Stage 3/4 EX/MEM Registers */
	reg [63:0] branchAddr4;
	reg [63:0] ALUBusW4;
	reg ALUZero4;
	reg [63:0] r2_data_4;
	reg [4:0] Rd4;
	reg Mem2Reg4, RegWrite4, MemRead4, MemWrite4, Branch4, Unconbranch4;
	
	/* Stage 4 - MEM Connections */
	wire [63:0] MemReadBus;
	wire PCSrc;
	
	/* Stage 4/5 MEM/WB Registers */
	reg [63:0] MemReadBus5;
	reg [63:0] ALUBusW5;
	reg [4:0] Rd5;
	reg Mem2Reg5, RegWrite5;
	
	/* Stage 5 WB Connections */
	wire [63:0] RegBusW;

    /* Assigns toepassen voor de testwaarden na te kijken zoals boven aangegeven. */
	assign mem_out  MemReadBus5;
	assign PC_fetch  current_PC; 

    /* Stage 1 - IF Logic */
	assign PC_next  PCSrc ? branchAddr4 : current_PC_4;
	assign current_PC_4  current_PC + 64'd4;

	always@(posedge clk)
	begin
		if(reset)
			current_PC <= PC_start;
		else
			current_PC <= PC_next;
	end
	
	InstructionMemory InstructionMem(
        .Instruction(instruction), 
		.Address(current_PC)
    );
							 
	/* Stage 1/2 - IF/ID Registers */
	
	always@(posedge clk or posedge reset)
	begin
		if(reset)
		begin
			current_PC2 <= 64'd0;
			instruction2 <= 64'd0;
		end
		else
		begin
			current_PC2 <= current_PC;
			instruction2 <= instruction;
		end
	end

    /* Stage 2 - ID Logic */
	
	assign rb  instruction2[28] ? instruction2[4:0] : instruction2[20:16];

	RegisterFile RegFile(
        .R1(instruction2[9:5]), 
        .R2(rb),
        .reg_write(Rd5), 
        .data(RegBusW),
        .WE(RegWrite5), 
        .r1_data(r1_data), 
        .r2_data(r2_data), 
        .clk(clk),
		.reset(reset)
    );
				 
	sign_extend sign_extention(
        .extended(sign_ext), 
        .Instruction(instruction2)
    );
				  
	Control control(
        .OPCode(instruction2[31:21]),
        .Reg2Loc(dummyWire),        //het hergebruiken van de control unit, gebruiken we de dummy wire.
        .Branch(Branch),
		.UnconBranch(Unconbranch),
        .MemRead(MemRead),
        .Mem2Reg(Mem2Reg), 
        .ALUOP(ALU_op), 
        .MemWrite(MemWrite), 
        .ALUSrc(ALUSrc), 
        .RegWrite(RegWrite)
    );
	
    /* Stage 2/3 - ID/EX Registers */
	always@(posedge clk or posedge reset)
	begin
		if(reset)
		begin
			ALUSrc3 <= 1'b0;
			Mem2Reg3 <= 1'b0;
			RegWrite3 <= 1'b0;
			MemRead3 <= 1'b0;
			MemWrite3 <= 1'b0;
			Branch3 <= 1'b0;
			Unconbranch3 <= 1'b0;
			ALU_op3 <= 2'b00;
	        current_PC3 <= 64'b0;
	        r1_data_3 <= 64'd0;
	        r2_data_3 <= 64'd0;
	        sign_ext3 <= 64'd0;
	        Opcode3 <= 10'b0;
	        Rd3 <= 5'b0; 
		end
		else
		begin
			ALUSrc3 <= ALUSrc;
			Mem2Reg3 <= Mem2Reg;
			RegWrite3 <= RegWrite;
			MemRead3 <= MemRead;
			MemWrite3 <= MemWrite;
			Branch3 <= Branch;
			Unconbranch3 <= Unconbranch;
			ALU_op3 <= ALU_op;
	        current_PC3 <= current_PC2;
	        r1_data_3 <= r1_data;
	        r2_data_3 <= r2_data;
	        sign_ext3 <= sign_ext;
	        Opcode3 <= instruction2[31:21];
	        Rd3 <= instruction2[4:0]; 
		end
	end

    /* Stage 3 - EX Logic */
	
	assign shiftedImm  sign_ext3 << 2;  //zoals de PC counter can single cycle, zie scheme single cycle boek p.272
	assign branchAddr  current_PC3 + shiftedImm;
	assign ALUBusB  ALUSrc3 ? sign_ext3 : r2_data_3;

	ALUControl ALUCont(
        .Operation(ALUControlBits), 
        .ALUOP(ALU_op3), 
        .OPCode(Opcode3)
    );
       
	ALU ALUcomp(
        .Out(ALUBusW), 
	    .R1(r1_data_3), 
	    .R2(ALUBusB), 
	    .Mode(ALUControlBits), 
	    .Zero(ALUZero)
    );

    /* Stage 3/4 - EX/MEM Registers */
	always@(posedge clk or posedge reset)
	begin
	    if(reset)
	    begin
	        branchAddr4 <= 64'b0;
	        ALUBusW4 <= 64'b0;
	        ALUZero4 <= 1'b0;
	        r2_data_4 <= 63'b0;
	        Rd4 <= 5'b0;
	        Mem2Reg4 <= 1'b0;
	        RegWrite4 <= 1'b0;
	        MemRead4 <= 1'b0;
	        MemWrite4 <= 1'b0;
	        Branch4 <= 1'b0;
	        Unconbranch4 <= 1'b0;
	    end
	    else
	    begin
	        branchAddr4 <= branchAddr;
	        ALUBusW4 <= ALUBusW;
	        ALUZero4 <= ALUZero;
	        r2_data_4 <= r2_data_3;
	        Rd4 <= Rd3;
	        Mem2Reg4 <= Mem2Reg3;
	        RegWrite4 <= RegWrite3;
	        MemRead4 <= MemRead3;
	        MemWrite4 <= MemWrite3;
	        Branch4 <= Branch3;
	        Unconbranch4 <= Unconbranch3;
	    end
	end

    /* Stage 4 - MEM Logic */
	assign PCSrc  ((ALUZero4 & Branch4) | Unconbranch4);

	dataMemory DMem(
        .Readdata(MemReadBus),
        .Address(ALUBusW4), 
        .Writedata(r2_data_4), 
        .MemoryRead(MemRead4), 
        .MemoryWrite(MemWrite4), 
        .clk(clk)
    );
	
	/* Stage 4/5 - MEM/WB Registers */
	
	always@(posedge clk or posedge reset)
	begin
	    if(reset)
	    begin
		MemReadBus5 <= 64'b0;
		ALUBusW5 <= 64'b0;
		Rd5 <= 5'b0;
		Mem2Reg5 <= 1'b0;
		RegWrite5 <= 1'b0;
	    end
	    else
	    begin
		MemReadBus5 <= MemReadBus;
		ALUBusW5 <= ALUBusW4;
		Rd5 <= Rd4;
		Mem2Reg5 <= Mem2Reg4;
		RegWrite5 <= RegWrite4;
	    end
	end
	
	/* Stage 5 - WB Logic */
	assign RegBusW  Mem2Reg5 ? MemReadBus5 : ALUBusW5;
	
endmodule
	