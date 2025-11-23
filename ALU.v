module ALU(output reg [15:0]Result,output reg [5:0]Status,
            input [15:0] A,B,
            input [4:0] F,
            input Cin);
    // All functions Needed in ALU
    localparam   [4:0]
                INC = 5'b00001,
                DEC = 5'b00011,
                ADD = 5'b00100,
                ADD_CARRY = 5'b00101,
                SUB = 5'b00110,
                SUB_BORROW = 5'b00111,
                AND = 5'b01000,
                OR = 5'b01001,
                XOR = 5'b01010,
                NOT = 5'b01011,
                SHL = 5'b10000,
                SHR = 5'b10001,
                SAL = 5'b10010,
                SAR = 5'b10011,
                ROL = 5'b10100,
                ROR = 5'b10101,
                RCL = 5'b10110,
                RCR = 5'b10111;
    // FLags
    localparam  
                CARRY_F = 5,
                ZERO_F = 4,
                NEGATIVE_F = 3,
                OVERFLOW_F = 2,
                PARITY_F = 1,
                AUX_CARRY_F = 0;
    // Internal signals for arithmetic operations
    wire [16:0] add_result = A + B + ((F == ADD_CARRY) ? Cin : 1'b0);
    wire [16:0] sub_result = A - B - ((F == SUB_BORROW) ? Cin : 1'b0);
    wire [16:0] inc_result = A + 17'h00001;
    wire [16:0] dec_result = A - 17'h00001;
    // Result Setting
    always @(*) begin
        case (F)
            INC : Result = A + 16'h0001;
            DEC : Result = A - 16'h0001;
            ADD : Result = A + B;
            ADD_CARRY : Result = A + B + Cin;
            SUB : Result = A - B;
            SUB_BORROW : Result = A - B - Cin;
            AND : Result = A & B;
            OR : Result =  A | B;
            XOR : Result = A ^ B;
            NOT : Result = ~A;
            SHL : Result = A << 1;
            SHR : Result = A >> 1;
            SAL : Result = A <<< 1;
            SAR : Result = {A[15], A[15:1]};
            ROL : Result = {A[14:0],A[15]};
            ROR : Result = {A[0],A[15:1]};
            RCL : Result = {A[14:0],Cin};
            RCR : Result = {Cin,A[15:1]};
            default: Result = 16'h0000;
        endcase
    end
    // Status Setting
    always @(*) begin
        Status[ZERO_F] = (Result == 16'h0000);
        Status[NEGATIVE_F] = Result[15];              // MSB
        Status[PARITY_F] = ~(^Result);
        Status[CARRY_F] = 1'b0;
        Status[OVERFLOW_F] = 1'b0;
        Status[AUX_CARRY_F] = 1'b0;
        case (F)
            ADD,ADD_CARRY,SUB,SUB_BORROW,INC,DEC: begin
                // SUB Operation
                if(F[1]) begin
                    Status[CARRY_F] = (F == DEC) ? dec_result[16] : sub_result[16];
                    Status[OVERFLOW_F] = (A[15] != B[15]) && (Result[15] != A[15]);
                    if(F != DEC) Status[AUX_CARRY_F] = (A[3:0] < B[3:0]);
                    else Status[AUX_CARRY_F] = (A[3:0] < (B[3:0] + Cin));
                end
                // ADD Operation
                else begin
                    Status[CARRY_F] = (F == INC) ? inc_result[16] : add_result[16];
                    Status[OVERFLOW_F] = (A[15] == B[15]) && (Result[15] != A[15]);
                    if(F != INC) Status[AUX_CARRY_F] = (A[3:0] + B[3:0] > 4'hF);
                    else Status[AUX_CARRY_F] = (A[3:0] + B[3:0] + Cin > 4'hF);
                end
            end
            ROL,RCL,SHL,SAL:begin
                Status[CARRY_F] = A[15];
            end
            ROR,RCR,SAR,SHR:begin
                Status[CARRY_F] = A[0];
            end
            default: begin
                Status[CARRY_F] = 1'b0;
                Status[OVERFLOW_F] = 1'b0;
                Status[AUX_CARRY_F] = 1'b0;
            end
        endcase
    end
endmodule

`timescale 1ps/1ps
module TB_ALU;
wire [15:0] Result;
wire [5:0] Status;
reg [15:0] A,B;
reg [4:0] F;
reg Cin;
ALU ALU_BlockTest (.Result(Result),.Status(Status),.A(A),.B(B),.F(F),.Cin(Cin));
initial begin
    #240 $finish;
end
initial begin
    $dumpfile("ALU_TESTBENCH.vcd");
    $dumpvars(0,TB_ALU);
end
initial fork 
    // operations
    #0 {A,B,F,Cin} = {16'h0000,16'h0000,5'b00000,1'b1};
    #10 {A,B,F,Cin} = {16'h0000,16'h0000,5'b00010,1'b1};
    #20 {A,B,F,Cin} = {16'h0000,16'h0000,5'b00001,1'b1};
    #30 {A,B,F,Cin} = {16'h0001,16'h0000,5'b00001,1'b1};
    #40 {A,B,F,Cin} = {16'h0001,16'h0000,5'b00011,1'b1};
    #50 {A,B,F,Cin} = {16'h0003,16'h0004,5'b00100,1'b1};
    #60 {A,B,F,Cin} = {16'h0003,16'h0004,5'b00101,1'b1};
    #70 {A,B,F,Cin} = {16'h0007,16'h0003,5'b00110,1'b1};
    #80 {A,B,F,Cin} = {16'h0007,16'h0003,5'b00111,1'b1};
    #90 {A,B,F,Cin} = {16'h0007,16'h0003,5'b01000,1'b1};
    #100 {A,B,F,Cin} = {16'h0007,16'h0003,5'b01001,1'b1};
    #110 {A,B,F,Cin} = {16'h0007,16'h0003,5'b01010,1'b1};
    #120 {A,B,F,Cin} = {16'h0007,16'h0003,5'b01011,1'b1};
    #130 {A,B,F,Cin} = {16'h80f0,16'h0003,5'b10000,1'b1};
    #140 {A,B,F,Cin} = {16'h0f0f,16'h0003,5'b10001,1'b1};
    #150 {A,B,F,Cin} = {16'hf0f0,16'h0003,5'b10010,1'b1};
    #160 {A,B,F,Cin} = {16'hf0f1,16'h0003,5'b10011,1'b1};
    #170 {A,B,F,Cin} = {16'hf0f0,16'h0003,5'b10100,1'b1};
    #180 {A,B,F,Cin} = {16'h0f0f,16'h0003,5'b10101,1'b1};
    #190 {A,B,F,Cin} = {16'h0f0f,16'h0003,5'b10110,1'b1};
    #200 {A,B,F,Cin} = {16'h3f00,16'h0003,5'b10111,1'b1};
    // FLags
join
endmodule