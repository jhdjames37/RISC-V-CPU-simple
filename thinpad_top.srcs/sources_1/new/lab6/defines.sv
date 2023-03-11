// some constants needed to be defined
`ifndef __DEFINES__ // I don't know whether header protection is needed
 `define __DEFINES__


 `define ALU_SEL_PC 2'd0
 `define ALU_SEL_RS1 2'd1
 // ALU_SEL_CSR  2'd2 reserved here

 `define ALU_SEL_IMM 2'd0
 `define ALU_SEL_RS2 2'd1
 `define ALU_SEL_CSR 2'd2

 `define RD_SEL_ALU 2'd0
 `define RD_SEL_MEM 2'd1
 `define RD_SEL_PC 2'd2
 `define RD_SEL_CSR 2'd3

 `define ALU_OP_PLUS 5'd1
 `define ALU_OP_MINUS 5'd2
 `define ALU_OP_AND 5'd3
 `define ALU_OP_OR 5'd4
 `define ALU_OP_XOR 5'd5
 `define ALU_OP_NOT 5'd6
 `define ALU_OP_SLL 5'd7
 `define ALU_OP_SRL 5'd8
 `define ALU_OP_SRA 5'd9
 `define ALU_OP_ROL 5'd10
 `define ALU_OP_SEL_B 5'd11
 `define ALU_OP_SEL_A 5'd12
 `define ALU_OP_BIT_CLEAR 5'd13
 `define ALU_OP_SLTU 5'd14
 `define ALU_OP_SLT 5'd15
 `define ALU_OP_CTZ 5'd29
 `define ALU_OP_MIN 5'd30
 `define ALU_OP_ANDN 5'd31

 `define SRAM_SEL_1 2'b00
 `define SRAM_SEL_2 2'b01
 `define SRAM_SEL_4 2'b10

 `define MODE_M 2'b11
 `define MODE_S 2'b01
 `define MODE_U 2'b00

 `define INT_NONE 2'b00
 `define INT_TIMER_M 2'b01
 `define INT_TIMER_S 2'b10

 `define PC_ADD4 2'b00
 `define PC_BRANCH 2'b01
 `define PC_TRAP 2'b10

 `define BRANCH_EQU 3'd0
 `define BRANCH_NEQ 3'd1
 `define BRANCH_LT 3'd2
 `define BRANCH_GE 3'd3
 `define BRANCH_LTU 3'd4
 `define BRANCH_GEU 3'd5

 `define LOAD_EXT_ZERO 1'b0
 `define LOAD_EXT_SIGNED 1'b1
 
`endif //__DEFINES__

