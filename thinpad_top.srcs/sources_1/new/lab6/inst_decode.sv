`include "defines.sv"

module inst_decode(
  input wire [31:0] inst_i,
  input wire [1:0]  mode_i,

  output reg [31:0] imm_o,

  output reg [4:0]  rs1_o,
  output reg [4:0]  rs2_o,
  
  output reg [1:0]  alu_sel_a_o,
  output reg [1:0]  alu_sel_b_o,
  output reg [4:0]  alu_op_o,

  output reg        sram_we_o,
  output reg        sram_re_o,
  output reg [1:0]  sram_sel_o,
  output reg [1:0]  rd_sel_o,
  output reg [4:0]  rd_o,
  output reg        rd_we_o,

  // branch enable, branch type, branch is j-type
  output reg        branch_o,
  output reg [2:0]  branch_type_o,
  output reg        branch_ja_o,

  // will be used one day. Now for test and debug.
  output reg        invalid_inst_o,

  // CSR regs
  output reg [11:0] csr_addr_o,
  output reg        csr_re_o,
  output reg        csr_we_o,
  output reg [4:0]  csr_imm_o, 

  // FENCE.I
  output reg        fencei_o,

  // ecall, ebreak, mret
  output reg        ebreak_o,
  output reg        ecall_o,
  output reg        mret_o,
  output reg        sret_o,
  output reg        sfence_o,

  output reg        load_ext_type
  );

  assign rs1_o = inst_i[19:15];
  assign rs2_o = inst_i[24:20];

  assign rd_o = inst_i[11:7];
  
  typedef enum logic [2:0] {
               INST_R,
               INST_I,
               INST_S,
               INST_B,
               INST_U,
               INST_J
               } inst_type;
  inst_type imm_type;

  enum logic [1:0] {
       LEN_1,
       LEN_2,
       LEN_4
       } io_len;  

  logic invalid_op;
  assign invalid_inst_o = invalid_op;
  assign csr_addr_o = inst_i[31:20];
  assign csr_imm_o = inst_i[19:15];  

  always_comb begin
    // default
    imm_type = INST_R;
    alu_sel_a_o = 2'bXX;
    alu_sel_b_o = 2'bXX;
    alu_op_o = 5'd0;
    
    sram_we_o = 1'b0;
    sram_re_o = 1'b0;

    rd_sel_o = 2'bXX;
    rd_we_o = 1'b0;

    branch_o = 1'b0;
    branch_type_o = 3'b0;
    branch_ja_o = 1'b0;
    io_len = LEN_1;

    invalid_op = 1'b0;

    fencei_o = 1'b0;

    ecall_o = 1'b0;
    ebreak_o = 1'b0;
    mret_o = 1'b0;
    sret_o = 1'b0;    
    sfence_o = 1'b0;

    csr_we_o = 1'b0;
    csr_re_o = 1'b0;    
    
    sfence_o = 1'b0;
    load_ext_type = 1'b0;

    casez ({inst_i[14:12],inst_i[6:0]})
      10'b???_0110111: begin // LUI
        imm_type = INST_U;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_SEL_B;
        rd_sel_o = `RD_SEL_ALU;
        rd_we_o = 1'b1;       
      end
      10'b???_0010111: begin // AUIPC
        imm_type = INST_U;
        alu_sel_a_o = `ALU_SEL_PC;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        rd_sel_o = `RD_SEL_ALU;
        rd_we_o = 1'b1;       
      end
      10'b000_1100011: begin // BEQ
        imm_type = INST_B;
        alu_sel_a_o = `ALU_SEL_PC;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        branch_o = 1'b1;        
        branch_type_o = `BRANCH_EQU;
      end
      10'b001_1100011: begin // BNE
        imm_type = INST_B;
        alu_sel_a_o = `ALU_SEL_PC;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        branch_o = 1'b1;
        branch_type_o = `BRANCH_NEQ;
      end
      10'b100_1100011: begin // BLT
        imm_type = INST_B;
        alu_sel_a_o = `ALU_SEL_PC;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        branch_o = 1'b1;
        branch_type_o = `BRANCH_LT;
      end
      10'b101_1100011: begin // BGE
        imm_type = INST_B;
        alu_sel_a_o = `ALU_SEL_PC;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        branch_o = 1'b1;
        branch_type_o = `BRANCH_GE;
      end
      10'b110_1100011: begin // BLTU
        imm_type = INST_B;
        alu_sel_a_o = `ALU_SEL_PC;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        branch_o = 1'b1;
        branch_type_o = `BRANCH_LTU;
      end
      10'b111_1100011: begin // BGEU
        imm_type = INST_B;
        alu_sel_a_o = `ALU_SEL_PC;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        branch_o = 1'b1;
        branch_type_o = `BRANCH_GEU;
      end
      10'b???_1101111: begin // JAL
        imm_type = INST_J;
        alu_sel_a_o = `ALU_SEL_PC;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        branch_o = 1'b1;
        branch_ja_o = 1'b1;
        rd_sel_o = `RD_SEL_PC;
        rd_we_o = 1'b1;
      end
      10'b000_1100111: begin // JALR
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        branch_o = 1'b1;
        branch_ja_o = 1'b1;
        rd_sel_o = `RD_SEL_PC;
        rd_we_o = 1'b1;
      end
      10'b000_0000011: begin // LB
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        sram_re_o = 1'b1;
        io_len = LEN_1;
        rd_sel_o = `RD_SEL_MEM;
        rd_we_o = 1'b1;
        load_ext_type = `LOAD_EXT_SIGNED;
      end
      10'b001_0000011: begin // LH
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        sram_re_o = 1'b1;
        io_len = LEN_2;
        rd_sel_o = `RD_SEL_MEM;
        rd_we_o = 1'b1;
        load_ext_type = `LOAD_EXT_SIGNED;
      end
      10'b010_0000011: begin // LW
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        sram_re_o = 1'b1;
        io_len = LEN_4;
        rd_sel_o = `RD_SEL_MEM;
        rd_we_o = 1'b1;
      end
      10'b100_0000011: begin // LBU
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        sram_re_o = 1'b1;
        io_len = LEN_1;
        rd_sel_o = `RD_SEL_MEM;
        rd_we_o = 1'b1;
      end
      10'b101_0000011: begin // LHU
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        sram_re_o = 1'b1;
        io_len = LEN_2;
        rd_sel_o = `RD_SEL_MEM;
        rd_we_o = 1'b1;
      end
      10'b000_0100011: begin // SB
        imm_type = INST_S;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        sram_we_o = 1'b1;
        io_len = LEN_1;
      end
      10'b001_0100011: begin // SH
        imm_type = INST_S;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        sram_we_o = 1'b1;
        io_len = LEN_2;        
      end
      10'b010_0100011: begin // SW
        imm_type = INST_S;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        sram_we_o = 1'b1;
        io_len = LEN_4;
      end
      10'b000_0010011: begin // ADDI
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_PLUS;
        rd_sel_o = `RD_SEL_ALU;        
        rd_we_o = 1'b1;        
      end
      10'b010_0010011: begin // SLTI
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_SLT;
        rd_sel_o = `RD_SEL_ALU;        
        rd_we_o = 1'b1;        
      end
      10'b011_0010011: begin // SLTIU
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_SLTU;
        rd_sel_o = `RD_SEL_ALU;        
        rd_we_o = 1'b1;        
      end
      10'b111_0010011: begin // ANDI
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_AND;
        rd_sel_o = `RD_SEL_ALU;        
        rd_we_o = 1'b1;        
      end
      10'b100_0010011: begin // XORI
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_XOR;
        rd_sel_o = `RD_SEL_ALU;        
        rd_we_o = 1'b1;        
      end
      10'b110_0010011: begin // ORI
        imm_type = INST_I;
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_IMM;
        alu_op_o = `ALU_OP_OR;
        rd_sel_o = `RD_SEL_ALU;        
        rd_we_o = 1'b1;        
      end
      10'b001_0010011: begin // SLLI
        if (inst_i[31:25] != 7'b0000000) begin
          invalid_op = 1'b1;          
        end else begin
          imm_type = INST_I;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_IMM;
          alu_op_o = `ALU_OP_SLL;
          rd_sel_o = `RD_SEL_ALU;
          rd_we_o = 1'b1;
        end
      end
      10'b101_0010011: begin // SRLI
        if (inst_i[31:25] == 7'b0100000) begin // SRAI
          imm_type = INST_I;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_IMM;
          alu_op_o = `ALU_OP_SRA;
          rd_sel_o = `RD_SEL_ALU;
          rd_we_o = 1'b1;
        end else if (inst_i[31:25] != 7'b0000000) begin
          invalid_op = 1'b1; 
        end else begin
          imm_type = INST_I;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_IMM;
          alu_op_o = `ALU_OP_SRL;
          rd_sel_o = `RD_SEL_ALU;
          rd_we_o = 1'b1;
        end
      end // case: 10'b101_0010011
      10'b000_0110011: begin // ADD
        if (inst_i[31:25] == 7'b0100000) begin // SUB
          imm_type = INST_R;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_RS2;
          alu_op_o = `ALU_OP_MINUS;
          rd_sel_o = `RD_SEL_ALU;
          rd_we_o = 1'b1;
        end else if (inst_i[31:25] != 7'b0000000) begin
          invalid_op = 1'b1;          
        end else begin
          imm_type = INST_R;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_RS2;
          alu_op_o = `ALU_OP_PLUS;
          
          rd_sel_o = `RD_SEL_ALU;        
          rd_we_o = 1'b1;
        end
      end
      10'b001_0110011: begin // SLL
        if (inst_i[31:25] != 7'b0000000) begin
          invalid_op = 1'b1;          
        end else begin
          imm_type = INST_R;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_RS2;
          alu_op_o = `ALU_OP_SLL;
          rd_sel_o = `RD_SEL_ALU;        
          rd_we_o = 1'b1;
        end
      end
      10'b010_0110011: begin // SLT
        if (inst_i[31:25] != 7'b0000000) begin
          invalid_op = 1'b1;          
        end else begin
          imm_type = INST_R;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_RS2;
          alu_op_o = `ALU_OP_SLT;
          rd_sel_o = `RD_SEL_ALU;        
          rd_we_o = 1'b1;
        end
      end
      10'b011_0110011: begin // SLTU
        if (inst_i[31:25] != 7'b0000000) begin
          invalid_op = 1'b1;
        end else begin
          imm_type = INST_R;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_RS2;
          alu_op_o = `ALU_OP_SLTU;
          rd_sel_o = `RD_SEL_ALU;
          rd_we_o = 1'b1;
        end
      end
      10'b101_0110011: begin // SRL
        if (inst_i[31:25] == 7'b0100000) begin // SRA
          imm_type = INST_R;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_RS2;
          alu_op_o = `ALU_OP_SRA;
          rd_sel_o = `RD_SEL_ALU;
          rd_we_o = 1'b1;
        end else if (inst_i[31:25] != 7'b0000000) begin
          invalid_op = 1'b1;
        end else begin
          imm_type = INST_R;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_RS2;
          alu_op_o = `ALU_OP_SRL;
          rd_sel_o = `RD_SEL_ALU;
          rd_we_o = 1'b1;
        end
      end
      10'b111_0110011: begin // AND
        if (inst_i[31:25] != 7'b0000000) begin
          invalid_op = 1'b1;          
        end else begin
          imm_type = INST_R;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_RS2;
          alu_op_o = `ALU_OP_AND;
          rd_sel_o = `RD_SEL_ALU;        
          rd_we_o = 1'b1;
        end
      end
      10'b110_0110011: begin // OR
        if (inst_i[31:25] != 7'b0000000) begin
          invalid_op = 1'b1;          
        end else begin
          imm_type = INST_R;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_RS2;
          alu_op_o = `ALU_OP_OR;
          rd_sel_o = `RD_SEL_ALU;
          rd_we_o = 1'b1;
        end
      end
      10'b100_0110011: begin // XOR
        if (inst_i[31:25] != 7'b0000000) begin
          invalid_op = 1'b1;          
        end else begin
          imm_type = INST_R;
          alu_sel_a_o = `ALU_SEL_RS1;
          alu_sel_b_o = `ALU_SEL_RS2;
          alu_op_o = `ALU_OP_XOR;
          rd_sel_o = `RD_SEL_ALU;
          rd_we_o = 1'b1;
        end
      end

      10'b000_1110011: begin // ECALL & EBREAK, {U,S,M}RET, SFENCE.VMA
        casez (inst_i[31:7])
          25'b0000000_00000_00000_000_00000: // ECALL
            ecall_o = 1'b1;
          25'b0000000_00001_00000_000_00000: // EBREAK
            ebreak_o = 1'b1;
          25'b0011000_00010_00000_000_00000: begin // MRET
            if (mode_i != `MODE_M) begin
              invalid_op = 1'b1;
            end else begin
              mret_o = 1'b1;              
            end
          end
          25'b0001001_?????_?????_000_00000: begin // SFENCE_VMA
            if (mode_i != `MODE_M && mode_i != `MODE_S) begin
              invalid_op = 1'b1;
            end else begin
              sfence_o = 1'b1;              
            end
          end
          25'b0001000_00010_00000_000_00000: begin // SRET
            if (mode_i != `MODE_M && mode_i != `MODE_S) begin
              invalid_op = 1'b1;
            end else begin
              sret_o = 1'b1;
            end
          end
          default:
            invalid_op = 1'b1;          
        endcase // case (inst_i)        
      end

      10'b001_1110011: begin // CSRRW
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_CSR;
        alu_op_o = `ALU_OP_SEL_A;
        rd_sel_o = `RD_SEL_CSR;
        rd_we_o = 1'b1;
        csr_re_o = rd_o != 5'b0; // See spec.
        csr_we_o = 1'b1;        
      end

      10'b010_1110011: begin // CSRRS
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_CSR;
        alu_op_o = `ALU_OP_OR;
        rd_sel_o = `RD_SEL_CSR;
        rd_we_o = 1'b1;
        csr_re_o = 1'b1;        
        csr_we_o = rs1_o != 5'b0;        
      end

      10'b011_1110011: begin // CSRRC
        alu_sel_a_o = `ALU_SEL_RS1;
        alu_sel_b_o = `ALU_SEL_CSR;
        alu_op_o = `ALU_OP_BIT_CLEAR;
        rd_sel_o = `RD_SEL_CSR;
        rd_we_o = 1'b1;
        csr_re_o = 1'b1;        
        csr_we_o = rs1_o != 5'b0;        
      end

      10'b101_1110011: begin // CSRRWI
        alu_sel_a_o = `ALU_SEL_CSR;
        alu_sel_b_o = `ALU_SEL_CSR;
        alu_op_o = `ALU_OP_SEL_A;
        rd_sel_o = `RD_SEL_CSR;
        rd_we_o = 1'b1;
        csr_re_o = rd_o != 5'b0; // See spec.
        csr_we_o = 1'b1;        
      end

      10'b110_1110011: begin // CSRRSI
        alu_sel_a_o = `ALU_SEL_CSR;
        alu_sel_b_o = `ALU_SEL_CSR;
        alu_op_o = `ALU_OP_OR;
        rd_sel_o = `RD_SEL_CSR;
        rd_we_o = 1'b1;
        csr_re_o = 1'b1;        
        csr_we_o = rs1_o != 5'b0;        
      end

      10'b111_1110011: begin // CSRRCI
        alu_sel_a_o = `ALU_SEL_CSR;
        alu_sel_b_o = `ALU_SEL_CSR;
        alu_op_o = `ALU_OP_BIT_CLEAR;
        rd_sel_o = `RD_SEL_CSR;
        rd_we_o = 1'b1;
        csr_re_o = 1'b1;        
        csr_we_o = rs1_o != 5'b0;        
      end

      

      10'b001_0001111: begin // FENCE.I
        if ({inst_i[31:15], inst_i[11:7]} != 22'b0) begin
          invalid_op = 1'b1;
        end
        fencei_o = 1'b1;
      end

      10'b000_0001111: begin // FENCE
      end
      
      default: begin
        invalid_op = 1'b1;
      end
    endcase // case ({inst_i[14:12],inst_i[6:0]})    
  end // always_comb

  always_comb begin //sel_o
    case (io_len)
      LEN_1: begin
        sram_sel_o = `SRAM_SEL_1;        
      end
      LEN_2: begin
        sram_sel_o = `SRAM_SEL_2;        
      end
      LEN_4: begin
        sram_sel_o = `SRAM_SEL_4;        
      end 
      default:
        sram_sel_o = 2'b00;      
    endcase
  end // always_comb

  always_comb begin // imm
    case(imm_type)
      INST_R:
        imm_o = 32'd0;
      INST_I:
        imm_o = {{20{inst_i[31]}}, inst_i[31:20]};
      INST_S:
        imm_o = {{20{inst_i[31]}}, inst_i[31:25], inst_i[11:7]};
      INST_B:
        imm_o = {{19{inst_i[31]}}, inst_i[31], inst_i[7], inst_i[30:25], inst_i[11:8], 1'b0};
      INST_U:
        imm_o = {inst_i[31:12], 12'd0};
      INST_J:
        imm_o = {{11{inst_i[31]}}, inst_i[31], inst_i[19:12], inst_i[20], inst_i[30:21], 1'b0};      
      default:
        imm_o = 32'd0;        
    endcase // case (imm_type)    
  end
  
endmodule
