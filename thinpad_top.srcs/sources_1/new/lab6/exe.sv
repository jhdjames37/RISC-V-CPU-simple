`include "defines.sv"

module execution(
  input wire [31:0] pc_i,
  input wire [31:0] rs1_data_i,
  input wire [31:0] rs2_data_i,
  input wire [31:0] imm_i,
  input wire [4:0]  csr_imm_i, 
  input wire [31:0] csr_i,
  input wire [1:0]  alu_sel_a_i,
  input wire [1:0]  alu_sel_b_i,
  input wire [4:0]  alu_op_i,
  input wire        branch_i,
  input wire [2:0]  branch_type_i,
  input wire        branch_ja_i,

  
  output reg [31:0] alu_y_o,
  output reg [31:0] sram_data_o,
  output reg        branch_o
  );

  assign sram_data_o = rs2_data_i;


  always_comb begin
    branch_o = 1'b0;
    if (branch_i) begin
      if (branch_ja_i) begin
        branch_o = 1'b1;
      end else begin
        case(branch_type_i)
          `BRANCH_EQU: branch_o = (rs1_data_i == rs2_data_i);
          `BRANCH_NEQ: branch_o = (rs1_data_i != rs2_data_i);
          `BRANCH_LT: branch_o = ($signed(rs1_data_i) < $signed(rs2_data_i));
          `BRANCH_GE: branch_o = ~($signed(rs1_data_i) < $signed(rs2_data_i));
          `BRANCH_LTU: branch_o = (rs1_data_i < rs2_data_i);
          `BRANCH_GEU: branch_o = ~(rs1_data_i < rs2_data_i);
        endcase
      end
    end
  end

  alu_32 alu(
    .a(alu_sel_a_i == `ALU_SEL_RS1 ? rs1_data_i : (
       alu_sel_a_i == `ALU_SEL_CSR ? {27'b0, csr_imm_i} : pc_i )),
    .b(alu_sel_b_i == `ALU_SEL_RS2 ? rs2_data_i : (
                    alu_sel_b_i == `ALU_SEL_CSR ? csr_i : imm_i )),
    .y(alu_y_o),
    .op(alu_op_i)
    );
  

    
  
endmodule // execution

