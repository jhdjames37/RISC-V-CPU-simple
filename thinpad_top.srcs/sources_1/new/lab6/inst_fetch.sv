`include "defines.sv"

module inst_fetch(
  input wire          clk,
  input wire          rst,
  
  input wire          branch_failed_i,
  input wire [31:0]   branch_pc_i,
  input wire [31:0]   trap_pc_i,
  input wire [1:0]    branch_i,
  
  input wire          exe_stall_i,
  input wire          exe_branch_enable_i,
  input wire          exe_branch_taken_i,
  input wire [31:0]   exe_pc_i,
  input wire [31:0]   exe_alu_y_i,

  input wire          stall_i,  

  output logic        branch_taken_o,
  output logic [31:0] pc_o


  );

  reg   [31:0]        pc_reg;
  logic [31:0]        pc_predict;


  branch_predictor btb(
    .clk(clk),
    .rst(rst),

    .old_pc_i(pc_reg),
    .branch_taken_o(branch_taken_o),
    .pred_target_o(pc_predict),

    .exe_stall_i(exe_stall_i),
    .exe_branch_enable_i(exe_branch_enable_i),
    .exe_branch_taken_i(exe_branch_taken_i && (branch_i != `PC_TRAP)),
    .exe_pc_i(exe_pc_i),
    .exe_alu_y_i(exe_alu_y_i)
  );
 
  //assign pc_predict = pc_reg + 4;
  //assign branch_taken_o = 1'b0;
  
  // pc mux + pc reg
  always_ff @ (posedge clk) begin
    if (rst) begin
      pc_reg <= 32'h8000_0000;      
    end else begin
      if (!stall_i) begin
        pc_reg <= branch_i == `PC_TRAP ? trap_pc_i :
                  branch_failed_i ? (
                    branch_i == `PC_BRANCH ? branch_pc_i : exe_pc_i + 4
                  ) : pc_predict;
      end
    end
  end

  assign pc_o = pc_reg;
  
endmodule // inst_fetch

