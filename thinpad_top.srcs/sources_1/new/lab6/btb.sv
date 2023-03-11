`include "defines.sv"

`define VALID 66
`define STATE 65:64
`define TAG 63:32
`define TGT 31:0

module branch_predictor(
  input wire        clk,
  input wire        rst,
  // BTB predict
  input wire [31:0] old_pc_i,
  output reg        branch_taken_o,
  output reg [31:0] pred_target_o,
  // BTB refresh
  input wire        exe_stall_i,
  input wire        exe_branch_enable_i,
  input wire        exe_branch_taken_i,
  input wire [31:0] exe_pc_i,
  input wire [31:0] exe_alu_y_i
);
  // TAG_WIDTH=32, VALID_BIT=1, STATUS_BITS=2, TARGET=32
  logic [66:0]  buffer[8];
  logic [2:0]   rank[8]; // rank[i] == 0 is the most recent used
  logic [2:0]   old_pc_idx;
  logic         old_pc_match;
  logic [2:0]   exe_pc_idx;
  logic         exe_pc_match;
  logic [2:0]   target_rank;
  logic [2:0]   evict;

  always_comb begin
    if (buffer[0][`TAG] == old_pc_i && buffer[0][`VALID]) {old_pc_idx, old_pc_match} = {3'd0, 1'b1};
    else if (buffer[1][`TAG] == old_pc_i && buffer[1][`VALID]) {old_pc_idx, old_pc_match} = {3'd1, 1'b1};
    else if (buffer[2][`TAG] == old_pc_i && buffer[2][`VALID]) {old_pc_idx, old_pc_match} = {3'd2, 1'b1};
    else if (buffer[3][`TAG] == old_pc_i && buffer[3][`VALID]) {old_pc_idx, old_pc_match} = {3'd3, 1'b1};
    else if (buffer[4][`TAG] == old_pc_i && buffer[4][`VALID]) {old_pc_idx, old_pc_match} = {3'd4, 1'b1};
    else if (buffer[5][`TAG] == old_pc_i && buffer[5][`VALID]) {old_pc_idx, old_pc_match} = {3'd5, 1'b1};
    else if (buffer[6][`TAG] == old_pc_i && buffer[6][`VALID]) {old_pc_idx, old_pc_match} = {3'd6, 1'b1};
    else if (buffer[7][`TAG] == old_pc_i && buffer[7][`VALID]) {old_pc_idx, old_pc_match} = {3'd7, 1'b1};
    else {old_pc_idx, old_pc_match} = 4'b0;
    
    if (buffer[0][`TAG] == exe_pc_i && buffer[0][`VALID]) {exe_pc_idx, exe_pc_match} = {3'd0, 1'b1};
    else if (buffer[1][`TAG] == exe_pc_i && buffer[1][`VALID]) {exe_pc_idx, exe_pc_match} = {3'd1, 1'b1};
    else if (buffer[2][`TAG] == exe_pc_i && buffer[2][`VALID]) {exe_pc_idx, exe_pc_match} = {3'd2, 1'b1};
    else if (buffer[3][`TAG] == exe_pc_i && buffer[3][`VALID]) {exe_pc_idx, exe_pc_match} = {3'd3, 1'b1};
    else if (buffer[4][`TAG] == exe_pc_i && buffer[4][`VALID]) {exe_pc_idx, exe_pc_match} = {3'd4, 1'b1};
    else if (buffer[5][`TAG] == exe_pc_i && buffer[5][`VALID]) {exe_pc_idx, exe_pc_match} = {3'd5, 1'b1};
    else if (buffer[6][`TAG] == exe_pc_i && buffer[6][`VALID]) {exe_pc_idx, exe_pc_match} = {3'd6, 1'b1};
    else if (buffer[7][`TAG] == exe_pc_i && buffer[7][`VALID]) {exe_pc_idx, exe_pc_match} = {3'd7, 1'b1};
    else {exe_pc_idx, exe_pc_match} = 4'b0;

    if (rank[0] == 3'b111) evict = 4'd0;
    else if (rank[1] == 3'b111) evict = 4'd1;
    else if (rank[2] == 3'b111) evict = 4'd2;
    else if (rank[3] == 3'b111) evict = 4'd3;
    else if (rank[4] == 3'b111) evict = 4'd4;
    else if (rank[5] == 3'b111) evict = 4'd5;
    else if (rank[6] == 3'b111) evict = 4'd6;
    else if (rank[7] == 3'b111) evict = 4'd7;
    else evict = 4'd0;
  end

  assign branch_taken_o = old_pc_match;
  assign pred_target_o = old_pc_match ? buffer[old_pc_idx][`TGT] : old_pc_i + 4;

  assign target_rank = rank[exe_pc_idx];

  always_ff @(posedge clk) begin
    if (rst) begin
      rank[0] <= 3'd0;
      rank[1] <= 3'd1;
      rank[2] <= 3'd2;
      rank[3] <= 3'd3;
      rank[4] <= 3'd4;
      rank[5] <= 3'd5;
      rank[6] <= 3'd6;
      rank[7] <= 3'd7;
      for (integer i = 0; i < 8; i += 1)
        buffer[i][66] <= 1'b0;
    end else if (exe_branch_enable_i) begin
      // update only for branch instrs
      if (exe_pc_match) begin
        if (target_rank != 3'b0) begin // update rank
          if (rank[0] < target_rank) rank[0] <= rank[0] + 1;
          if (rank[1] < target_rank) rank[1] <= rank[1] + 1;
          if (rank[2] < target_rank) rank[2] <= rank[2] + 1;
          if (rank[3] < target_rank) rank[3] <= rank[3] + 1;
          if (rank[4] < target_rank) rank[4] <= rank[4] + 1;
          if (rank[5] < target_rank) rank[5] <= rank[5] + 1;
          if (rank[6] < target_rank) rank[6] <= rank[6] + 1;
          if (rank[7] < target_rank) rank[7] <= rank[7] + 1;
          rank[exe_pc_idx] <= 3'b0;
        end
        // update bimodal
        if (exe_branch_taken_i) begin // goto taken, 1
          if (buffer[old_pc_idx][65:64] != 2'b11)
            buffer[old_pc_idx][65:64] <= buffer[old_pc_idx][65:64] + 1;
          buffer[old_pc_idx][`TGT] <= exe_alu_y_i;
        end else begin // goto nottaken, 0
          if (buffer[old_pc_idx][65:64] != 2'b00)
            buffer[old_pc_idx][65:64] <= buffer[old_pc_idx][65:64] - 1;
        end
      end else begin // replace
        rank[0] <= rank[0] + 1;
        rank[1] <= rank[1] + 1;
        rank[2] <= rank[2] + 1;
        rank[3] <= rank[3] + 1;
        rank[4] <= rank[4] + 1;
        rank[5] <= rank[5] + 1;
        rank[6] <= rank[6] + 1;
        rank[7] <= rank[7] + 1;
        rank[evict] <= 4'b0;
        buffer[evict] <= {1'b1, exe_branch_taken_i, ~exe_branch_taken_i, exe_pc_i, exe_alu_y_i};
      end
    end
  end

endmodule