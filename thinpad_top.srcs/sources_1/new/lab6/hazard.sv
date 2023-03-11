`include "defines.sv"

module hazard_detector (
  input wire        immu_wait_i,
  input wire        imem_wait_i,
  
  input wire [4:0]  id_rs1_i,
  input wire [4:0]  id_rs2_i,
  input wire        id_fencei_i,

  output reg [31:0] rs1_out_o,
  output reg        rs1_side_path_o,
  output reg [31:0] rs2_out_o,
  output reg        rs2_side_path_o, 
  
  input wire [4:0]  exe_rd_i,
  input wire        exe_rd_we_i,
  input wire [31:0] exe_rd_data_i,
  input wire        exe_rd_ready_i,
  input wire        branch_i,
  input wire        branch_failed,
  
  input wire [4:0]  mem1_rd_i,
  input wire        mem1_rd_we_i,
  input wire [31:0] mem1_rd_data_i,
  input wire        mem1_rd_ready_i, 
  input wire [4:0]  mem2_rd_i,
  input wire        mem2_rd_we_i,
  input wire [31:0] mem2_rd_data_i,
  input wire        mem2_rd_ready_i, 


  input wire        dmmu_wait_i,
  input wire        dmem_wait_i,
  
  input wire [4:0]  wb_rd_i,
  input wire        wb_rd_we_i,
  input wire [31:0] wb_rd_data_i,
  input wire        wb_rd_ready_i,

  input wire        csr_write_i,
  input wire        mem2_trap_i,
  input wire        wb_trap_i,

  output reg        if1_stall_o,
  output reg [1:0]  if1_branch_o,
  output reg        if2_stall_o,
  output reg        if2_bubble_o,
  output reg        id_stall_o,
  output reg        id_bubble_o,
  output reg        exe_stall_o,
  output reg        exe_bubble_o,
  output reg        mem1_stall_o,
  output reg        mem1_bubble_o,
  output reg        mem2_stall_o,
  output reg        mem2_bubble_o,
  output reg        wb_stall_o,
  output reg        wb_bubble_o

  );


  // RAW detection
  logic            read_after_write;
  logic            raw_rs1, raw_rs2;
  assign read_after_write = raw_rs1 | raw_rs2;
  
  always_comb begin
    raw_rs1 = 1'b0;
    rs1_out_o = 32'b0;
    rs1_side_path_o = 1'b0;    
    
    if (exe_rd_i == id_rs1_i && exe_rd_i != 5'h0 && exe_rd_we_i) begin
      if (exe_rd_ready_i) begin
        rs1_out_o = exe_rd_data_i;
        rs1_side_path_o = 1'b1;        
      end else 
        raw_rs1 = 1'b1;
    end else
    if (mem1_rd_i == id_rs1_i && mem1_rd_i != 5'h0 && mem1_rd_we_i) begin
      if (mem1_rd_ready_i) begin
        rs1_out_o = mem1_rd_data_i;
        rs1_side_path_o = 1'b1;        
      end else 
        raw_rs1 = 1'b1;
    end else
    if (mem2_rd_i == id_rs1_i && mem2_rd_i != 5'h0 && mem2_rd_we_i) begin
      if (mem2_rd_ready_i) begin
        rs1_out_o = mem2_rd_data_i;
        rs1_side_path_o = 1'b1;        
      end else 
        raw_rs1 = 1'b1;
    end else
    if (wb_rd_i == id_rs1_i && wb_rd_i != 5'h0 && wb_rd_we_i) begin
      if (wb_rd_ready_i) begin
        rs1_out_o = wb_rd_data_i;
        rs1_side_path_o = 1'b1;        
      end else
        raw_rs1 = 1'b1;
    end
  end // always_comb  

  always_comb begin
    raw_rs2 = 1'b0;
    rs2_out_o = 32'b0;
    rs2_side_path_o = 1'b0;    
    
    if (exe_rd_i == id_rs2_i && exe_rd_i != 5'h0 && exe_rd_we_i) begin
      if (exe_rd_ready_i) begin
        rs2_out_o = exe_rd_data_i;
        rs2_side_path_o = 1'b1;
      end else         
        raw_rs2 = 1'b1;
    end else
    if (mem1_rd_i == id_rs2_i && mem1_rd_i != 5'h0 && mem1_rd_we_i) begin
      if (mem1_rd_ready_i) begin
        rs2_out_o = mem1_rd_data_i;
        rs2_side_path_o = 1'b1;        
      end else         
        raw_rs2 = 1'b1;
    end else
    if (mem2_rd_i == id_rs2_i && mem2_rd_i != 5'h0 && mem2_rd_we_i) begin
      if (mem2_rd_ready_i) begin
        rs2_out_o = mem2_rd_data_i;
        rs2_side_path_o = 1'b1;        
      end else         
        raw_rs2 = 1'b1;
    end else
    if (wb_rd_i == id_rs2_i && wb_rd_i != 5'h0 && wb_rd_we_i) begin
      if (wb_rd_ready_i) begin
        rs2_out_o = wb_rd_data_i;
        rs2_side_path_o = 1'b1;        
      end else                 
        raw_rs2 = 1'b1;
    end
  end // always_comb


  //assign if_stall_o = read_after_write | imem_wait_i | dmem_wait_i;

  // ID + EXE + MEM
  always_comb begin
    if1_stall_o = 1'b0;
    if1_branch_o = `PC_ADD4;    

    if2_stall_o = 1'b0;
    if2_bubble_o = 1'b0;
    
    id_stall_o = 1'b0;
    id_bubble_o = 1'b0;

    exe_stall_o = 1'b0;
    exe_bubble_o = 1'b0;

    mem1_stall_o = 1'b0;
    mem1_bubble_o = 1'b0;
    
    mem2_stall_o = 1'b0;
    mem2_bubble_o = 1'b0;

    wb_stall_o = 1'b0;
    wb_bubble_o = 1'b0;    

    if (dmmu_wait_i && wb_trap_i) begin
      if1_stall_o = 1'b1;
      if2_stall_o = 1'b1;
      id_stall_o = 1'b1;
      exe_stall_o = 1'b1;
      mem1_stall_o = 1'b1;
      mem2_bubble_o = 1'b1;
      wb_stall_o = 1'b1;
    end else if (imem_wait_i && wb_trap_i) begin
      if1_stall_o = 1'b1;
      if2_stall_o = 1'b1;
      id_bubble_o = 1'b1;
      exe_bubble_o = 1'b1;
      mem1_bubble_o = 1'b1;
      mem2_bubble_o = 1'b1;
      wb_stall_o = 1'b1;      
    end else if (immu_wait_i && wb_trap_i) begin
      if1_stall_o = 1'b1;
      if2_bubble_o = 1'b1;
      id_bubble_o = 1'b1;
      exe_bubble_o = 1'b1;
      mem1_bubble_o = 1'b1;
      mem2_bubble_o = 1'b1;
      wb_stall_o = 1'b1;      
    end else if (wb_trap_i) begin
      if1_branch_o = `PC_TRAP;
      if2_bubble_o = 1'b1;      
      id_bubble_o = 1'b1;
      exe_bubble_o = 1'b1;
      mem1_bubble_o = 1'b1;
      mem2_bubble_o = 1'b1;
      wb_bubble_o = 1'b1;      
    end else if (dmem_wait_i) begin
      if1_stall_o = 1'b1;
      if2_stall_o = 1'b1;
      id_stall_o = 1'b1;
      exe_stall_o = 1'b1;
      mem1_stall_o = 1'b1;
      mem2_stall_o = 1'b1;
      wb_bubble_o = 1'b1;      
    end else if (mem2_trap_i) begin
      if1_stall_o = 1'b1;
      if2_stall_o = 1'b1;
      id_bubble_o = 1'b1;
      exe_bubble_o = 1'b1;
      mem1_bubble_o = 1'b1;
      mem2_bubble_o = 1'b1;
    end else if (dmmu_wait_i) begin
      if1_stall_o = 1'b1;
      if2_stall_o = 1'b1;
      id_stall_o = 1'b1;
      exe_stall_o = 1'b1;
      mem1_stall_o = 1'b1;
      mem2_bubble_o = 1'b1;
    end else if (imem_wait_i && branch_failed) begin
      if1_stall_o = 1'b1;
      if2_stall_o = 1'b1;      
      id_stall_o = 1'b1;
      exe_stall_o = 1'b1;
      mem1_bubble_o = 1'b1;
    end else if (immu_wait_i && branch_failed) begin
      if1_stall_o = 1'b1;
      if2_bubble_o = 1'b1;      
      id_bubble_o = 1'b1;
      exe_stall_o = 1'b1;
      mem1_bubble_o = 1'b1;
    end else if (branch_failed) begin
      if1_branch_o = branch_i ? `PC_BRANCH : `PC_ADD4;
      if2_bubble_o = 1'b1;      
      id_bubble_o = 1'b1;
      exe_bubble_o = 1'b1;      
    end else if (read_after_write) begin
      if1_stall_o = 1'b1;
      if2_stall_o = 1'b1;
      id_stall_o = 1'b1;
      exe_bubble_o = 1'b1;      
    end else if (imem_wait_i | csr_write_i | id_fencei_i) begin
      if1_stall_o = 1'b1;
      if2_stall_o = 1'b1;      
      id_bubble_o = 1'b1;
    end else if (immu_wait_i) begin
      if1_stall_o = 1'b1;
      if2_bubble_o = 1'b1;      
    end
  end // always_comb

  //assign wb_stall_o = 1'b0;
  //assign wb_bubble_o = dmem_wait_i;
  

  
endmodule // hazard_detector

  
