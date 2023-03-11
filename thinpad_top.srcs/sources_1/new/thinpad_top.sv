`default_nettype none
`include "lab6/defines.sv"

module thinpad_top (
    input wire clk_50M,     // 50MHz 时钟输入
    input wire clk_11M0592, // 11.0592MHz 时钟输入（备用，可不用）

    input wire push_btn,  // BTN5 按钮开关，带消抖电路，按下时为 1
    input wire reset_btn, // BTN6 复位按钮，带消抖电路，按下时为 1

    input  wire [ 3:0] touch_btn,  // BTN1~BTN4，按钮开关，按下时为 1
    input  wire [31:0] dip_sw,     // 32 位拨码开关，拨到“ON”时为 1
    output wire [15:0] leds,       // 16 位 LED，输出时 1 点亮
    output wire [ 7:0] dpy0,       // 数码管低位信号，包括小数点，输出 1 点亮
    output wire [ 7:0] dpy1,       // 数码管高位信号，包括小数点，输出 1 点亮

    // CPLD 串口控制器信号
    output wire uart_rdn,        // 读串口信号，低有效
    output wire uart_wrn,        // 写串口信号，低有效
    input  wire uart_dataready,  // 串口数据准备好
    input  wire uart_tbre,       // 发送数据标志
    input  wire uart_tsre,       // 数据发送完毕标志

    // BaseRAM 信号
    inout wire [31:0] base_ram_data,  // BaseRAM 数据，低 8 位与 CPLD 串口控制器共享
    output wire [19:0] base_ram_addr,  // BaseRAM 地址
    output wire [3:0] base_ram_be_n,  // BaseRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire base_ram_ce_n,  // BaseRAM 片选，低有效
    output wire base_ram_oe_n,  // BaseRAM 读使能，低有效
    output wire base_ram_we_n,  // BaseRAM 写使能，低有效

    // ExtRAM 信号
    inout wire [31:0] ext_ram_data,  // ExtRAM 数据
    output wire [19:0] ext_ram_addr,  // ExtRAM 地址
    output wire [3:0] ext_ram_be_n,  // ExtRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire ext_ram_ce_n,  // ExtRAM 片选，低有效
    output wire ext_ram_oe_n,  // ExtRAM 读使能，低有效
    output wire ext_ram_we_n,  // ExtRAM 写使能，低有效

    // 直连串口信号
    output wire txd,  // 直连串口发送端
    input  wire rxd,  // 直连串口接收端

    // Flash 存储器信号，参考 JS28F640 芯片手册
    output wire [22:0] flash_a,  // Flash 地址，a0 仅在 8bit 模式有效，16bit 模式无意义
    inout wire [15:0] flash_d,  // Flash 数据
    output wire flash_rp_n,  // Flash 复位信号，低有效
    output wire flash_vpen,  // Flash 写保护信号，低电平时不能擦除、烧写
    output wire flash_ce_n,  // Flash 片选信号，低有效
    output wire flash_oe_n,  // Flash 读使能信号，低有效
    output wire flash_we_n,  // Flash 写使能信号，低有效
    output wire flash_byte_n, // Flash 8bit 模式选择，低有效。在使用 flash 的 16 位模式时请设为 1

    // USB 控制器信号，参考 SL811 芯片手册
    output wire sl811_a0,
    // inout  wire [7:0] sl811_d,     // USB 数据线与网络控制器的 dm9k_sd[7:0] 共享
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    // 网络控制器信号，参考 DM9000A 芯片手册
    output wire dm9k_cmd,
    inout wire [15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input wire dm9k_int,

    // 图像输出信号
    output wire [2:0] video_red,    // 红色像素，3 位
    output wire [2:0] video_green,  // 绿色像素，3 位
    output wire [1:0] video_blue,   // 蓝色像素，2 位
    output wire       video_hsync,  // 行同步（水平同步）信号
    output wire       video_vsync,  // 场同步（垂直同步）信号
    output wire       video_clk,    // 像素时钟输出
    output wire       video_de      // 行数据有效信号，用于区分消隐区
);

  /* =========== Demo code begin =========== */

  // PLL 分频示例
  logic locked, clk_10M, clk_20M, clk_high;
  pll_example clock_gen (
      // Clock in ports
      .clk_in1(clk_50M),  // 外部时钟输入
      // Clock out ports
      .clk_out1(clk_10M),  // 时钟输出 1，频率在 IP 配置界面中设置
      .clk_out2(clk_20M),  // 时钟输出 2，频率在 IP 配置界面中设置
      .clk_out3(clk_high),
      // Status and control signals
      .reset(reset_btn),  // PLL 复位输入
      .locked(locked)  // PLL 锁定指示输出，"1"表示时钟稳定，
                       // 后级电路复位信号应当由它生成（见下）
  );

  logic reset_of_clk10M;
  // 异步复位，同步释放，将 locked 信号转为后级电路的复位 reset_of_clk10M
  always_ff @(posedge clk_10M or negedge locked) begin
    if (~locked) reset_of_clk10M <= 1'b1;
    else reset_of_clk10M <= 1'b0;
  end
  
  logic reset_of_clk50M;
  // 异步复位，同步释放，将 locked 信号转为后级电路的复位 reset_of_clk10M
  always_ff @(posedge clk_50M or negedge locked) begin
    if (~locked) reset_of_clk50M <= 1'b1;
    else reset_of_clk50M <= 1'b0;
  end
  
  logic reset_of_clk_high;
  // 异步复位，同步释放，将 locked 信号转为后级电路的复位 reset_of_clk10M
  always_ff @(posedge clk_high or negedge locked) begin
    if (~locked) reset_of_clk_high <= 1'b1;
    else reset_of_clk_high <= 1'b0;
  end

  always_ff @(posedge clk_10M or posedge reset_of_clk10M) begin
    if (reset_of_clk10M) begin
      // Your Code
    end else begin
      // Your Code
    end
  end



  /* ====== instruction fetch stage ===== */
  logic clk, rst;
  
  assign clk = clk_high;
  assign rst = reset_of_clk_high;
  

  /* ======= IF1 ======== */
  
  logic immu_wb_cyc_o;
  logic immu_wb_stb_o;
  logic immu_wb_ack_i;
  logic [31:0] immu_wb_adr_o;
  logic [31:0] immu_wb_dat_o;
  logic [31:0] immu_wb_dat_i;
  logic [3:0]  immu_wb_sel_o;
  logic        immu_wb_we_o;
  

  logic [31:0] if1_pc_o;
  logic        if1_stall;  
  logic [1:0]  if1_interrupt_o;

  logic [1:0]  if_pc_sel;
  logic        if1_page_fault_o;

  logic [31:0] if1_ppc_o; // physical pc
  logic        if1_mem_wait;  
  logic        if1_branch_taken_o;
  
  
  inst_fetch if_stage(
    .clk(clk),
    .rst(rst),

    .branch_failed_i(exe_branch_failed_o),
    .branch_pc_i(exe_alu_y_o),
    .branch_i(if_pc_sel),
    .trap_pc_i(wb_pc_o),

    .exe_stall_i(exe_stall),
    .exe_branch_enable_i(exe_branch_reg),
    .exe_branch_taken_i(exe_branch_o),
    .exe_pc_i(exe_pc_reg),
    .exe_alu_y_i(exe_alu_y_o),
    
    .stall_i(if1_stall),
    .branch_taken_o(if1_branch_taken_o),
    .pc_o(if1_pc_o)
    );

  mmu if_mmu(
    .clk(clk),
    .rst(rst),

    .tlb_rst_i(if2_sfence),

    .addr_i(if1_pc_o),
    .mode_i(global_mode),
    .satp_i(global_satp),
    .sum_i(global_sum),
    .enable_i(1'b1),
    .permission_i(3'b100),

    .addr_o(if1_ppc_o),
    .wait_o(if1_mem_wait),
    .page_fault_o(if1_page_fault_o),

    .wb_cyc_o(immu_wb_cyc_o),
    .wb_stb_o(immu_wb_stb_o),
    .wb_ack_i(immu_wb_ack_i),
    .wb_adr_o(immu_wb_adr_o),
    .wb_dat_o(immu_wb_dat_o),
    .wb_dat_i(immu_wb_dat_i),
    .wb_sel_o(immu_wb_sel_o),
    .wb_we_o(immu_wb_we_o)
    );

  /* =========== IF2 ===================== */
  
  logic imem_wb_cyc_o;
  logic imem_wb_stb_o;
  logic imem_wb_ack_i;
  logic [31:0] imem_wb_adr_o;
  logic [31:0] imem_wb_dat_o;
  logic [31:0] imem_wb_dat_i;
  logic [3:0]  imem_wb_sel_o;
  logic        imem_wb_we_o;

  logic        imem_wait;
  logic        if2_stall;
  logic        if2_bubble;
  
  logic [31:0] if2_inst_tmp;
  logic [31:0] if2_inst_o;  
  logic [31:0] if2_pc_o;


  logic        if2_read_reg;
  logic [31:0] if2_ppc_reg;
  logic        if2_page_fault_o;
  logic        if2_misaligned_o;
  logic [31:0] if2_tval_o;
  logic [1:0]  if2_interrupt_o;
  logic        if2_sfence;  
  logic        if2_branch_taken_o;
  

  always_ff @ (posedge clk) begin
    if (rst) begin
      if2_read_reg <= 1'b0;
      if2_page_fault_o <= 1'b0;
      if2_interrupt_o <= 2'b0;      
      if2_branch_taken_o <= 1'b0;

      if2_pc_o <= 32'b0;
      if2_ppc_reg <= 32'b0;
      if2_tval_o <= 32'b0;      
    end else begin
      if (!if2_stall) begin
        if (if2_bubble) begin
          if2_read_reg <= 1'b0;
          if2_page_fault_o <= 1'b0;
          if2_interrupt_o <= 2'b0;          
          if2_branch_taken_o <= 1'b0;
        end else begin
          if2_read_reg <= ~if1_mem_wait & ~if1_page_fault_o;
          if2_page_fault_o <= if1_page_fault_o;
          if2_interrupt_o <= if1_interrupt_o;        
          if2_branch_taken_o <= if1_branch_taken_o;
        end
      
        if2_pc_o <= if1_pc_o;
        if2_ppc_reg <= if1_ppc_o;
        if2_tval_o <= if1_page_fault_o ? if1_pc_o : 32'h0;

      end
    end
  end
`define INST_NOP 32'h0000_0013
  assign if2_inst_o = if2_read_reg ? if2_inst_tmp : `INST_NOP;
  assign if2_sfence = {if2_inst_o[31:25], if2_inst_o[14:0]} == 22'b0001001_000_00000_1110011;
  assign if2_misaligned_o = if2_pc_o[1:0] != 2'b00;
  
  // imem
  instr_cache icache (
    .clk(clk),
    .rst(rst),

    .cache_rst_i(id_fencei_o),

    .addr_i(if2_ppc_reg),
    .enable_i(if2_read_reg),
    .data_o(if2_inst_tmp),
    .wait_o(imem_wait),

    .wb_cyc_o(imem_wb_cyc_o),
    .wb_stb_o(imem_wb_stb_o),
    .wb_ack_i(imem_wb_ack_i),
    .wb_adr_o(imem_wb_adr_o),
    .wb_dat_o(imem_wb_dat_o),
    .wb_dat_i(imem_wb_dat_i),
    .wb_sel_o(imem_wb_sel_o),
    .wb_we_o(imem_wb_we_o)
    );
  
  /* =========== Inst decode stage ======= */

  // flip-flop

  logic [31:0] id_inst_i;
  logic [31:0] id_pc_o;

  logic        id_stall;
  logic        id_bubble;  
  logic [1:0]  id_interrupt_o;
  logic        id_misaligned_o;  
  logic        id_page_fault_o;  
  logic [31:0] id_tval_o;
  logic        id_branch_taken_o;

  
  always_ff @ (posedge clk) begin
    if (rst) begin
      id_pc_o <= 32'h8000_0000;
      id_inst_i <= `INST_NOP; // nop
      id_interrupt_o <= 2'b0;
      id_page_fault_o <= 1'b0;
      id_misaligned_o <= 1'b0;      
      id_branch_taken_o <= 1'b0;

      id_pc_o <= 32'h0;
      id_tval_o <= 32'h0;
      
    end else begin
      if (!id_stall) begin
        if (id_bubble) begin
          id_inst_i <= `INST_NOP;
          id_interrupt_o <= 2'b0;
          id_page_fault_o <= 1'b0;
          id_misaligned_o <= 1'b0;          
          id_branch_taken_o <= 1'b0;
        end else begin
          id_inst_i <= if2_inst_o;
          id_interrupt_o <= if2_interrupt_o;
          id_page_fault_o <= if2_page_fault_o;
          id_misaligned_o <= if2_misaligned_o;          
          id_branch_taken_o <= if2_branch_taken_o;
        end

        id_pc_o <= if2_pc_o;
        if (if2_page_fault_o)
          id_tval_o <= if2_tval_o;
        else if (if2_misaligned_o)
          id_tval_o <= if2_pc_o;
        else
          id_tval_o <= if2_tval_o;
        
      end
    end
  end // always_ff @ (posedge clk)

  // main decoder

  logic [31:0] id_imm_o;
  logic [4:0]  id_rs1_o;
  logic [4:0]  id_rs2_o;
  logic [1:0]  id_alu_sel_a_o;
  logic [1:0]  id_alu_sel_b_o;  
  logic [4:0]  id_alu_op_o;  
  logic        id_sram_we_o;
  logic        id_sram_re_o;
  logic [1:0]  id_sram_sel_o;
  logic [1:0]  id_rd_sel_o;
  logic [4:0]  id_rd_o;
  logic        id_rd_we_o;
  logic        id_branch_o;  
  logic [2:0]  id_branch_type_o;
  logic        id_branch_ja_o;
  
  logic        id_invalid_inst_decode;
  logic        id_invalid_inst_csr;
  logic        id_invalid_inst_o;

  logic        id_fencei_o;

  logic        id_ebreak_o;
  logic        id_ecall_o;
  logic        id_mret_o;
  logic        id_sret_o;
  logic        id_sfence_o;

  logic [11:0] id_csr_addr_o;
  logic        id_csr_re_o;
  logic        id_csr_we_o;
  logic [31:0] id_csr_data_o;
  logic [4:0]  id_csr_imm_o;  
  logic        id_load_ext_type_o;
               
  assign id_invalid_inst_o = id_invalid_inst_decode | id_invalid_inst_csr;
  
  inst_decode id_stage(
    .inst_i(id_inst_i),
    .mode_i(global_mode),

    .imm_o(id_imm_o),

    .rs1_o(id_rs1_o),
    .rs2_o(id_rs2_o),
    
    .alu_sel_a_o(id_alu_sel_a_o),
    .alu_sel_b_o(id_alu_sel_b_o),
    .alu_op_o(id_alu_op_o),
    
    .sram_we_o(id_sram_we_o),
    .sram_re_o(id_sram_re_o),
    .sram_sel_o(id_sram_sel_o),

    .rd_sel_o(id_rd_sel_o),
    .rd_o(id_rd_o),
    .rd_we_o(id_rd_we_o),

    .branch_o(id_branch_o),
    .invalid_inst_o(id_invalid_inst_decode),

    .csr_addr_o(id_csr_addr_o),
    .csr_re_o(id_csr_re_o),
    .csr_we_o(id_csr_we_o),
    .csr_imm_o(id_csr_imm_o),

    .fencei_o(id_fencei_o),

    .ebreak_o(id_ebreak_o),
    .ecall_o(id_ecall_o),
    .mret_o(id_mret_o),
    .sret_o(id_sret_o),
    .sfence_o(id_sfence_o),
    
    .branch_type_o(id_branch_type_o),
    .branch_ja_o(id_branch_ja_o),
    .load_ext_type(id_load_ext_type_o)
    );

  // Reg File

  logic [31:0] id_rs1_data_o;
  logic [31:0] id_rs2_data_o;

  logic [31:0] id_rs1_side_o;
  logic [31:0] id_rs2_side_o;

  logic        id_rs1_side_sel_o;
  logic        id_rs2_side_sel_o;    
  
  reg_file_32 reg_file(
    .clk(clk),
    .reset(rst),
    .raddr_a(id_rs1_o),
    .raddr_b(id_rs2_o),

    .rdata_a(id_rs1_data_o),
    .rdata_b(id_rs2_data_o),

    .waddr(wb_rd_reg),
    .wdata(wb_rd_data_reg),
    .we(wb_rd_we_reg & ~wb_trap_disable)    
    );
  
  /* =========== Execution Stage =========== */

  // == flip-flop ==

  // == passby ==
  logic              exe_sram_we_o;
  logic              exe_sram_re_o;
  logic [1:0]        exe_sram_sel_o;
  logic [1:0]        exe_rd_sel_o;
  logic [4:0]        exe_rd_o;
  logic              exe_rd_we_o;

  logic              exe_bubble;
  logic              exe_stall;

  logic              exe_misaligned_o;  
  logic              exe_invalid_inst_o;
  logic              exe_page_fault_o;  
  logic [1:0]        exe_interrupt_o;

  logic              exe_sfence_o;  
  logic              exe_ebreak_o;
  logic              exe_ecall_o;
  logic              exe_mret_o;
  logic              exe_sret_o;
  
  logic [11:0]       exe_csr_addr_o;
  logic              exe_csr_we_o;
  logic [31:0]       exe_csr_data_o;


  logic [31:0]       exe_tval_o;  

  logic              exe_branch_taken_o;
  logic              exe_branch_failed_o;


  always_ff @ (posedge clk) begin
    if (rst) begin
      exe_sram_we_o <= 1'b0;
      exe_sram_re_o <= 1'b0;
      exe_rd_we_o <= 1'b0;

      exe_misaligned_o <= 1'b0;      
      exe_page_fault_o <= 1'b0;      
      exe_invalid_inst_o <= 1'b0;
      exe_interrupt_o <= 2'b0;

      exe_sfence_o <= 1'b0;      
      exe_ebreak_o <= 1'b0;
      exe_ecall_o <= 1'b0;
      exe_mret_o <= 1'b0;
      exe_sret_o <= 1'b0;
      exe_csr_we_o <= 1'b0;      

      exe_branch_taken_o <= 1'b0;

      
      exe_sram_sel_o <= 2'b0;      
      exe_rd_sel_o <= 2'b0;      
      exe_rd_o <= 5'b0;      
      
      exe_csr_addr_o <= 12'b0;
      exe_csr_data_o <= 32'b0;
      exe_tval_o <= 32'b0;

    end else begin
      if (!exe_stall) begin
        if (exe_bubble) begin          
          exe_sram_we_o <= 1'b0;
          exe_sram_re_o <= 1'b0;
          exe_rd_we_o <= 1'b0;

          exe_misaligned_o <= 1'b0;          
          exe_page_fault_o <= 1'b0;          
          exe_invalid_inst_o <= 1'b0;
          exe_interrupt_o <= 2'b0;

          exe_sfence_o <= 1'b0;          
          exe_ebreak_o <= 1'b0;
          exe_ecall_o <= 1'b0;
          exe_mret_o <= 1'b0;
          exe_sret_o <= 1'b0;
          exe_csr_we_o <= 1'b0;
          
          exe_branch_taken_o <= 1'b0;
        end else begin
          exe_sram_we_o <= id_sram_we_o;
          exe_sram_re_o <= id_sram_re_o;
          exe_rd_we_o <= id_rd_we_o;

          exe_misaligned_o <= id_misaligned_o;          
          exe_page_fault_o <= id_page_fault_o;          
          exe_invalid_inst_o <= id_invalid_inst_o;
          exe_interrupt_o <= id_interrupt_o;

          exe_sfence_o <= id_sfence_o;          
          exe_ebreak_o <= id_ebreak_o;
          exe_ecall_o <= id_ecall_o;
          exe_mret_o <= id_mret_o;
          exe_sret_o <= id_sret_o;
          exe_csr_we_o <= id_csr_we_o;

          exe_branch_taken_o <= id_branch_taken_o;
        end
        exe_sram_sel_o <= id_sram_sel_o;
        exe_rd_sel_o <= id_rd_sel_o;
        exe_rd_o <= id_rd_o;
        
        exe_csr_addr_o <= id_csr_addr_o;
        exe_csr_data_o <= id_csr_data_o;

        if (id_page_fault_o || id_misaligned_o)
          exe_tval_o <= id_tval_o;
        else if (id_invalid_inst_o)
          exe_tval_o <= id_inst_i;
        else
          exe_tval_o <= id_tval_o;        
      end
    end
  end // always_ff @ (posedge clk)

  // == consumed ==
  logic [31:0]     exe_rs1_data_reg;
  logic [31:0]     exe_rs2_data_reg;
  logic [31:0]     exe_pc_reg;
  logic [31:0]     exe_imm_reg;
  logic            exe_branch_reg;
  logic [2:0]      exe_branch_type_reg;
  logic            exe_branch_ja_reg;
  logic [1:0]      exe_alu_sel_a_reg;
  logic [1:0]      exe_alu_sel_b_reg;
  logic [4:0]      exe_alu_op_reg;
  logic [4:0]      exe_csr_imm_reg;  
  
  logic            exe_load_ext_type_o;

  always_ff @ (posedge clk) begin
    if (rst) begin
      exe_alu_op_reg <= 5'h0;         
      exe_branch_reg <= 1'b0;

      exe_rs1_data_reg <= 32'h0;      
      exe_rs2_data_reg <= 32'h0;      
      exe_pc_reg <= 32'h0;
      exe_imm_reg <= 32'h0;      
      exe_branch_type_reg <= 3'b0;      
      exe_branch_ja_reg <= 1'b0;      
      exe_alu_sel_a_reg <= 2'b0;      
      exe_alu_sel_b_reg <= 2'b0;      
      exe_csr_imm_reg <= 5'b0;      

      exe_load_ext_type_o <= 1'b0;
    end else begin
      // TODO: bubble and stall
      if (!exe_stall) begin
        if (exe_bubble) begin
          exe_alu_op_reg <= 5'h0;
          exe_branch_reg <= 1'b0;
          exe_load_ext_type_o <= 1'b0;
        end
        else begin
          exe_alu_op_reg <= id_alu_op_o;
          exe_branch_reg <= id_branch_o;
          exe_load_ext_type_o <= id_load_ext_type_o;
        end
        
        exe_rs1_data_reg <= id_rs1_side_sel_o ? id_rs1_side_o : id_rs1_data_o;
        exe_rs2_data_reg <= id_rs2_side_sel_o ? id_rs2_side_o : id_rs2_data_o;
        exe_pc_reg <= id_pc_o;
        exe_imm_reg <= id_imm_o;
        exe_branch_type_reg <= id_branch_type_o;
        exe_branch_ja_reg <= id_branch_ja_o;
        exe_alu_sel_a_reg <= id_alu_sel_a_o;          
        exe_alu_sel_b_reg <= id_alu_sel_b_o;
        exe_csr_imm_reg <= id_csr_imm_o;        
      end
    end
  end // always_ff @ (posedge clk)

  // main logic
  
  logic [31:0]       exe_alu_y_o;
  logic [31:0]       exe_sram_data_o;
  logic              exe_branch_o;  
  
  execution exe_stage(
    .pc_i(exe_pc_reg),
    .rs1_data_i(exe_rs1_data_reg),
    .rs2_data_i(exe_rs2_data_reg),
    .csr_i(exe_csr_data_o),
    .imm_i(exe_imm_reg),
    .csr_imm_i(exe_csr_imm_reg),
    
    .alu_sel_a_i(exe_alu_sel_a_reg),
    .alu_sel_b_i(exe_alu_sel_b_reg),
    .alu_op_i(exe_alu_op_reg),
    .branch_i(exe_branch_reg),
    .branch_type_i(exe_branch_type_reg),
    .branch_ja_i(exe_branch_ja_reg),

    .alu_y_o(exe_alu_y_o),
    .sram_data_o(exe_sram_data_o),
    .branch_o(exe_branch_o)
    );  

  always_comb begin
    exe_branch_failed_o = 1'b0;
    if (exe_branch_reg) begin
      if (exe_branch_o) begin
        exe_branch_failed_o = (id_pc_o != exe_alu_y_o);
      end
      else begin
        exe_branch_failed_o = (id_pc_o != exe_pc_reg + 4);
      end
    end
  end

  /* =========== Memory stage ============ */

  // === flip flop ===

  /* =========== MEM1 ====================*/
  
  logic [31:0]       mem1_pc_reg;
  logic [31:0]       mem1_alu_y_reg;
  logic [31:0]       mem1_sram_data_reg;
  logic              mem1_sram_re_reg;
  logic              mem1_sram_we_reg;
  logic [1:0]        mem1_sram_sel_reg;

  logic [4:0]        mem1_rd_o;
  logic [1:0]        mem1_rd_sel_o;
  logic              mem1_rd_we_o;

  logic              mem1_inst_misaligned_o;  
  logic              mem1_inst_page_fault_o;  
  logic              mem1_invalid_inst_o;
  logic [1:0]        mem1_interrupt_o;

  logic              mem1_stall;
  logic              mem1_bubble;

  logic              mem1_ebreak_o;
  logic              mem1_ecall_o;
  logic              mem1_mret_o;
  logic              mem1_sret_o;
  
  logic [11:0]       mem1_csr_addr_o;
  logic              mem1_csr_we_o;
  logic [31:0]       mem1_csr_data_o;

  logic [31:0]       mem1_tval_o;

  logic              mem1_load_ext_type_o;

  always_ff @ (posedge clk) begin
    if (rst) begin
      mem1_sram_re_reg <= 1'b0;
      mem1_sram_we_reg <= 1'b0;
      mem1_rd_we_o <= 1'b0;

      mem1_inst_misaligned_o <= 1'b0;      
      mem1_inst_page_fault_o <= 1'b0;      
      mem1_invalid_inst_o <= 1'b0;
      mem1_interrupt_o <= 2'b0;

      mem1_ebreak_o <= 1'b0;
      mem1_ecall_o <= 1'b0;
      mem1_mret_o <= 1'b0;
      mem1_sret_o <= 1'b0;
      mem1_csr_we_o <= 1'b0;

      mem1_pc_reg <= 32'h0;      
      mem1_alu_y_reg <= 32'h0;      
      mem1_sram_data_reg <= 32'h0;      
      mem1_sram_sel_reg <= 2'h0;      
      
      mem1_rd_o <= 5'b0;      
      mem1_rd_sel_o <= 2'b0;      
      
      mem1_csr_addr_o <= 12'h0;      
      mem1_csr_data_o <= 32'h0;      
      
      mem1_tval_o <= 32'h0;      

      mem1_load_ext_type_o <= 1'b0;
    end else begin
      if (!mem1_stall) begin
        if (mem1_bubble) begin
          mem1_sram_re_reg <= 1'b0;
          mem1_sram_we_reg <= 1'b0;
          mem1_rd_we_o <= 1'b0;

          mem1_inst_misaligned_o <= 1'b0;
          mem1_inst_page_fault_o <= 1'b0;      
          mem1_invalid_inst_o <= 1'b0;
          mem1_interrupt_o <= 2'b0;

          mem1_ebreak_o <= 1'b0;
          mem1_ecall_o <= 1'b0;
          mem1_mret_o <= 1'b0;
          mem1_sret_o <= 1'b0;
          mem1_csr_we_o <= 1'b0;      

          mem1_load_ext_type_o <= 1'b0;
        end else begin
          mem1_sram_re_reg <= exe_sram_re_o;
          mem1_sram_we_reg <= exe_sram_we_o;
          mem1_rd_we_o <= exe_rd_we_o;

          mem1_inst_misaligned_o <= exe_misaligned_o;          
          mem1_inst_page_fault_o <= exe_page_fault_o;          
          mem1_invalid_inst_o <= exe_invalid_inst_o;
          mem1_interrupt_o <= exe_interrupt_o;

          mem1_ebreak_o <= exe_ebreak_o;
          mem1_ecall_o <= exe_ecall_o;
          mem1_mret_o <= exe_mret_o;
          mem1_sret_o <= exe_sret_o;
          mem1_csr_we_o <= exe_csr_we_o;

          mem1_load_ext_type_o <= exe_load_ext_type_o;
        end

        mem1_pc_reg <= exe_pc_reg;
        mem1_alu_y_reg <= exe_alu_y_o;
        mem1_sram_data_reg <= exe_sram_data_o;
        mem1_sram_sel_reg <= exe_sram_sel_o;

        mem1_rd_o <= exe_rd_o;
        mem1_rd_sel_o <= exe_rd_sel_o;
        
        mem1_csr_addr_o <= exe_csr_addr_o;
        mem1_csr_data_o <= exe_csr_data_o;

        mem1_tval_o <= exe_tval_o;
      end
    end
  end // always_ff @ (posedge clk)

  logic mem1_trap_disable;
  assign mem1_trap_disable = mem1_interrupt_o != `INT_NONE | mem1_invalid_inst_o | mem1_inst_page_fault_o | mem1_inst_misaligned_o;

  logic [31:0] mem1_paddr_o;
  logic        mem1_wait;
  logic        mem1_mem_page_fault;
  logic        mem1_mem_page_fault_w_o;
  logic        mem1_mem_page_fault_r_o;
  
  logic dmmu_wb_cyc_o;
  logic dmmu_wb_stb_o;
  logic dmmu_wb_ack_i;
  logic [31:0] dmmu_wb_adr_o;
  logic [31:0] dmmu_wb_dat_o;
  logic [31:0] dmmu_wb_dat_i;
  logic [3:0]  dmmu_wb_sel_o;
  logic        dmmu_wb_we_o;

  mmu mem_mmu (
    .clk(clk),
    .rst(rst),

    .tlb_rst_i(exe_sfence_o), // TODO

    .addr_i(mem1_alu_y_reg),
    .mode_i(global_mode),
    .satp_i(global_satp),
    .sum_i(global_sum),
    .enable_i(~mem1_trap_disable & (mem1_sram_re_reg | mem1_sram_we_reg)),
    .permission_i({1'b0, mem1_sram_we_reg, mem1_sram_re_reg}),
    
    .addr_o(mem1_paddr_o),
    .wait_o(mem1_wait),
    .page_fault_o(mem1_mem_page_fault),
    
    .wb_cyc_o(dmmu_wb_cyc_o),
    .wb_stb_o(dmmu_wb_stb_o),
    .wb_ack_i(dmmu_wb_ack_i),
    .wb_adr_o(dmmu_wb_adr_o),
    .wb_dat_o(dmmu_wb_dat_o),
    .wb_dat_i(dmmu_wb_dat_i),
    .wb_sel_o(dmmu_wb_sel_o),
    .wb_we_o(dmmu_wb_we_o)
    );

  assign mem1_mem_page_fault_w_o = mem1_mem_page_fault & mem1_sram_we_reg;
  assign mem1_mem_page_fault_r_o = mem1_mem_page_fault & mem1_sram_re_reg;

  /* ========== MEM2 =========== */

  logic [31:0] mem2_pc_reg;
  logic [31:0] mem2_alu_y_reg;
  logic [31:0] mem2_paddr_reg;
  logic [31:0] mem2_sram_data_reg;
  logic        mem2_sram_re_reg;
  logic        mem2_sram_we_reg;
  logic [1:0]  mem2_sram_sel_reg;
  
  logic [4:0]  mem2_rd_o;
  logic [1:0]  mem2_rd_sel_o;
  logic        mem2_rd_we_o;

  logic        mem2_mem_misaligned_w_o;
  logic        mem2_mem_misaligned_r_o;  
  logic        mem2_mem_page_fault_w_o;
  logic        mem2_mem_page_fault_r_o;
  logic        mem2_inst_misaligned_o;  
  logic        mem2_inst_page_fault_o;  
  logic        mem2_invalid_inst_o;
  logic [1:0]  mem2_interrupt_o;
  
  logic        mem2_stall;
  logic        mem2_bubble;
  
  logic        mem2_ebreak_o;
  logic        mem2_ecall_o;
  logic        mem2_mret_o;
  logic        mem2_sret_o;
  
  logic [11:0] mem2_csr_addr_o;
  logic        mem2_csr_we_o;
  logic [31:0] mem2_csr_data_o;
  
  logic [31:0] mem2_tval_o;
  logic        mem2_load_ext_type_reg;
  

  always_ff @ (posedge clk) begin
    if (rst) begin
      mem2_sram_re_reg <= 1'b0;
      mem2_sram_we_reg <= 1'b0;
      mem2_rd_we_o <= 1'b0;


      mem2_mem_page_fault_w_o <= 1'b0;
      mem2_mem_page_fault_r_o <= 1'b0;
      mem2_inst_misaligned_o <= 1'b0;
      mem2_inst_page_fault_o <= 1'b0;      
      mem2_invalid_inst_o <= 1'b0;
      mem2_interrupt_o <= 2'b0;

      mem2_ebreak_o <= 1'b0;
      mem2_ecall_o <= 1'b0;
      mem2_mret_o <= 1'b0;
      mem2_sret_o <= 1'b0;
      mem2_csr_we_o <= 1'b0;

      mem2_pc_reg <= 32'h0;
      mem2_alu_y_reg <= 32'h0;      
      mem2_paddr_reg <= 32'h0;      
      mem2_sram_data_reg <= 32'h0;      
      mem2_sram_sel_reg <= 2'h0;      
      
      mem2_rd_o <= 5'h0;      
      mem2_rd_sel_o <= 2'h0;      
      
      mem2_csr_addr_o <= 12'h0;
      mem2_csr_data_o <= 32'h0;      
      
      mem2_tval_o <= 32'h0;      
      mem2_load_ext_type_reg <= 1'b0;
    end else begin
      if (!mem2_stall) begin
        if (mem2_bubble) begin
          mem2_sram_re_reg <= 1'b0;
          mem2_sram_we_reg <= 1'b0;
          mem2_rd_we_o <= 1'b0;

          mem2_mem_page_fault_w_o <= 1'b0;
          mem2_mem_page_fault_r_o <= 1'b0;
          mem2_inst_misaligned_o <= 1'b0;
          mem2_inst_page_fault_o <= 1'b0;      
          mem2_invalid_inst_o <= 1'b0;
          mem2_interrupt_o <= 2'b0;
          
          mem2_ebreak_o <= 1'b0;
          mem2_ecall_o <= 1'b0;
          mem2_mret_o <= 1'b0;
          mem2_sret_o <= 1'b0;
          mem2_csr_we_o <= 1'b0;
          mem2_load_ext_type_reg <= 1'b0;
        end else begin
          mem2_sram_re_reg <= mem1_sram_re_reg;
          mem2_sram_we_reg <= mem1_sram_we_reg;
          mem2_rd_we_o <= mem1_rd_we_o;

          mem2_inst_misaligned_o <= mem1_inst_misaligned_o;
          mem2_inst_page_fault_o <= mem1_inst_page_fault_o;
          mem2_mem_page_fault_w_o <= mem1_mem_page_fault_w_o;
          mem2_mem_page_fault_r_o <= mem1_mem_page_fault_r_o;          
          mem2_invalid_inst_o <= mem1_invalid_inst_o;
          mem2_interrupt_o <= mem1_interrupt_o;

          mem2_ebreak_o <= mem1_ebreak_o;
          mem2_ecall_o <= mem1_ecall_o;
          mem2_mret_o <= mem1_mret_o;
          mem2_sret_o <= mem1_sret_o;
          mem2_csr_we_o <= mem1_csr_we_o;
          mem2_load_ext_type_reg <= mem1_load_ext_type_o;
        end

        mem2_pc_reg <= mem1_pc_reg;
        mem2_alu_y_reg <= mem1_alu_y_reg;
        mem2_paddr_reg <= mem1_paddr_o;
        mem2_sram_data_reg <= mem1_sram_data_reg;
        mem2_sram_sel_reg <= mem1_sram_sel_reg;

        mem2_rd_o <= mem1_rd_o;
        mem2_rd_sel_o <= mem1_rd_sel_o;
        
        mem2_csr_addr_o <= mem1_csr_addr_o;
        mem2_csr_data_o <= mem1_csr_data_o;

        if (mem1_invalid_inst_o || mem1_inst_page_fault_o || mem1_inst_misaligned_o)
          mem2_tval_o <= mem1_tval_o;
        else if (mem1_mem_page_fault_w_o || mem1_mem_page_fault_r_o)
          mem2_tval_o <= mem1_alu_y_reg;
        else
          mem2_tval_o <= mem1_tval_o;        
      end
    end
  end // always_ff @ (posedge clk)

  logic mem2_trap_disable;

  assign mem2_trap_disable = mem2_interrupt_o != `INT_NONE | mem2_invalid_inst_o | 
                             mem2_inst_page_fault_o | mem2_mem_page_fault_w_o | 
                             mem2_mem_page_fault_r_o | mem2_inst_misaligned_o | 
                             mem2_mem_misaligned_w_o | mem2_mem_misaligned_r_o;
  

  // === byte len decode ===

  logic mem2_bad_addr;
  logic [3:0] mem2_dmem_sel;
  logic [31:0] mem2_data_w; // into DMEM
  logic [31:0] mem2_data_r; // from DMEM
  logic [31:0] mem2_data_o; // output
    
  mem_decoder mem_stage(
    .addr_i(mem2_paddr_reg[1:0]),
    .sel_i(mem2_sram_sel_reg),
    .load_ext_type_i(mem2_load_ext_type_reg),
    .data_w_i(mem2_sram_data_reg),
    .data_r_i(mem2_data_r),

    .data_w_o(mem2_data_w),
    .data_r_o(mem2_data_o),
    .sel_o(mem2_dmem_sel),
    .bad_addr(mem2_bad_addr)
    );

  assign mem2_mem_misaligned_w_o = mem2_bad_addr & mem2_sram_we_reg;
  assign mem2_mem_misaligned_r_o = mem2_bad_addr & mem2_sram_re_reg;

  // === DMEM ===


  logic        dmem_wait;  

  logic dmem_wb_cyc_o;
  logic dmem_wb_stb_o;
  logic dmem_wb_ack_i;
  logic [31:0] dmem_wb_adr_o;
  logic [31:0] dmem_wb_dat_o;
  logic [31:0] dmem_wb_dat_i;
  logic [3:0]  dmem_wb_sel_o;
  logic        dmem_wb_we_o;
  
  mem_controller dmem(
    .clk(clk),
    .rst(rst),

    .addr_i(mem2_paddr_reg),
    .data_i(mem2_data_w),
    .sel_i(mem2_dmem_sel),
    .we_i(mem2_sram_we_reg & ~mem2_trap_disable),
    .re_i(mem2_sram_re_reg & ~mem2_trap_disable),
    
    .data_o(mem2_data_r),
    .mem_wait_o(dmem_wait),

    .wb_cyc_o(dmem_wb_cyc_o),
    .wb_stb_o(dmem_wb_stb_o),
    .wb_ack_i(dmem_wb_ack_i),
    .wb_adr_o(dmem_wb_adr_o),
    .wb_dat_o(dmem_wb_dat_o),
    .wb_dat_i(dmem_wb_dat_i),
    .wb_sel_o(dmem_wb_sel_o),
    .wb_we_o(dmem_wb_we_o)
    );
  
  logic [31:0] mem2_rd_data_o;

  assign mem2_rd_data_o = mem2_rd_sel_o == `RD_SEL_ALU ? 
                          mem2_alu_y_reg : (
                            mem2_rd_sel_o == `RD_SEL_PC ?
                            mem2_pc_reg + 4 : (
                              mem2_rd_sel_o == `RD_SEL_CSR ?
                              mem2_csr_data_o : 
                              mem2_data_o
                            )                                                   
                          );
  

  /* =========== Write back stage ======== */

  logic [31:0] wb_pc_reg;
  logic        wb_stall;
  logic        wb_bubble;

  logic        wb_inst_misaligned_reg;  
  logic        wb_inst_page_fault_reg;
  logic        wb_mem_misaligned_r_reg;
  logic        wb_mem_misaligned_w_reg;
  logic        wb_mem_page_fault_w_reg;
  logic        wb_mem_page_fault_r_reg;  
  logic        wb_invalid_inst_reg;
  logic [1:0]  wb_interrupt_reg;

  logic        wb_ebreak_reg;
  logic        wb_ecall_reg;
  logic        wb_mret_reg;
  logic        wb_sret_reg;
  
  logic [11:0] wb_csr_addr_reg;
  logic        wb_csr_we_reg;
  logic [31:0] wb_alu_y_reg;

  logic [31:0] wb_tval_reg;

  logic [31:0] wb_rd_data_reg;
  logic [4:0]  wb_rd_reg;
  logic        wb_rd_we_reg;
  logic        wb_trap_disable;  


  always_ff @ (posedge clk) begin
    if (rst) begin
      wb_rd_we_reg <= 1'b0;

      wb_inst_misaligned_reg <= 1'b0;      
      wb_inst_page_fault_reg <= 1'b0;
      wb_mem_misaligned_w_reg <= 1'b0;
      wb_mem_misaligned_r_reg <= 1'b0;
      wb_mem_page_fault_w_reg <= 1'b0;
      wb_mem_page_fault_r_reg <= 1'b0;      
      wb_invalid_inst_reg <= 1'b0;
      wb_interrupt_reg <= 2'b0;

      wb_ebreak_reg <= 1'b0;
      wb_ecall_reg <= 1'b0;
      wb_mret_reg <= 1'b0;
      wb_sret_reg <= 1'b0;
      wb_csr_we_reg <= 1'b0;      

      wb_rd_reg <= 5'h0;      
      wb_pc_reg <= 32'h0;      
      wb_rd_data_reg <= 32'h0;      
      wb_csr_addr_reg <= 12'h0;      
      wb_alu_y_reg <= 32'h0;      
     
      wb_tval_reg <= 32'h0;      
    end else begin
      if (!wb_stall) begin
        if (wb_bubble) begin
          wb_rd_we_reg <= 1'b0;

          wb_inst_misaligned_reg <= 1'b0;      
          wb_inst_page_fault_reg <= 1'b0;
          wb_mem_misaligned_w_reg <= 1'b0;
          wb_mem_misaligned_r_reg <= 1'b0;
          wb_mem_page_fault_w_reg <= 1'b0;
          wb_mem_page_fault_r_reg <= 1'b0;          
          wb_invalid_inst_reg <= 1'b0;
          wb_interrupt_reg <= 2'b0;

          wb_ebreak_reg <= 1'b0;
          wb_ecall_reg <= 1'b0;
          wb_mret_reg <= 1'b0;
          wb_sret_reg <= 1'b0;
          wb_csr_we_reg <= 1'b0;      

        end else begin
          wb_rd_we_reg <= mem2_rd_we_o;

          wb_inst_misaligned_reg <= mem2_inst_misaligned_o;
          wb_inst_page_fault_reg <= mem2_inst_page_fault_o;
          wb_mem_misaligned_w_reg <= mem2_mem_misaligned_w_o;
          wb_mem_misaligned_r_reg <= mem2_mem_misaligned_r_o;
          wb_mem_page_fault_w_reg <= mem2_mem_page_fault_w_o;
          wb_mem_page_fault_r_reg <= mem2_mem_page_fault_r_o;          
          wb_invalid_inst_reg <= mem2_invalid_inst_o;
          wb_interrupt_reg <= mem2_interrupt_o;

          wb_ebreak_reg <= mem2_ebreak_o;
          wb_ecall_reg <= mem2_ecall_o;
          wb_mret_reg <= mem2_mret_o;
          wb_sret_reg <= mem2_sret_o;
          wb_csr_we_reg <= mem2_csr_we_o;      

        end
        wb_rd_reg <= mem2_rd_o;
        wb_pc_reg <= mem2_pc_reg;
        wb_rd_data_reg <= mem2_rd_data_o;
        wb_csr_addr_reg <= mem2_csr_addr_o;
        wb_alu_y_reg <= mem2_alu_y_reg;

        if (mem2_inst_misaligned_o || mem2_inst_page_fault_o ||  mem2_invalid_inst_o)
          wb_tval_reg <= mem2_tval_o;
        else if (mem2_mem_misaligned_w_o || mem2_mem_misaligned_r_o)
          wb_tval_reg <= mem2_alu_y_reg;
        else
          wb_tval_reg <= mem2_tval_o;        
      end
    end
  end // always_ff @ (posedge clk)

  assign wb_trap_disable = wb_interrupt_reg != `INT_NONE | wb_invalid_inst_reg | wb_inst_page_fault_reg | wb_mem_page_fault_w_reg | wb_mem_page_fault_r_reg | wb_mem_misaligned_r_reg | wb_mem_misaligned_w_reg | wb_inst_misaligned_reg;
  
  /* =========== Traps & CSR Module ====== */

  logic timeout_sig;
  logic wb_trap_o;
  logic [31:0] wb_pc_o;
  logic [1:0]  global_mode;
  logic [31:0] global_satp;
  logic [63:0] global_mtime;
  logic        global_sum;  
  
  
  
  csr_trap trap_m(
    .clk(clk),
    .rst(rst),

    .addr_r_i(id_csr_addr_o),
    .addr_w_i(wb_csr_addr_reg),
    .re_i(id_csr_re_o),
    .we_i(wb_csr_we_reg & ~wb_trap_disable),
    .data_w_i(wb_alu_y_reg),
    .data_r_o(id_csr_data_o),
    .invalid_inst_o(id_invalid_inst_csr),
    .mtime_i(global_mtime),

    .inst_page_fault_i(wb_inst_page_fault_reg),
    .inst_misaligned_i(wb_inst_misaligned_reg),
    .mem_page_fault_w_i(wb_mem_page_fault_w_reg),
    .mem_page_fault_r_i(wb_mem_page_fault_r_reg),
    .mem_misaligned_w_i(wb_mem_misaligned_w_reg),
    .mem_misaligned_r_i(wb_mem_misaligned_r_reg),
    .interrupt_i(wb_interrupt_reg),
    .invalid_inst_i(wb_invalid_inst_reg),
    .ecall_i(wb_ecall_reg),
    .ebreak_i(wb_ebreak_reg),
    .mret_i(wb_mret_reg),
    .sret_i(wb_sret_reg),
    .pc_i(wb_pc_reg),
    .val_i(wb_tval_reg),
    
    .if_stall(if1_stall),
    .pc_o(wb_pc_o),
    .trap_o(wb_trap_o),

    .mode_o(global_mode),
    .satp_o(global_satp),
    .sum_o(global_sum),
    
    .timeout_i(timeout_sig),
    .int_type(if1_interrupt_o)
    );  

  
  /* =========== Hazard Detector ========= */

  logic [31:0] hz_exe_data;
  logic        hz_exe_ready;  
  logic [31:0] hz_mem1_data;
  logic        hz_mem1_ready;
  logic [31:0] hz_mem2_data;
  logic        hz_mem2_ready;
  logic [31:0] hz_wb_data;
  logic        hz_wb_ready;

  always_comb begin
    hz_exe_data = 32'b0;
    hz_exe_ready = 1'b0;
    hz_mem1_data = 32'b0;
    hz_mem1_ready = 1'b0;
    
    hz_mem2_data = mem2_rd_data_o;
    hz_mem2_ready = 1'b1;
    hz_wb_data = wb_rd_data_reg;
    hz_wb_ready = 1'b1;

    case(exe_rd_sel_o)
      `RD_SEL_ALU: begin
        hz_exe_data = exe_alu_y_o;
        hz_exe_ready = 1'b1;        
      end
      `RD_SEL_PC: begin
        hz_exe_data = exe_pc_reg + 4;
        hz_exe_ready = 1'b1;        
      end
      `RD_SEL_CSR: begin
        hz_exe_data = exe_csr_data_o;
        hz_exe_ready = 1'b1;        
      end
    endcase // case (exe_rd_sel_o)

    case(mem1_rd_sel_o)
      `RD_SEL_ALU: begin
        hz_mem1_data = mem1_alu_y_reg;
        hz_mem1_ready = 1'b1;        
      end
      `RD_SEL_PC: begin
        hz_mem1_data = mem1_pc_reg + 4;
        hz_mem1_ready = 1'b1;        
      end
      `RD_SEL_CSR: begin
        hz_mem1_data = mem1_csr_data_o;
        hz_mem1_ready = 1'b1;        
      end
    endcase // case (mem1_rd_sel_o)
    
  end
  
  hazard_detector hazard(
    .immu_wait_i(if1_mem_wait | if2_sfence),
    .imem_wait_i(imem_wait),
    
    .id_rs1_i(id_rs1_o),
    .id_rs2_i(id_rs2_o),
    .id_fencei_i(id_fencei_o),

    .rs1_out_o(id_rs1_side_o),
    .rs2_out_o(id_rs2_side_o),
    .rs1_side_path_o(id_rs1_side_sel_o),
    .rs2_side_path_o(id_rs2_side_sel_o),
    
    .exe_rd_i(exe_rd_o),
    .exe_rd_we_i(exe_rd_we_o),
    .exe_rd_data_i(hz_exe_data),
    .exe_rd_ready_i(hz_exe_ready),
    .branch_i(exe_branch_o),
    .branch_failed(exe_branch_failed_o),

    .mem1_rd_i(mem1_rd_o),
    .mem1_rd_we_i(mem1_rd_we_o),
    .mem1_rd_data_i(hz_mem1_data),
    .mem1_rd_ready_i(hz_mem1_ready),

    .mem2_rd_i(mem2_rd_o),
    .mem2_rd_we_i(mem2_rd_we_o),
    .mem2_rd_data_i(hz_mem2_data),
    .mem2_rd_ready_i(hz_mem2_ready),

    .dmmu_wait_i(mem1_wait),
    .dmem_wait_i(dmem_wait),

    .wb_rd_i(wb_rd_reg),
    .wb_rd_we_i(wb_rd_we_reg),
    .wb_rd_data_i(hz_wb_data),
    .wb_rd_ready_i(hz_wb_ready),


    .csr_write_i(exe_csr_we_o | mem1_csr_we_o | mem2_csr_we_o | wb_csr_we_reg),
    .mem2_trap_i(mem2_trap_disable | mem2_mret_o | mem2_ecall_o | mem2_ebreak_o | mem2_sret_o),
    .wb_trap_i(wb_trap_o),

    .if1_stall_o(if1_stall),
    .if1_branch_o(if_pc_sel),
    .if2_stall_o(if2_stall),
    .if2_bubble_o(if2_bubble),
    .id_stall_o(id_stall),
    .id_bubble_o(id_bubble),
    .exe_stall_o(exe_stall),
    .exe_bubble_o(exe_bubble),
    .mem1_stall_o(mem1_stall),
    .mem1_bubble_o(mem1_bubble),
    .mem2_stall_o(mem2_stall),
    .mem2_bubble_o(mem2_bubble),
    .wb_stall_o(wb_stall),
    .wb_bubble_o(wb_bubble)
    );
  

  
  /* =========== Memory module ============ */

  /* =========== Arbiter ================== */

  logic wbm_cyc_o;
  logic wbm_stb_o;
  logic wbm_ack_i;
  logic [31:0] wbm_adr_o;
  logic [31:0] wbm_dat_o;
  logic [31:0] wbm_dat_i;
  logic [3:0]  wbm_sel_o;
  logic        wbm_we_o;

  wb_arbiter_4 arbiter (
    .clk,
    .rst,
  
    .wbm0_adr_i(dmem_wb_adr_o),
    .wbm0_dat_i(dmem_wb_dat_o),
    .wbm0_dat_o(dmem_wb_dat_i),
    .wbm0_we_i(dmem_wb_we_o),
    .wbm0_sel_i(dmem_wb_sel_o),
    .wbm0_stb_i(dmem_wb_stb_o),
    .wbm0_ack_o(dmem_wb_ack_i),
    .wbm0_err_o(),
    .wbm0_rty_o(),
    .wbm0_cyc_i(dmem_wb_cyc_o),

    .wbm1_adr_i(dmmu_wb_adr_o),
    .wbm1_dat_i(dmmu_wb_dat_o),
    .wbm1_dat_o(dmmu_wb_dat_i),
    .wbm1_we_i(dmmu_wb_we_o),
    .wbm1_sel_i(dmmu_wb_sel_o),
    .wbm1_stb_i(dmmu_wb_stb_o),
    .wbm1_ack_o(dmmu_wb_ack_i),
    .wbm1_err_o(),
    .wbm1_rty_o(),
    .wbm1_cyc_i(dmmu_wb_cyc_o),
    
    .wbm2_adr_i(imem_wb_adr_o),
    .wbm2_dat_i(imem_wb_dat_o),
    .wbm2_dat_o(imem_wb_dat_i),
    .wbm2_we_i(imem_wb_we_o),
    .wbm2_sel_i(imem_wb_sel_o),
    .wbm2_stb_i(imem_wb_stb_o),
    .wbm2_ack_o(imem_wb_ack_i),
    .wbm2_err_o(),
    .wbm2_rty_o(),
    .wbm2_cyc_i(imem_wb_cyc_o),
    
    .wbm3_adr_i(immu_wb_adr_o),
    .wbm3_dat_i(immu_wb_dat_o),
    .wbm3_dat_o(immu_wb_dat_i),
    .wbm3_we_i(immu_wb_we_o),
    .wbm3_sel_i(immu_wb_sel_o),
    .wbm3_stb_i(immu_wb_stb_o),
    .wbm3_ack_o(immu_wb_ack_i),
    .wbm3_err_o(),
    .wbm3_rty_o(),
    .wbm3_cyc_i(immu_wb_cyc_o),

    .wbs_adr_o(wbm_adr_o),
    .wbs_dat_o(wbm_dat_o),
    .wbs_dat_i(wbm_dat_i),
    .wbs_we_o(wbm_we_o),
    .wbs_sel_o(wbm_sel_o),
    .wbs_stb_o(wbm_stb_o),
    .wbs_ack_i(wbm_ack_i),
    .wbs_err_i('0),
    .wbs_rty_i('0),
    .wbs_cyc_o(wbm_cyc_o)
    
    );
  


  /* =========== Below are from Lab5 ========= */
  logic        sys_clk;
  logic        sys_rst;

  assign sys_clk = clk;
  assign sys_rst = rst;

  assign uart_rdn = 1'b1;
  assign uart_wrn = 1'b1;
  
  /* =========== MUX begin =========== */
  // Wishbone MUX (Masters) => bus slaves
  logic        wbs0_cyc_o;
  logic        wbs0_stb_o;
  logic        wbs0_ack_i;
  logic [31:0]       wbs0_adr_o;
  logic [31:0] wbs0_dat_o;
  logic [31:0] wbs0_dat_i;
  logic [3:0]  wbs0_sel_o;
  logic        wbs0_we_o;
  
  logic        wbs1_cyc_o;
  logic        wbs1_stb_o;
  logic        wbs1_ack_i;
  logic [31:0] wbs1_adr_o;
  logic [31:0] wbs1_dat_o;
  logic [31:0] wbs1_dat_i;
  logic [3:0]  wbs1_sel_o;
  logic        wbs1_we_o;
  
  logic        wbs2_cyc_o;
  logic        wbs2_stb_o;
  logic        wbs2_ack_i;
  logic [31:0] wbs2_adr_o;
  logic [31:0] wbs2_dat_o;
  logic [31:0] wbs2_dat_i;
  logic [3:0]  wbs2_sel_o;
  logic        wbs2_we_o;

  logic        wbs3_cyc_o;
  logic        wbs3_stb_o;
  logic        wbs3_ack_i;
  logic [31:0] wbs3_adr_o;
  logic [31:0] wbs3_dat_o;
  logic [31:0] wbs3_dat_i;
  logic [3:0]  wbs3_sel_o;
  logic        wbs3_we_o;
  
  wb_mux_4 wb_mux (
    .clk(sys_clk),
    .rst(sys_rst),
    
    // Master interface (to Lab5 master)
    .wbm_adr_i(wbm_adr_o),
    .wbm_dat_i(wbm_dat_o),
    .wbm_dat_o(wbm_dat_i),
    .wbm_we_i (wbm_we_o),
    .wbm_sel_i(wbm_sel_o),
    .wbm_stb_i(wbm_stb_o),
    .wbm_ack_o(wbm_ack_i),
    .wbm_err_o(),
    .wbm_rty_o(),
    .wbm_cyc_i(wbm_cyc_o),

    // Slave interface 0 (to BaseRAM controller)
    // Address range: 0x8000_0000 ~ 0x803F_FFFF
    .wbs0_addr    (32'h8000_0000),
    .wbs0_addr_msk(32'hFFC0_0000),

    .wbs0_adr_o(wbs0_adr_o),
    .wbs0_dat_i(wbs0_dat_i),
    .wbs0_dat_o(wbs0_dat_o),
    .wbs0_we_o (wbs0_we_o),
    .wbs0_sel_o(wbs0_sel_o),
    .wbs0_stb_o(wbs0_stb_o),
    .wbs0_ack_i(wbs0_ack_i),
    .wbs0_err_i('0),
    .wbs0_rty_i('0),
    .wbs0_cyc_o(wbs0_cyc_o),

    // Slave interface 1 (to ExtRAM controller)
    // Address range: 0x8040_0000 ~ 0x807F_FFFF
    .wbs1_addr    (32'h8040_0000),
    .wbs1_addr_msk(32'hFFC0_0000),

    .wbs1_adr_o(wbs1_adr_o),
    .wbs1_dat_i(wbs1_dat_i),
    .wbs1_dat_o(wbs1_dat_o),
    .wbs1_we_o (wbs1_we_o),
    .wbs1_sel_o(wbs1_sel_o),
    .wbs1_stb_o(wbs1_stb_o),
    .wbs1_ack_i(wbs1_ack_i),
    .wbs1_err_i('0),
    .wbs1_rty_i('0),
    .wbs1_cyc_o(wbs1_cyc_o),

    // Slave interface 2 (to UART controller)
    // Address range: 0x1000_0000 ~ 0x1000_FFFF
    .wbs2_addr    (32'h1000_0000),
    .wbs2_addr_msk(32'hFFFF_0000),

    .wbs2_adr_o(wbs2_adr_o),
    .wbs2_dat_i(wbs2_dat_i),
    .wbs2_dat_o(wbs2_dat_o),
    .wbs2_we_o (wbs2_we_o),
    .wbs2_sel_o(wbs2_sel_o),
    .wbs2_stb_o(wbs2_stb_o),
    .wbs2_ack_i(wbs2_ack_i),
    .wbs2_err_i('0),
    .wbs2_rty_i('0),
    .wbs2_cyc_o(wbs2_cyc_o),
    
    // Slave interface 3 (to mtime and mtimecmp)
    // Address range: 0x0200_0000 ~ 0x0200_FFFF
    .wbs3_addr    (32'h0200_0000),
    .wbs3_addr_msk(32'hFFFF_0000),

    .wbs3_adr_o(wbs3_adr_o),
    .wbs3_dat_i(wbs3_dat_i),
    .wbs3_dat_o(wbs3_dat_o),
    .wbs3_we_o (wbs3_we_o),
    .wbs3_sel_o(wbs3_sel_o),
    .wbs3_stb_o(wbs3_stb_o),
    .wbs3_ack_i(wbs3_ack_i),
    .wbs3_err_i('0),
    .wbs3_rty_i('0),
    .wbs3_cyc_o(wbs3_cyc_o)
    );

  /* =========== MUX end =========== */

  /* =========== Slaves begin =========== */
  sram_controller #(
    .SRAM_ADDR_WIDTH(20),
    .SRAM_DATA_WIDTH(32)
    ) sram_controller_base (
    .clk_i(sys_clk),
    .rst_i(sys_rst),

    // Wishbone slave (to MUX)
    .wb_cyc_i(wbs0_cyc_o),
    .wb_stb_i(wbs0_stb_o),
    .wb_ack_o(wbs0_ack_i),
    .wb_adr_i(wbs0_adr_o),
    .wb_dat_i(wbs0_dat_o),
    .wb_dat_o(wbs0_dat_i),
    .wb_sel_i(wbs0_sel_o),
    .wb_we_i (wbs0_we_o),

    // To SRAM chip
    .sram_addr(base_ram_addr),
    .sram_data(base_ram_data),
    .sram_ce_n(base_ram_ce_n),
    .sram_oe_n(base_ram_oe_n),
    .sram_we_n(base_ram_we_n),
    .sram_be_n(base_ram_be_n)
    );

  sram_controller #(
    .SRAM_ADDR_WIDTH(20),
    .SRAM_DATA_WIDTH(32)
    ) sram_controller_ext (
    .clk_i(sys_clk),
    .rst_i(sys_rst),

    // Wishbone slave (to MUX)
    .wb_cyc_i(wbs1_cyc_o),
    .wb_stb_i(wbs1_stb_o),
    .wb_ack_o(wbs1_ack_i),
    .wb_adr_i(wbs1_adr_o),
    .wb_dat_i(wbs1_dat_o),
    .wb_dat_o(wbs1_dat_i),
    .wb_sel_i(wbs1_sel_o),
    .wb_we_i (wbs1_we_o),

    // To SRAM chip
    .sram_addr(ext_ram_addr),
    .sram_data(ext_ram_data),
    .sram_ce_n(ext_ram_ce_n),
    .sram_oe_n(ext_ram_oe_n),
    .sram_we_n(ext_ram_we_n),
    .sram_be_n(ext_ram_be_n)
    );

  // 串口控制器模块
  // NOTE: 如果修改系统时钟频率，也需要修改此处的时钟频率参数
  uart_controller #(
    .CLK_FREQ(80_000_000),
    .BAUD    (115200)
    ) uart_controller (
    .clk_i(sys_clk),
    .rst_i(sys_rst),

    .wb_cyc_i(wbs2_cyc_o),
    .wb_stb_i(wbs2_stb_o),
    .wb_ack_o(wbs2_ack_i),
    .wb_adr_i(wbs2_adr_o),
    .wb_dat_i(wbs2_dat_o),
    .wb_dat_o(wbs2_dat_i),
    .wb_sel_i(wbs2_sel_o),
    .wb_we_i (wbs2_we_o),

    // to UART pins
    .uart_txd_o(txd),
    .uart_rxd_i(rxd)
    );

  mtime mtime (
    .clk_i(sys_clk),
    .rst_i(sys_rst),

    .wb_cyc_i(wbs3_cyc_o),
    .wb_stb_i(wbs3_stb_o),
    .wb_ack_o(wbs3_ack_i),
    .wb_adr_i(wbs3_adr_o),
    .wb_dat_i(wbs3_dat_o),
    .wb_dat_o(wbs3_dat_i),
    .wb_sel_i(wbs3_sel_o),
    .wb_we_i (wbs3_we_o),

    .timeout_o(timeout_sig),
    .mtime_o(global_mtime)
    );
  /* =========== Slaves end =========== */



endmodule
