`include "defines.sv"

module instr_cache#(
    parameter INDEX_WIDTH = 5,
    parameter NROWS = 32,
    parameter TAG_WIDTH = 31 - INDEX_WIDTH // 27
)(
  input wire        clk,
  input wire        rst,

  input wire        cache_rst_i,

  input wire [31:0] addr_i,
  input wire        enable_i,
  
  output reg [31:0] data_o,
  output reg        wait_o,

  // interface of wishbone
  output reg        wb_cyc_o,
  output reg        wb_stb_o,
  input wire        wb_ack_i,
  output reg [31:0] wb_adr_o,
  output reg [31:0] wb_dat_o,
  input wire [31:0] wb_dat_i,
  output reg [3:0]  wb_sel_o,
  output reg        wb_we_o 
);
  enum logic[1:0] {
    ICACHE_INIT,
    ICACHE_MEM
  } icache_state;
  // 1 + TAG_WIDTH + 32
  logic [TAG_WIDTH+32:0]  cachelines[NROWS];
  logic [TAG_WIDTH-1:0]   addr_tag;
  logic [INDEX_WIDTH-1:0] addr_idx;
  logic                   icache_hit;
  logic [TAG_WIDTH+32:0]  selected_line;
  logic                   imem_wait;
  logic                   imem_read_enable;
  logic [31:0]            imem_data_o;

  assign addr_tag = addr_i[31:INDEX_WIDTH+1];
  assign addr_idx = addr_i[INDEX_WIDTH:1];
  assign selected_line = cachelines[addr_idx];
  assign icache_hit = (selected_line[TAG_WIDTH+32] &&
                       selected_line[TAG_WIDTH+31:32] == addr_tag
                       ) & ~cache_rst_i;

  always_ff @(posedge clk) begin
    if (rst | cache_rst_i) begin
      for (integer i = 0; i < NROWS; i = i + 1)
        cachelines[i][TAG_WIDTH+32] <= 1'b0;
    end
    if (rst) begin
      icache_state <= ICACHE_INIT;
    end else begin
      case(icache_state)
        ICACHE_INIT: begin
          if (enable_i) begin
            if (icache_hit)
              icache_state <= ICACHE_INIT;
            else
              icache_state <= ICACHE_MEM;
          end
        end
        ICACHE_MEM: begin
          if (~imem_wait) begin
            cachelines[addr_idx] <= {1'b1, addr_tag, imem_data_o};
            icache_state <= ICACHE_INIT;
          end
          // else remain in icache_state
        end
      endcase
    end
  end

  always_comb begin
    wait_o = 1'b1;
    imem_read_enable = 1'b0;
    data_o = 32'b0;
    case(icache_state)
      ICACHE_INIT: begin
        if (enable_i) begin
          if (icache_hit) begin
            data_o = selected_line[31:0];
            wait_o = 1'b0;
          end else begin
            wait_o = 1'b1;
          end
        end else begin
          wait_o = 1'b0;
        end
      end
      ICACHE_MEM: begin
        wait_o = imem_wait;
        imem_read_enable = 1'b1;
        data_o = imem_data_o;
      end
    endcase
  end
  
  mem_controller icache_mem (
    .clk(clk),
    .rst(rst),

    .addr_i(addr_i),
    .data_i(32'h0),
    .sel_i(4'b1111),
    .we_i(1'b0),
    .re_i(imem_read_enable),
    .data_o(imem_data_o),
    .mem_wait_o(imem_wait),

    .wb_cyc_o(wb_cyc_o),
    .wb_stb_o(wb_stb_o),
    .wb_ack_i(wb_ack_i),
    .wb_adr_o(wb_adr_o),
    .wb_dat_o(wb_dat_o),
    .wb_dat_i(wb_dat_i),
    .wb_sel_o(wb_sel_o),
    .wb_we_o(wb_we_o)
    );
endmodule