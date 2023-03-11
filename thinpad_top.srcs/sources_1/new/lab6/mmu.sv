// Virtual Page Tranlation Module

`include "defines.sv"

module mmu(
  input wire        clk,
  input wire        rst,

  input wire        tlb_rst_i,

  input wire [31:0] addr_i,
  input wire [1:0]  mode_i,
  input wire [31:0] satp_i,
  input wire        enable_i,
  input wire [2:0]  permission_i, // XWR
  input wire        sum_i, // sum field in mstatus
  
  output reg [31:0] addr_o,
  output reg        wait_o,
  output reg        page_fault_o,

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

  enum logic [2:0] {
       MMU_INIT,
       MMU_LV1,
       MMU_LV1_DONE,
       MMU_LV2,
       MMU_DONE
       } state;

  logic tlb_valid;
  logic [19:0] tlb_vp;
  logic [31:0] tlb_pe;

  logic        tlb_found;
  logic        mode_direct;        


  assign mode_direct = (mode_i == `MODE_M | satp_i[31] == 1'b0); 
  
  assign tlb_found = (!mode_direct & tlb_valid & tlb_vp == addr_i[31:12]);


  logic [19:0] page_table_addr;
  logic        mem_wait;
  logic [31:0] page_table_data;
  logic        request;
  logic [31:0] lv1_addr;
  logic [9:0] offset;  
  
  always_ff @ (posedge clk) begin
    if (rst) begin
      state <= MMU_INIT;
      lv1_addr <= 32'h0;      
    end else begin
      case(state)
        MMU_INIT: begin
          if (enable_i) begin
            if (tlb_found || mode_direct) begin
              state <= MMU_INIT;            
            end else begin
              state <= MMU_LV1;
            end
          end
        end
        MMU_LV1: begin
          if (!mem_wait) begin
            state <= MMU_LV1_DONE;
            lv1_addr <= page_table_data;
          end
        end  
        MMU_LV1_DONE: begin
          if (page_fault_o) begin
            state <= MMU_INIT;
          end else begin
            state <= MMU_LV2;
          end
        end
        MMU_LV2: begin
          if (!mem_wait) begin
            state <= MMU_DONE;
          end
        end
        MMU_DONE: begin
          state <= MMU_INIT;          
        end
        default: begin
          state <= MMU_INIT;          
        end
      endcase
    end
  end // always_ff @ (posedge clk)

  logic [19:0] ppage;
  
  always_comb begin
    ppage = 20'h0;
    page_fault_o = 1'b0;
    request = 1'b0;
    page_table_addr = 20'b0;
    wait_o = 1'b1;    
    offset = 10'b0;
    case(state)
      MMU_INIT: begin
        if (enable_i) begin
          if (mode_direct) begin
            ppage = addr_i[31:12];
            wait_o = 1'b0;          
          end else if (tlb_found) begin
            if ((mode_i == `MODE_U && !tlb_pe[4]) || 
                (permission_i & ~tlb_pe[3:1]) != 3'b0 ||
                (mode_i == `MODE_S && tlb_pe[4] && ( !sum_i || permission_i[2] ) )) begin
              page_fault_o = 1'b1;
            end
            ppage = tlb_pe[29:10];
            wait_o = 1'b0;
          end
          // TODO: may reduce cycle?
          /* else begin
            request = 1'b1;
            page_table_addr = satp[19:0];
            offset = addr_i[31:22];   
          end
           */
        end else begin // if (enable_i)
          wait_o = 1'b0;
        end
      end // case: MMU_INIT
      MMU_LV1: begin
        request = 1'b1;
        page_table_addr = satp_i[19:0];        
        offset = addr_i[31:22];
      end
      MMU_LV1_DONE: begin
        // !V || leaf_page || !U
        if (lv1_addr[0] == 1'b0 || lv1_addr[3:1] != 3'b0 || (mode_i == `MODE_U && !lv1_addr[4])) begin
          page_fault_o = 1'b1;
          wait_o = 1'b0;          
        end                                
      end
      MMU_LV2: begin
        request = 1'b1;
        page_table_addr = lv1_addr[29:10];
        offset = addr_i[21:12];        
        if (!mem_wait) begin
          wait_o = 1'b0;            
          // !V || !R&W || !U || permission || S&(!SUM | exec)
          if (page_table_data[0] == 1'b0 || 
              page_table_data[2:1] == 2'b10 || 
              (mode_i == `MODE_U && !page_table_data[4]) || 
              (permission_i & (~page_table_data[3:1])) != 3'b0 || 
              (mode_i == `MODE_S && page_table_data[4] && ( !sum_i || permission_i[2] ) ) ) begin
            page_fault_o = 1'b1;
          end else begin
            ppage = page_table_data[29:10];
          end
        end
      end // case: MMU_LV2
      MMU_DONE: begin
        wait_o = enable_i;
      end
    endcase
  end // always_comb

  always_ff @ (posedge clk) begin
    if (rst || tlb_rst_i)  begin
      tlb_valid <= 1'b0;      
    end else begin
      if (state == MMU_LV2 && !mem_wait && !page_fault_o) begin
        tlb_valid <= 1'b1;
        tlb_vp <= addr_i[31:12];
        tlb_pe <= page_table_data;        
      end
    end
  end // always_ff @ (posedge clk)
  
  assign addr_o = {ppage, addr_i[11:0]};  
  

  mem_controller mem (
    .clk(clk),
    .rst(rst),
    
    .addr_i({page_table_addr, offset, 2'b0}),
    .data_i(32'h0),
    .sel_i(4'b1111),
    .we_i(1'b0),
    .re_i(request),
    .data_o(page_table_data),
    .mem_wait_o(mem_wait),

    .wb_cyc_o(wb_cyc_o),
    .wb_stb_o(wb_stb_o),
    .wb_ack_i(wb_ack_i),
    .wb_adr_o(wb_adr_o),
    .wb_dat_o(wb_dat_o),
    .wb_dat_i(wb_dat_i),
    .wb_sel_o(wb_sel_o),
    .wb_we_o(wb_we_o)
    );

  
  

  
endmodule // mmu

