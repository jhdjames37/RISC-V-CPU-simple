// mtime and mtimecmp MMIO register

module mtime (
  // clk and reset
  input wire        clk_i,
  input wire        rst_i,

    // wishbone slave interface
  input wire        wb_cyc_i,
  input wire        wb_stb_i,
  output reg        wb_ack_o,
  input wire [31:0] wb_adr_i,
  input wire [31:0] wb_dat_i,
  output reg [31:0] wb_dat_o,
  input wire [3:0]  wb_sel_i,
  input wire        wb_we_i,

  output reg        timeout_o,
  output reg [63:0] mtime_o
  );

  logic [63:0]      mtime;
  logic [63:0]      mtimecmp;

  assign mtime_o = mtime;  

  always_ff @ (posedge clk_i) begin
    if (rst_i) begin
      mtime <= 64'h0;
      mtimecmp <= 64'hffff_ffff_ffff_ffff;      
    end else begin      
      if (wb_cyc_i && wb_stb_i && wb_we_i) begin
        case (wb_adr_i)
          32'h0200_BFF8: begin // mtime
            if (wb_sel_i[0]) mtime[7:0] <= wb_dat_i[7:0];
            if (wb_sel_i[1]) mtime[15:8] <= wb_dat_i[15:8];
            if (wb_sel_i[2]) mtime[23:16] <= wb_dat_i[23:16];
            if (wb_sel_i[3]) mtime[31:24] <= wb_dat_i[31:24];
          end
          32'h0200_BFFC: begin // mtime+4
            if (wb_sel_i[0]) mtime[39:32] <= wb_dat_i[7:0];
            if (wb_sel_i[1]) mtime[47:40] <= wb_dat_i[15:8];
            if (wb_sel_i[2]) mtime[55:48] <= wb_dat_i[23:16];
            if (wb_sel_i[3]) mtime[63:56] <= wb_dat_i[31:24];
          end 
          32'h0200_4000: begin // mtimecmp
            if (wb_sel_i[0]) mtimecmp[7:0] <= wb_dat_i[7:0];
            if (wb_sel_i[1]) mtimecmp[15:8] <= wb_dat_i[15:8];
            if (wb_sel_i[2]) mtimecmp[23:16] <= wb_dat_i[23:16];
            if (wb_sel_i[3]) mtimecmp[31:24] <= wb_dat_i[31:24];
            mtime <= mtime + 1;            
          end 
          32'h0200_4004: begin // mtimecmp+4
            if (wb_sel_i[0]) mtimecmp[39:32] <= wb_dat_i[7:0];
            if (wb_sel_i[1]) mtimecmp[47:40] <= wb_dat_i[15:8];
            if (wb_sel_i[2]) mtimecmp[55:48] <= wb_dat_i[23:16];
            if (wb_sel_i[3]) mtimecmp[63:56] <= wb_dat_i[31:24];
            mtime <= mtime + 1;
          end
          default:
            mtime <= mtime + 1;           
        endcase // case (wb_adr_i)        
      end else // if (wb_cyc_i && wb_stb_i && wb_we_i)
        mtime <= mtime + 1;      
    end
  end

  // send ack immediately
  assign wb_ack_o = wb_cyc_i & wb_stb_i;

  // set time out
  assign timeout_o = mtime >= mtimecmp;

  always_comb begin
    wb_dat_o = 32'h0;
    if (wb_cyc_i && wb_stb_i && !wb_we_i) begin
      case (wb_adr_i) 
        32'h0200_BFF8: // mtime
          wb_dat_o = mtime[31:0];
        32'h0200_BFFC: // mtime+4
          wb_dat_o = mtime[63:32];
        32'h0200_4000: // mtimecmp
          wb_dat_o = mtimecmp[31:0];
        32'h0200_4004: // mtimecmp+4
          wb_dat_o = mtimecmp[63:32];                
      endcase // case (wb_adr_i)      
    end
  end
endmodule
