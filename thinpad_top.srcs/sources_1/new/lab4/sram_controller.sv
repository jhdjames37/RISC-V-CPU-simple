`default_nettype none

module sram_controller #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,

    parameter SRAM_ADDR_WIDTH = 20,
    parameter SRAM_DATA_WIDTH = 32,

    localparam SRAM_BYTES = SRAM_DATA_WIDTH / 8,
    localparam SRAM_BYTE_WIDTH = $clog2(SRAM_BYTES)
) (
    // clk and reset
    input wire clk_i,
    input wire rst_i,

    // wishbone slave interface
    input wire wb_cyc_i,
    input wire wb_stb_i,
    output reg wb_ack_o,
    input wire [ADDR_WIDTH-1:0] wb_adr_i,
    input wire [DATA_WIDTH-1:0] wb_dat_i,
    output reg [DATA_WIDTH-1:0] wb_dat_o,
    input wire [DATA_WIDTH/8-1:0] wb_sel_i,
    input wire wb_we_i,

    // sram interface
    output reg [SRAM_ADDR_WIDTH-1:0] sram_addr,
    inout wire [SRAM_DATA_WIDTH-1:0] sram_data,
    output reg sram_ce_n,
    output reg sram_oe_n,
    output reg sram_we_n,
    output reg [SRAM_BYTES-1:0] sram_be_n
);

  typedef enum logic [3:0]{
               IDLE,
               READ1,
               WRITE1,
               WRITE2
               } controller_state_t;
   controller_state_t state, nxt_state;

  
  // check whether data comes in
  wire                        data_in;
  assign data_in = wb_cyc_i & wb_stb_i;


  // flip-flop
  always_ff @ (posedge clk_i) begin
    if (rst_i) begin
      state <= IDLE;
    end else begin
      state <= nxt_state;
    end
  end
    
  // state trans
  always_comb begin
    nxt_state = state;

    case (state)
      IDLE: begin
        if (data_in) begin
          if (wb_we_i) nxt_state = WRITE1;
          else nxt_state = READ1;          
        end
      end
      READ1: nxt_state = IDLE;
      WRITE1: nxt_state = WRITE2;
      WRITE2: nxt_state = IDLE;      
      default: nxt_state = IDLE;      
    endcase // case (state)    
  end // always_comb

  // sram tri-state config
  // using lab's doc approach
  
  logic [SRAM_DATA_WIDTH-1:0] sram_i;
  logic [SRAM_DATA_WIDTH-1:0] sram_o;
  logic                       sram_writing;
  

  // we_n ? disable : enable
  
  assign sram_data = sram_writing ? sram_o : {SRAM_DATA_WIDTH{1'bz}} ;
  assign sram_i = sram_data;  

  
  // output item
  always_comb begin
    wb_ack_o = 1'b0;
    wb_dat_o = {DATA_WIDTH{1'b0}};
    sram_oe_n = 1'b1;
    sram_we_n = 1'b1;
    sram_ce_n = 1'b1;
    sram_writing = 1'b0;    
    
    sram_o = wb_dat_i;   // for convience    
    sram_addr = wb_adr_i[SRAM_ADDR_WIDTH+1:2]; // for convience    
    sram_be_n = ~wb_sel_i; // for convience

    case (state)
      IDLE: begin
        if (data_in) begin
          if (wb_we_i) begin // WRITE0
            sram_writing = 1'b1; 
            sram_ce_n = 1'b0;           
          end else begin // READ0
            sram_oe_n = 1'b0;
            sram_ce_n = 1'b0;       
          end          
        end        
      end // case: IDLE
      READ1: begin
        sram_oe_n = 1'b0;
        sram_ce_n = 1'b0;        
        wb_ack_o = 1'b1;
        wb_dat_o = sram_i;        
      end      
      WRITE1: begin
        sram_writing = 1'b1; 
        sram_we_n = 1'b0;
        sram_ce_n = 1'b0;        
      end
      WRITE2: begin
        sram_writing = 1'b1; 
        sram_ce_n = 1'b0;        
        wb_ack_o = 1'b1;
      end     
    endcase // case (state)
  end // always_comb  

endmodule
