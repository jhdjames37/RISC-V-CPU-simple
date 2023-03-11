module mem_controller(


  input wire        clk,
  input wire        rst,
  // interface to CPU.
  input wire [31:0] addr_i,
  input wire [31:0] data_i,
  input wire [3:0]  sel_i, 
  input wire        we_i, // write enable
  input wire        re_i, // read enable
  output reg [31:0] data_o, 
  output reg        mem_wait_o, // whether we should wait

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

  typedef enum logic [2:0] {
               ST_IDLE,
               ST_WAIT,
               ST_DONE
               } state_t;
  state_t state;

  always_ff @ (posedge clk) begin
    if (rst) begin
      state <= ST_IDLE;      
    end else begin
      case (state)
        ST_IDLE: begin
          if (we_i || re_i) begin
            state <= ST_WAIT;            
          end
        end
        ST_WAIT: begin
          if (wb_ack_i) begin
            state <= ST_DONE;
          end
        end
        ST_DONE: begin
          state <= ST_IDLE;
        end                     
      endcase
    end
  end // always_ff @ (posedge clk)

  
  always_comb begin
    wb_adr_o = addr_i;
    wb_dat_o = data_i;
    data_o = wb_dat_i;
    wb_we_o = 1'b0;
    wb_cyc_o = 1'b0;
    mem_wait_o = 1'b0;
    wb_sel_o = sel_i;
    
    
    case (state)
      ST_IDLE: begin
        if (we_i || re_i) begin
          wb_we_o = we_i & ~re_i;
          wb_cyc_o = 1'b1;
          mem_wait_o = 1'b1;          
        end
      end
      ST_WAIT: begin
        wb_we_o = we_i & ~re_i;
        wb_cyc_o = 1'b1;
        mem_wait_o = ~wb_ack_i;        
      end
      ST_DONE: begin // to lower cyc & stb a cycle, according to TA

        wb_cyc_o = 1'b0;
        if (we_i || re_i) begin
          mem_wait_o = 1'b1;
        end
      end
    endcase
  end
  assign wb_stb_o = wb_cyc_o;  
  
endmodule // mem_controller

  
