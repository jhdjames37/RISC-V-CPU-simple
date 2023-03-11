`include "defines.sv"



module mem_decoder (
  input wire [1:0]   addr_i,
  input wire [1:0]   sel_i,
  input wire         load_ext_type_i,
  input wire [31:0]  data_w_i, // from EXE
  input wire [31:0]  data_r_i, // from DMEM
  
  output reg [31:0] data_w_o, // to DMEM
  output reg [31:0] data_r_o, // to WB
  output reg [3:0]   sel_o,
  output reg         bad_addr
  );

  // since it's a combintional logic,
  // it's proper to combine two parts
  // - before SRAM and after SRAM
  // together
  
  always_comb begin
    sel_o = 4'b0;
    bad_addr = 1'b0;
    data_w_o = 32'b0;
    data_r_o = 32'b0;    
    
    case(sel_i)
      `SRAM_SEL_1: begin
        case(addr_i)
          2'b00: begin
            sel_o = 4'b0001;
            data_w_o[7:0] = data_w_i[7:0];            
            data_r_o[7:0] = data_r_i[7:0];
          end
          2'b01: begin
            sel_o = 4'b0010;
            data_w_o[15:8] = data_w_i[7:0];
            data_r_o[7:0] = data_r_i[15:8];
          end
          2'b10: begin
            sel_o = 4'b0100;
            data_w_o[23:16] = data_w_i[7:0];
            data_r_o[7:0] = data_r_i[23:16];
          end
          2'b11: begin
            sel_o = 4'b1000;
            data_w_o[31:24] = data_w_i[7:0];
            data_r_o[7:0] = data_r_i[31:24];
          end
        endcase         
        if (load_ext_type_i == `LOAD_EXT_ZERO) data_r_o[31:8] = 24'b0;
        else data_r_o[31:8] = {{24{data_r_o[7]}}};
      end
      `SRAM_SEL_2: begin
        case(addr_i)
          2'b00: begin
            sel_o = 4'b0011;
            data_w_o[15:0] = data_w_i[15:0];
            data_r_o[15:0] = data_r_i[15:0];
          end
          2'b10: begin
            sel_o = 4'b1100;
            data_w_o[31:16] = data_w_i[15:0];
            data_r_o[15:0] = data_r_i[31:16];            
          end
          default: bad_addr = 1'b1;
        endcase // case (addr_i)        
        if (load_ext_type_i == `LOAD_EXT_ZERO) data_r_o[31:16] = 16'b0;
        else data_r_o[31:16] = {{16{data_r_o[15]}}};
      end
      `SRAM_SEL_4: begin
        if (addr_i == 2'b00) begin
          sel_o = 4'b1111;
          data_w_o = data_w_i;
          data_r_o = data_r_i;
        end
        else bad_addr = 1'b1;        
      end        
    endcase // case (mem_sram_sel_reg)    
  end

endmodule // mem_decoder

  
