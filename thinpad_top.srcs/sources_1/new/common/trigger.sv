`default_nettype none

module trigger (
  input wire  clk,
  input wire  push_btn,
  output wire trigger
  );

  reg         btn_last_reg;
  reg         trigger_reg;
         
  
  always_ff @ (posedge clk) begin
    btn_last_reg <= push_btn;
    if (btn_last_reg == 1'b0 && push_btn == 1'b1)
      trigger_reg <= 1'b1;
    else
      trigger_reg <= 1'b0;                
  end
  
  assign trigger = trigger_reg;
  
endmodule // trigger

  
