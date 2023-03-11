// CSR I/O module as well as trap management module
// since traps will modify lots of CSRs.

`include "defines.sv"


// code of interrupts and exceptions
`define C_MTI 7
`define C_STI 5

`define C_INVALID_INST 2
`define C_ECALL_U 8
`define C_ECALL_S 9
`define C_ECALL_M 11
`define C_EBREAK 3
`define C_INST_PF 12
`define C_INST_MISALIGN 0
`define C_MEM_MISALIGN_W 6
`define C_MEM_MISALIGN_R 4
`define C_MEM_PF_W 15
`define C_MEM_PF_R 13

// code of mstatus
`define S_MIE 3
`define S_MPIE 7
`define S_MPP 12:11
`define S_SPP 8
`define S_SPIE 5
`define S_SIE 1
`define S_SUM 18

module csr_trap (
  input wire        clk,
  input wire        rst,

  // in ID & WB, CSR R/W
  input wire [11:0] addr_r_i,
  input wire [11:0] addr_w_i,
  input wire        re_i, // read enable
  input wire        we_i, // write enable
  input wire [31:0] data_w_i, 
  output reg [31:0] data_r_o,
  output reg        invalid_inst_o,
  input wire [63:0] mtime_i,

  // read from WB
  input wire [1:0]  interrupt_i,
  input wire        invalid_inst_i,
  input wire        inst_page_fault_i,
  input wire        inst_misaligned_i,
  input wire        mem_page_fault_w_i,
  input wire        mem_page_fault_r_i,
  input wire        mem_misaligned_w_i,
  input wire        mem_misaligned_r_i,
  input wire        ecall_i,
  input wire        ebreak_i,
  input wire        mret_i,
  input wire        sret_i,
  input wire [31:0] pc_i,
  input wire [31:0] val_i,
  // stall check
  input wire        if_stall,
  
  // output to WB
  output reg [31:0] pc_o,
  output reg        trap_o,

  // used in all stages
  output reg [1:0]  mode_o,
  output reg [31:0] satp_o,
  output reg        sum_o,

  // send to IF
  input wire        timeout_i,
  output reg [1:0]  int_type
  );

  logic [1:0]       cur_mode;   
  // CSRs
  logic [31:0]      mtvec;
  logic [31:0]      mscratch;
  logic [31:0]      mepc;
  logic [31:0]      mcause;
  logic [31:0]      mstatus;
  logic [31:0]      mie;
  logic [31:0]      mip;
  logic [31:0]      mtval;
  logic [31:0]      medeleg;
  logic [31:0]      mideleg;  

  logic [31:0]      satp;
  logic [31:0]      sepc;
  logic [31:0]      scause;
  logic [31:0]      sscratch;
  logic [31:0]      stval;
  logic [31:0]      stvec;

  always_comb begin
    invalid_inst_o = 1'b0;    
  end

  
  // CSR read
  always_comb begin
    data_r_o = 32'h0000_0000;
    if (re_i) begin
      case (addr_r_i)
        12'hF14: // mhartid
          if (cur_mode == `MODE_M) begin
            data_r_o = 32'b0;            
          end
        12'h300: // mstatus
          if (cur_mode == `MODE_M) begin
            data_r_o = mstatus;          
          end 
        12'h302: // medeleg
          if (cur_mode == `MODE_M) begin
            data_r_o = medeleg;
          end
        12'h303: // mideleg
          if (cur_mode == `MODE_M) begin
            data_r_o = mideleg;
          end
        12'h304: // mie
          if (cur_mode == `MODE_M) begin
            data_r_o = mie;
          end
        12'h305: // mtvec
          if (cur_mode == `MODE_M) begin
            data_r_o = mtvec;          
          end
        12'h340: // mscratch
          if (cur_mode == `MODE_M) begin
            data_r_o = mscratch;          
          end
        12'h341: // mepc
          if (cur_mode == `MODE_M) begin
            data_r_o = mepc;
          end
        12'h342: // mcause
          if (cur_mode == `MODE_M) begin
            data_r_o = mcause;          
          end
        12'h343: // mtval
          if (cur_mode == `MODE_M) begin
            data_r_o = mtval;            
          end
        12'h344: // mip
          if (cur_mode == `MODE_M) begin
            data_r_o = mip;
          end
        12'h100: // sstatus, return mstatus
          if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
            data_r_o = mstatus;            
          end
        12'h104: // sie, use mie
          if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
            data_r_o = mie;            
          end
        12'h105: // stvec
          if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
            data_r_o = stvec;         
          end
        12'h140: // sscratch
          if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
            data_r_o = sscratch;            
          end
        12'h141: // sepc
          if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
            data_r_o = sepc;            
          end
        12'h142: // scause
          if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
            data_r_o = scause;            
          end
        12'h143: // stval
          if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
            data_r_o = stval;            
          end
        12'h144: // sip, use mip
          if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
            data_r_o = mip;           
          end
        12'h180: // satp
          if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
            data_r_o = satp;            
          end
        12'hC01: // time (rdtime)
          data_r_o = mtime_i[31:0];
        12'hC81: // timeh (rdtimeh)
          data_r_o = mtime_i[63:32];
      endcase // case (addr_r_i)
    end
  end // always_comb


  // Interrupt process in IF
  always_comb begin
    int_type = `INT_NONE;
    if (mie[`C_MTI] && timeout_i && (cur_mode != `MODE_M || mstatus[`S_MIE])) begin // MTI(7)
      int_type = `INT_TIMER_M;      
    end
    else if (mie[`C_STI] && mip[`C_STI] && (cur_mode == `MODE_U || (cur_mode == `MODE_S && mstatus[`S_SIE]))) begin // STI
      int_type = `INT_TIMER_S;
    end
  end

  logic [31:0] cause_val;  

  // Trap detection in WB
  typedef enum logic [3:0] {
               T_INT,
               T_EXCEPTION,
               T_ERET,
               T_NOTRAP
               } trap_t;
  trap_t trap_type;

  logic        mtrap;  

  assign mtrap = ~(cause_val[31] ? 
                   mideleg[cause_val[4:0]] :  // interrupt
                   medeleg[cause_val[4:0]]);  // exception
  
  always_comb begin
    trap_type = T_NOTRAP;
    cause_val = 32'h0;
    
    // Interrupt
    if (interrupt_i != `INT_NONE) begin
      trap_type = T_INT;
      if (interrupt_i == `INT_TIMER_M) begin
        cause_val = {1'b1, 31'd`C_MTI};
      end else begin // INT_TIMER_S
        cause_val = {1'b1, 31'd`C_STI};
      end
    end
    // Exception
    else if (inst_page_fault_i) begin
      trap_type = T_EXCEPTION;
      cause_val = {1'b0, 31'd`C_INST_PF};
      
    end else if (inst_misaligned_i) begin
      trap_type = T_EXCEPTION;
      cause_val = {1'b0, 31'd`C_INST_MISALIGN};
      
    end else if (invalid_inst_i) begin
      trap_type = T_EXCEPTION;
      cause_val = {1'b0, 31'd`C_INVALID_INST};
      
    end else if (ecall_i) begin
      trap_type = T_EXCEPTION;
      
      if (cur_mode == `MODE_U) begin      
        cause_val = {1'b0, 31'd`C_ECALL_U};
      end else if (cur_mode == `MODE_S) begin
        cause_val = {1'b0, 31'd`C_ECALL_S};
      end else begin // Mode-M
        cause_val = {1'b0, 31'd`C_ECALL_M};
      end                 

    end else if (ebreak_i) begin
      trap_type = T_EXCEPTION;
      cause_val = {1'b0, 31'd`C_EBREAK};
      
    end else if (mem_misaligned_w_i) begin
      trap_type = T_EXCEPTION;
      cause_val = {1'b0, 31'd`C_MEM_MISALIGN_W};
      
    end else if (mem_misaligned_r_i) begin
      trap_type = T_EXCEPTION;
      cause_val = {1'b0, 31'd`C_MEM_MISALIGN_R};
      
    end else if (mem_page_fault_w_i) begin
      trap_type = T_EXCEPTION;
      cause_val = {1'b0, 31'd`C_MEM_PF_W};
    end else if (mem_page_fault_r_i) begin
      trap_type = T_EXCEPTION;
      cause_val = {1'b0, 31'd`C_MEM_PF_R};
    end
    // (m/s)ret
    else if (mret_i || sret_i) begin
      trap_type = T_ERET;
    end
  end

  logic timeout_tmp1;  
  
  always_ff @ (posedge clk) begin
    if (rst) begin // CSR init
      cur_mode <= `MODE_M;
      mcause <= 32'h0;   
      mstatus <= 32'h0;
      mip <= 32'h0;
      mie <= 32'h0;
      mepc <= 32'h0;
      mtvec <= 32'h0;
      mscratch <= 32'h0;
      mtval <= 32'h0;
      medeleg <= 32'h0;
      mideleg <= 32'h0;

      satp <= 32'h0;
      sepc <= 32'h0;
      scause <= 32'h0;
      sscratch <= 32'h0;
      stval <= 32'h0;
      stvec <= 32'h0;      
      satp <= 32'h0;
      
    end else begin
      // outside interrupts
      // we suspend it by one cycle,
      // because the instruction is in IF yet.
      // (this is not essensial, to be honest)
      timeout_tmp1 <= timeout_i;      
      mip[`C_MTI] <= timeout_tmp1;
            
      // traps
      if (trap_type != T_NOTRAP) begin

        // change state iff next clk cycle can
        // transfer to other state
        // Otherwise, some variables (cur_mode) will
        // change multiple times
        if (!if_stall) begin
          if (trap_type == T_ERET) begin
            if (mret_i) begin
              mstatus[`S_MPIE] <= 1'b1;
              mstatus[`S_MIE] <= mstatus[`S_MPIE];
              mstatus[`S_MPP] <= `MODE_U;
              cur_mode <= mstatus[`S_MPP];
            end else begin
              mstatus[`S_SPIE] <= 1'b1;
              mstatus[`S_SIE] <= mstatus[`S_SPIE];
              mstatus[`S_SPP] <= 1'b0; // MODE_U
              cur_mode <= {1'b0, mstatus[`S_SPP]};
            end
          end else begin
            // general process of transfer control
            if (mtrap) begin
              mstatus[`S_MPIE] <= mstatus[`S_MIE];
              mstatus[`S_MIE] <= 1'b0;
              mstatus[`S_MPP] <= cur_mode;
              cur_mode <= `MODE_M;
              mepc <= pc_i;
              mtval <= val_i;            
              mcause <= cause_val;
            end else begin // strap
              mstatus[`S_SPIE] <= mstatus[`S_SIE];
              mstatus[`S_SIE] <= 1'b0;
              mstatus[`S_SPP] <= cur_mode[0];
              cur_mode <= `MODE_S;
              sepc <= pc_i;
              stval <= val_i;            
              scause <= cause_val;
            end            
          end // else: !if(trap_type = T_MRET)
        end // if (!if_stall)
        
      end else begin // if (trap_type != T_NO_TRAP)        
        if (we_i) begin // No trap, CSR write
          case (addr_w_i)
            12'h300: // mstatus
              if (cur_mode == `MODE_M) begin
                // MPP
                if (data_w_i[`S_MPP] != 2'b10)
                  mstatus[`S_MPP] <= data_w_i[`S_MPP];
                // MIE
                mstatus[`S_MIE] <= data_w_i[`S_MIE];
                // MPIE
                mstatus[`S_MPIE] <= data_w_i[`S_MPIE];
                mstatus[`S_SPP] <= data_w_i[`S_SPP];
                mstatus[`S_SIE] <= data_w_i[`S_SIE];
                mstatus[`S_SPIE] <= data_w_i[`S_SPIE];
                mstatus[`S_SUM] <= data_w_i[`S_SUM];
              end // if (cur_mode == `MODE_M)
            12'h302: // medeleg
              if (cur_mode == `MODE_M) begin
                medeleg[`C_INVALID_INST] <= data_w_i[`C_INVALID_INST];
                medeleg[`C_ECALL_U] <= data_w_i[`C_ECALL_U];
                medeleg[`C_ECALL_S] <= data_w_i[`C_ECALL_S];
                // No Ecall_M deleg allowed
                medeleg[`C_EBREAK] <= data_w_i[`C_EBREAK];
                medeleg[`C_INST_PF] <= data_w_i[`C_INST_PF];
                medeleg[`C_INST_MISALIGN] <= data_w_i[`C_INST_MISALIGN];
                medeleg[`C_MEM_MISALIGN_W] <= data_w_i[`C_MEM_MISALIGN_W];
                medeleg[`C_MEM_MISALIGN_R] <= data_w_i[`C_MEM_MISALIGN_R];
                medeleg[`C_MEM_PF_W] <= data_w_i[`C_MEM_PF_W];
                medeleg[`C_MEM_PF_R] <= data_w_i[`C_MEM_PF_R];                
              end
            12'h303: //mideleg
              if (cur_mode == `MODE_M) begin
                mideleg[`C_MTI] <= data_w_i[`C_MTI];
                mideleg[`C_STI] <= data_w_i[`C_STI];                
              end
            12'h304: // mie
              if (cur_mode == `MODE_M) begin
                // MTIE
                mie[`C_MTI] <= data_w_i[`C_MTI];
                mie[`C_STI] <= data_w_i[`C_STI];                
              end
            12'h305: // mtvec
              if (cur_mode == `MODE_M) begin
                // we do not check alignment
                mtvec[31:2] <= data_w_i[31:2];
                // MODE
                if (data_w_i[1:0] == 2'b00)
                  mtvec[1:0] <= data_w_i[1:0];            
              end
            12'h340: // mscratch
              if (cur_mode == `MODE_M) begin
                mscratch <= data_w_i;       
              end
            12'h341: // mepc
              if (cur_mode == `MODE_M) begin
                // force align
                mepc[31:2] <= data_w_i[31:2];
              end
            12'h342: // mcause
              if (cur_mode == `MODE_M) begin
                // WLRL, directly copy
                mcause <= data_w_i;          
              end
            12'h343: // mtval
              if (cur_mode == `MODE_M) begin
                mtval <= data_w_i;            
              end
            12'h344: // mip
              if (cur_mode == `MODE_M) begin
                // MTIP is hard wired
                mip[`C_STI] <= data_w_i[`C_STI];                
              end
            12'h100: // sstatus, use mstatus
              if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
                mstatus[`S_SPP] <= data_w_i[`S_SPP];
                mstatus[`S_SIE] <= data_w_i[`S_SIE];
                mstatus[`S_SPIE] <= data_w_i[`S_SPIE];
                mstatus[`S_SUM] <= data_w_i[`S_SUM];
              end
            12'h104: // sie, use mie
              if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
                mie[`C_STI] <= data_w_i[`C_STI];
              end
            12'h105: // stvec
              if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
                // we do not check alignment
                stvec[31:2] <= data_w_i[31:2];
                // MODE
                if (data_w_i[1:0] == 2'b00)
                  stvec[1:0] <= data_w_i[1:0];
              end
            12'h140: // sscratch
              if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
                sscratch <= data_w_i;       
              end
            12'h141: // sepc
              if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
                sepc[31:2] <= data_w_i[31:2];
              end
            12'h142: // scause
              if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
                scause <= data_w_i;          
              end
            12'h143: // stval
              if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
                stval <= data_w_i;            
              end
            12'h144: // sip, use mip
              if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
                mip[`C_STI] <= data_w_i[`C_STI];
              end  
            12'h180: // satp
              if (cur_mode == `MODE_M || cur_mode == `MODE_S) begin
                // ASID(30:22) field is not implemented
                {satp[31], satp[21:0]} <= {data_w_i[31], data_w_i[21:0]};
              end
          endcase // case (addr_w_i)
        end // if (we_i)
      end // else: !if(trap_type != T_NO_TRAP)        
    end    
  end

  // Outside transfer control
  always_comb begin
    pc_o = 32'h0;
    trap_o = 1'b0;
    if (trap_type == T_ERET) begin
      pc_o = mret_i ? mepc : sepc;
      trap_o = 1'b1;      
    end else if (trap_type != T_NOTRAP) begin
      pc_o = {mtrap ? mtvec[31:2] : stvec[31:2], 2'b0};
      trap_o = 1'b1;
    end
  end
    
  assign mode_o = cur_mode;
  assign satp_o = satp;
  assign sum_o = mstatus[`S_SUM];  
  
endmodule // csr_trap

