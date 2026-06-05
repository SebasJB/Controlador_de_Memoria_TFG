// ============================================================
//  File    : mem_ctrl_env.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Environment
//
//  Componentes:
//    - write_agent  (ACTIVE)
//    - read_agent   (ACTIVE)
//    - scoreboard
//    - coverage
//
//  Conexiones:
//    write_agent.ap_wr → scoreboard.ap_wr_imp + coverage.ap_wr_imp
//    write_agent.ap_b  → scoreboard.ap_b_imp
//    read_agent.ap_ar  → scoreboard.ap_ar_imp + coverage.ap_ar_imp
//    read_agent.ap_r   → scoreboard.ap_r_imp
// ============================================================

class mem_ctrl_env #(
    parameter int ADDR_W           = 32,
    parameter int DATA_W           = 32,
    parameter int N_BANKS          = 4,
    parameter int BANK_SIZE_BYTES  = 1024,
    parameter int WR_FIFO_DEPTH    = 8,
    parameter int RD_FIFO_DEPTH    = 8
) extends uvm_env;

    write_agent #(ADDR_W, DATA_W, N_BANKS) wr_agent;
    read_agent #(ADDR_W, DATA_W, N_BANKS) rd_agent;
    mem_ctrl_scoreboard #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES) sb;
    mem_ctrl_coverage #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES,WR_FIFO_DEPTH, RD_FIFO_DEPTH) cov;

    `uvm_component_param_utils(mem_ctrl_env #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES, WR_FIFO_DEPTH, RD_FIFO_DEPTH))

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        wr_agent = write_agent #(ADDR_W, DATA_W, N_BANKS)::type_id::create(
                    "wr_agent", this);
        rd_agent = read_agent  #(ADDR_W, DATA_W, N_BANKS)::type_id::create(
                    "rd_agent", this);
        sb       = mem_ctrl_scoreboard #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES)::type_id::create(
                    "sb", this);
        cov      = mem_ctrl_coverage #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES, WR_FIFO_DEPTH, RD_FIFO_DEPTH)::type_id::create(
                    "cov", this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        // write_agent → scoreboard + coverage
        wr_agent.ap_wr.connect(sb.ap_wr_imp);
        wr_agent.ap_wr.connect(cov.ap_wr_imp);
        wr_agent.ap_b.connect (sb.ap_b_imp);
        // read_agent → scoreboard + coverage
        rd_agent.ap_ar.connect(sb.ap_ar_imp);
        rd_agent.ap_ar.connect(cov.ap_ar_imp);
        rd_agent.ap_r.connect (sb.ap_r_imp);
    endfunction

endclass
