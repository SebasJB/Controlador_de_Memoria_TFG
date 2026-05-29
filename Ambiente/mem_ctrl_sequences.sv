// ============================================================
//  File    : mem_ctrl_sequences.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : Sequence library — enfoque híbrido
//
//  Sequences:
//    mem_wr_base_seq / mem_rd_base_seq — helpers (do_write/do_read)
//    mem_phased_wr_seq / mem_phased_rd_seq — sequences fasables
//        que reciben "phase" como argumento y emiten N txns
//        con los constraints correspondientes activos.
//    mem_smoke_wr_seq / mem_smoke_rd_seq — 1 txn por banco
//    mem_rob_wrap_rd_seq — sequence temporal específica
//    mem_fifo_sat_wr_seq / mem_fifo_sat_rd_seq — back-to-back
//
//  Las sequences "fasables" son las que la maestra invoca con
//  distintas phases para orquestar todo el flujo desde el test.
// ============================================================


// ============================================================
// BASE — helpers do_write / do_read
// ============================================================

class mem_wr_base_seq #(
    parameter int ADDR_W           = 32,
    parameter int DATA_W           = 32,
    parameter int N_BANKS          = 4,
    parameter int BANK_SIZE_BYTES  = 1024
) extends uvm_sequence #(mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES));

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES) item_t;

    `uvm_object_param_utils(mem_wr_base_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
    function new(string name = "mem_wr_base_seq"); super.new(name); endfunction

    task do_write(bit [ADDR_W-1:0]   addr,
                  bit [DATA_W-1:0]   data,
                  bit [DATA_W/8-1:0] wstrb = '1);
        item_t item;
        item = item_t::type_id::create("wr_item");
        start_item(item);
        if (!item.randomize() with {
            is_write == 1'b1;
            addr  == local::addr;
            data  == local::data;
            wstrb == local::wstrb;
        }) `uvm_fatal("WR_SEQ", "Randomize failed")
        finish_item(item);
    endtask
endclass


class mem_rd_base_seq #(
    parameter int ADDR_W           = 32,
    parameter int DATA_W           = 32,
    parameter int N_BANKS          = 4,
    parameter int BANK_SIZE_BYTES  = 1024
) extends uvm_sequence #(mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES));

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES) item_t;

    `uvm_object_param_utils(mem_rd_base_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
    function new(string name = "mem_rd_base_seq"); super.new(name); endfunction

    task do_read(bit [ADDR_W-1:0] addr);
        item_t item;
        item = item_t::type_id::create("rd_item");
        start_item(item);
        if (!item.randomize() with {
            is_write == 1'b0;
            addr     == local::addr;
        }) `uvm_fatal("RD_SEQ", "Randomize failed")
        finish_item(item);
    endtask
endclass


// ============================================================
// PHASED — sequences que toman "phase" y "n_txns" como knobs.
//   Activa constraints layered del seq_item antes de cada
//   randomize(). La sequence maestra las invoca repetidamente.
// ============================================================

class mem_phased_wr_seq #(
    parameter int ADDR_W           = 32,
    parameter int DATA_W           = 32,
    parameter int N_BANKS          = 4,
    parameter int BANK_SIZE_BYTES  = 1024
) extends uvm_sequence #(mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES));

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES) item_t;

    // Knobs (seteados por la maestra antes de start())
    rand int n_txns;
    string phase_name   = "GENERAL";
    int    target_bank  = 0;

    `uvm_object_param_utils(mem_phased_wr_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
    function new(string name = "mem_phased_wr_seq"); super.new(name); endfunction

    task body();
        item_t item;
        repeat (n_txns) begin
            item = item_t::type_id::create("p_wr");
            item.cfg_target_bank = target_bank;
            item.set_phase(phase_name);
            start_item(item);
            if (!item.randomize() with { is_write == 1'b1; })
                `uvm_fatal("PH_WR", $sformatf("Randomize failed in phase %s", phase_name))
            finish_item(item);
        end
    endtask
endclass


class mem_phased_rd_seq #(
    parameter int ADDR_W           = 32,
    parameter int DATA_W           = 32,
    parameter int N_BANKS          = 4,
    parameter int BANK_SIZE_BYTES  = 1024
) extends uvm_sequence #(mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES));

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES) item_t;

    string phase_name   = "GENERAL";
    rand int n_txns;
    int    target_bank  = 0;

    `uvm_object_param_utils(mem_phased_rd_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
    function new(string name = "mem_phased_rd_seq"); super.new(name); endfunction

    task body();
        item_t item;
        repeat (n_txns) begin
            item = item_t::type_id::create("p_rd");
            item.cfg_target_bank = target_bank;
            item.set_phase(phase_name);
            start_item(item);
            if (!item.randomize() with { is_write == 1'b0; })
                `uvm_fatal("PH_RD", $sformatf("Randomize failed in phase %s", phase_name))
            finish_item(item);
        end
    endtask
endclass


// ============================================================
// SMOKE — 1 par WR/RD por banco
// ============================================================

class mem_smoke_wr_seq #(parameter ADDR_W=32, DATA_W=32, N_BANKS=4, BANK_SIZE_BYTES=1024)
    extends mem_wr_base_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES);
    `uvm_object_param_utils(mem_smoke_wr_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
    function new(string name = "mem_smoke_wr_seq"); super.new(name); endfunction

    task body();
        int word_bytes = DATA_W / 8;
        for (int b = 0; b < N_BANKS; b++)
            do_write(b * word_bytes, 32'hCAFE_0000 + b);
    endtask
endclass


class mem_smoke_rd_seq #(parameter ADDR_W=32, DATA_W=32, N_BANKS=4, BANK_SIZE_BYTES=1024)
    extends mem_rd_base_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES);
    `uvm_object_param_utils(mem_smoke_rd_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
    function new(string name = "mem_smoke_rd_seq"); super.new(name); endfunction

    task body();
        int word_bytes = DATA_W / 8;
        for (int b = 0; b < N_BANKS; b++)
            do_read(b * word_bytes);
    endtask
endclass


// ============================================================
// ROB_WRAP — fenómeno temporal (no expresable como constraint)
// Emite 2*N_BANKS+4 lecturas back-to-back; el driver con
// r_backpressure_cycles>0 llena el ROB antes del primer R fire.
// ============================================================

class mem_rob_wrap_rd_seq #(parameter ADDR_W=32, DATA_W=32, N_BANKS=4, BANK_SIZE_BYTES=1024)
    extends mem_rd_base_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES);
    `uvm_object_param_utils(mem_rob_wrap_rd_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
    function new(string name = "mem_rob_wrap_rd_seq"); super.new(name); endfunction

    task body();
        int word_bytes = DATA_W / 8;
        for (int i = 0; i < 2 * N_BANKS + 4; i++)
            do_read((i % N_BANKS) * word_bytes);
    endtask
endclass


// ============================================================
// FIFO_SAT — back-to-back para saturar FIFOs REQ
// ============================================================

class mem_fifo_sat_wr_seq #(parameter ADDR_W=32, DATA_W=32, N_BANKS=4, BANK_SIZE_BYTES=1024)
    extends mem_wr_base_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES);
    `uvm_object_param_utils(mem_fifo_sat_wr_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
    int num_txns = 64;
    function new(string name = "mem_fifo_sat_wr_seq"); super.new(name); endfunction

    task body();
        int word_bytes = DATA_W / 8;
        int max_word = (N_BANKS * BANK_SIZE_BYTES / word_bytes) - 1;
        for (int i = 0; i < num_txns; i++)
            do_write((i % max_word) * word_bytes, 32'hF1F0_0000 + i);
    endtask
endclass


class mem_fifo_sat_rd_seq #(parameter ADDR_W=32, DATA_W=32, N_BANKS=4, BANK_SIZE_BYTES=1024)
    extends mem_rd_base_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES);
    `uvm_object_param_utils(mem_fifo_sat_rd_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
    int num_txns = 64;
    function new(string name = "mem_fifo_sat_rd_seq"); super.new(name); endfunction

    task body();
        int word_bytes = DATA_W / 8;
        int max_word = (N_BANKS * BANK_SIZE_BYTES / word_bytes) - 1;
        for (int i = 0; i < num_txns; i++)
            do_read((i % max_word) * word_bytes);
    endtask
endclass


// ============================================================
// MEM_MASTER_SEQ_WR / MEM_MASTER_SEQ_RD
//   Sequences maestras que orquestan todas las fases.
//   Se lanzan en paralelo desde mem_full_test (una contra
//   write_sequencer, otra contra read_sequencer).
//
//   Las dos saben las mismas fases y se coordinan vía variables
//   de instancia que el test setea antes del start().
//
//   Plusarg +PHASE_ONLY=<name> permite saltar fases para debug.
// ============================================================

class mem_master_wr_seq #(parameter ADDR_W=32, DATA_W=32, N_BANKS=4, BANK_SIZE_BYTES=1024)
    extends uvm_sequence #(mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES));

    typedef mem_phased_wr_seq    #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES) ph_wr_t;
    typedef mem_fifo_sat_wr_seq  #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES) sat_wr_t;

    string phase_only = "ALL";   // "ALL" o nombre específico de fase

    `uvm_object_param_utils(mem_master_wr_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
    function new(string name = "mem_master_wr_seq"); super.new(name); endfunction

    function bit run_phase(string p);
        return (phase_only == "ALL" || phase_only == p);
    endfunction

    task body();
        ph_wr_t  ph;
        sat_wr_t sat;

        void'($value$plusargs("PHASE_ONLY=%s", phase_only));

        // ── Fase A: GENERAL (régimen normal, backbone coverage)
        if (run_phase("GENERAL")) begin
            `uvm_info("MASTER_WR", "=== Phase A: GENERAL ===", UVM_LOW)
            ph = ph_wr_t::type_id::create("ph_a");
            ph.phase_name = "GENERAL";
            if (!ph.randomize() with { n_txns inside {[100:200]}; })
                `uvm_fatal("MASTER_WR", "Randomize n_txns failed — GENERAL")
            `uvm_info("MASTER_WR", $sformatf("Cantidad de transacciones de fase: %0d", ph.n_txns), UVM_LOW)
            ph.start(m_sequencer);
        end

        // ── Fase B: SINGLE_BANK (cada banco aislado)
        if (run_phase("SINGLE_BANK")) begin
            for (int b = 0; b < N_BANKS; b++) begin
                `uvm_info("MASTER_WR", $sformatf("=== Phase B%0d: SINGLE_BANK=%0d (200 WR) ===", b, b), UVM_LOW)
                ph = ph_wr_t::type_id::create($sformatf("ph_b_%0d", b));
                ph.phase_name = "SINGLE_BANK"; ph.target_bank = b;
                if (!ph.randomize() with { n_txns inside {[100:300]}; })
                    `uvm_fatal("MASTER_WR", "Randomize n_txns failed — SINGLE_BANK")
                `uvm_info("MASTER_WR", $sformatf("Cantidad de transacciones de fase: %0d", ph.n_txns), UVM_LOW)
                ph.start(m_sequencer);
            end
        end

        // ── Fase C: CONFLICT (addrs concentradas para forzar same_bank)
        if (run_phase("CONFLICT")) begin
            `uvm_info("MASTER_WR", "=== Phase C: CONFLICT===", UVM_LOW)
            ph = ph_wr_t::type_id::create("ph_c");
            ph.phase_name = "CONFLICT";
            if (!ph.randomize() with { n_txns inside {[100:300]}; })
                `uvm_fatal("MASTER_WR", "Randomize n_txns failed — CONFLICT")
            `uvm_info("MASTER_WR", $sformatf("Cantidad de transacciones de fase: %0d", ph.n_txns), UVM_LOW)
            ph.start(m_sequencer);
        end

        // ── Fase D: INVALID_ADDR (M5)
        if (run_phase("INVALID_ADDR")) begin
            `uvm_info("MASTER_WR", "=== Phase D: INVALID_ADDR ===", UVM_LOW)
            ph = ph_wr_t::type_id::create("ph_d");
            ph.phase_name = "INVALID_ADDR";
            if (!ph.randomize() with { n_txns inside {[20:80]}; })
                `uvm_fatal("MASTER_WR", "Randomize n_txns failed — INVALID_ADDR")
            `uvm_info("MASTER_WR", $sformatf("Cantidad de transacciones de fase: %0d", ph.n_txns), UVM_LOW)
            ph.start(m_sequencer);
        end

        // ── Fase E: WSTRB_STRESS (cobertura granular byte-enable)
        if (run_phase("WSTRB_STRESS")) begin
            `uvm_info("MASTER_WR", "=== Phase E: WSTRB_STRESS (200 WR) ===", UVM_LOW)
            ph = ph_wr_t::type_id::create("ph_e");
            ph.phase_name = "WSTRB_STRESS";
            if (!ph.randomize() with { n_txns inside {[100:300]}; })
                `uvm_fatal("MASTER_WR", "Randomize n_txns failed — WSTRB_STRESS")
            `uvm_info("MASTER_WR", $sformatf("Cantidad de transacciones de fase: %0d", ph.n_txns), UVM_LOW)
            ph.start(m_sequencer);
        end

        // ── Fase F: FIFO_SAT (back-to-back, satura FIFOs REQ)
        if (run_phase("FIFO_SAT")) begin
            `uvm_info("MASTER_WR", "=== Phase F: FIFO_SATURATION (64 WR back-to-back) ===", UVM_LOW)
            sat = sat_wr_t::type_id::create("sat_wr");
            sat.start(m_sequencer);
        end
    endtask
endclass


class mem_master_rd_seq #(parameter ADDR_W=32, DATA_W=32, N_BANKS=4, BANK_SIZE_BYTES=1024)
    extends uvm_sequence #(mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES));

    typedef mem_phased_rd_seq    #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES) ph_rd_t;
    typedef mem_rob_wrap_rd_seq  #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES) wrap_rd_t;
    typedef mem_fifo_sat_rd_seq  #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES) sat_rd_t;

    string phase_only = "ALL";

    `uvm_object_param_utils(mem_master_rd_seq #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
    function new(string name = "mem_master_rd_seq"); super.new(name); endfunction

    function bit run_phase(string p);
        return (phase_only == "ALL" || phase_only == p);
    endfunction

    task body();
        ph_rd_t   ph;
        wrap_rd_t wrap;
        sat_rd_t  sat;

        void'($value$plusargs("PHASE_ONLY=%s", phase_only));

        if (run_phase("GENERAL")) begin
            `uvm_info("MASTER_RD", "=== Phase A: GENERAL ===", UVM_LOW)
            ph = ph_rd_t::type_id::create("ph_a");
            ph.phase_name = "GENERAL";
            if (!ph.randomize() with { n_txns inside {[100:200]}; })
                `uvm_fatal("MASTER_RD", "Randomize n_txns failed — GENERAL")
            `uvm_info("MASTER_RD", $sformatf("Cantidad de transacciones de fase: %0d", ph.n_txns), UVM_LOW)
            ph.start(m_sequencer);
        end

        if (run_phase("SINGLE_BANK")) begin
            for (int b = 0; b < N_BANKS; b++) begin
                `uvm_info("MASTER_RD", $sformatf("=== Phase B%0d: SINGLE_BANK=%0d (200 RD) ===", b, b), UVM_LOW)
                ph = ph_rd_t::type_id::create($sformatf("ph_b_%0d", b));
                ph.phase_name = "SINGLE_BANK"; ph.target_bank = b;
                if (!ph.randomize() with { n_txns inside {[100:300]}; })
                    `uvm_fatal("MASTER_RD", "Randomize n_txns failed — SINGLE_BANK")
                `uvm_info("MASTER_RD", $sformatf("Cantidad de transacciones de fase: %0d", ph.n_txns), UVM_LOW)
                ph.start(m_sequencer);
            end
        end

        if (run_phase("CONFLICT")) begin
            `uvm_info("MASTER_RD", "=== Phase C: CONFLICT (300 RD) ===", UVM_LOW)
            ph = ph_rd_t::type_id::create("ph_c");
            ph.phase_name = "CONFLICT";
            if (!ph.randomize() with { n_txns inside {[100:300]}; })
                `uvm_fatal("MASTER_RD", "Randomize n_txns failed — CONFLICT")
            `uvm_info("MASTER_RD", $sformatf("Cantidad de transacciones de fase: %0d", ph.n_txns), UVM_LOW)
            ph.start(m_sequencer);
        end

        if (run_phase("INVALID_ADDR")) begin
            `uvm_info("MASTER_RD", "=== Phase D: INVALID_ADDR (50 RD) ===", UVM_LOW)
            ph = ph_rd_t::type_id::create("ph_d");
            ph.phase_name = "INVALID_ADDR";
            if (!ph.randomize() with { n_txns inside {[20:80]}; })
                `uvm_fatal("MASTER_RD", "Randomize n_txns failed — INVALID_ADDR")
            `uvm_info("MASTER_RD", $sformatf("Cantidad de transacciones de fase: %0d", ph.n_txns), UVM_LOW)
            ph.start(m_sequencer);
        end

        // ── Fase G: ROB_WRAP (específica RD, fenómeno temporal)
        // Requiere driver con r_backpressure_cycles>0 — el test
        // lo configura vía config_db antes de invocar.
        if (run_phase("ROB_WRAP")) begin
            `uvm_info("MASTER_RD", "=== Phase G: ROB_WRAP ===", UVM_LOW)
            wrap = wrap_rd_t::type_id::create("wrap");
            wrap.start(m_sequencer);
        end

        if (run_phase("FIFO_SAT")) begin
            `uvm_info("MASTER_RD", "=== Phase F: FIFO_SATURATION (64 RD back-to-back) ===", UVM_LOW)
            sat = sat_rd_t::type_id::create("sat_rd");
            sat.start(m_sequencer);
        end
    endtask
endclass
