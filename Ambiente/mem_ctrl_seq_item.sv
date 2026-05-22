// ============================================================
//  File    : mem_ctrl_seq_item.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM seq_item — transacción base AXI4-Lite
//
//  Refactor (enfoque híbrido):
//    Constraints LAYERED activados/desactivados desde la
//    sequence maestra para orquestar fases del test:
//      - default        : random global (todo válido)
//      - force_invalid  : direcciones fuera de rango (M5)
//      - force_bank     : fija banco objetivo (single_bank)
//      - force_op_wr    : solo writes
//      - force_op_rd    : solo reads
//      - force_addr_lo  : concentra accesos (conflict)
//      - force_wstrb_partial : ejercita patrones byte-enable
//
//  Las fases temporales (rob_wrap, fifo_saturation) NO se
//  resuelven aquí — se manejan como sub-sequences específicas.
//  Las fases de backpressure tampoco — son knobs del driver.
// ============================================================

class mem_ctrl_seq_item #(
    parameter int ADDR_W           = 32,
    parameter int DATA_W           = 32,
    parameter int N_BANKS          = 4,
    parameter int BANK_SIZE_BYTES  = 1024
) extends uvm_sequence_item;

    // ── Campos randomizables ─────────────────────────────
    rand bit                 is_write;
    rand bit [ADDR_W-1:0]    addr;
    rand bit [DATA_W-1:0]    data;
    rand bit [DATA_W/8-1:0]  wstrb;
    rand int delay_cycles;   // para secuencias con backpressure


    // ── Configurables para fases (no random) ─────────────
    int  cfg_target_bank   = 0;

    // ── Observados ───────────────────────────────────────
    bit [1:0]                resp;

    // ── Tracking ─────────────────────────────────────────
    int unsigned             txn_id;
    static int unsigned      id_counter = 0;
    time                     t_req_fire;
    time                     t_resp_fire;

    // ── Constantes derivadas (para constraints) ──────────
    localparam int WORD_BYTES   = DATA_W / 8;
    localparam int OFF_BITS     = $clog2(WORD_BYTES);
    localparam int BANK_BITS    = $clog2(N_BANKS);
    localparam int TOTAL_BYTES  = N_BANKS * BANK_SIZE_BYTES;

    `uvm_object_param_utils_begin(mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))
        `uvm_field_int(is_write, UVM_ALL_ON)
        `uvm_field_int(addr,     UVM_ALL_ON)
        `uvm_field_int(data,     UVM_ALL_ON)
        `uvm_field_int(wstrb,    UVM_ALL_ON)
        `uvm_field_int(resp,     UVM_ALL_ON)
        `uvm_field_int(txn_id,   UVM_ALL_ON | UVM_DEC)
    `uvm_object_utils_end

    function new(string name = "mem_ctrl_seq_item");
        super.new(name);
        txn_id = id_counter++;

        // ── FIX: garantizar estado inicial limpio ──
        // Todas las c_force_* arrancan OFF; los defaults ON.
        // Esto evita conflictos cuando do_write() / do_read()
        // randomizan sin pasar por set_phase() primero.
        c_default_addr_valid.constraint_mode(1);
        c_default_wstrb.constraint_mode(1);
        c_force_invalid.constraint_mode(0);
        c_force_bank.constraint_mode(0);
        c_force_op_wr.constraint_mode(0);
        c_force_op_rd.constraint_mode(0);
        c_force_addr_lo.constraint_mode(0);
        c_force_wstrb_partial.constraint_mode(0);
    endfunction


    // ========================================================
    // CONSTRAINTS LAYERED
    // ========================================================

    // Default: addr válida + alineada. ON salvo INVALID_ADDR.
    constraint c_default_addr_valid {
        soft addr < TOTAL_BYTES;
        soft addr[OFF_BITS-1:0] == '0;
    }

    // Default: wstrb all-ones. ON salvo WSTRB_STRESS.
    constraint c_default_wstrb {
        soft wstrb == {(DATA_W/8){1'b1}};
    }

    // force_invalid: addr fuera de rango. OFF por defecto.
    constraint c_force_invalid {
        soft addr >= TOTAL_BYTES;
        soft addr <  TOTAL_BYTES * 2;
    }

    // force_bank: fuerza banco objetivo. OFF por defecto.
    constraint c_force_bank {
        addr[OFF_BITS+BANK_BITS-1 : OFF_BITS] == cfg_target_bank[BANK_BITS-1:0];
    }

    // force_op_wr / force_op_rd. OFF por defecto.
    constraint c_force_op_wr { is_write == 1'b1; }
    constraint c_force_op_rd { is_write == 1'b0; }

    // force_addr_lo: concentra accesos en primeras palabras
    constraint c_force_addr_lo {
        soft addr < (N_BANKS * WORD_BYTES * 16);
    }

    // force_wstrb_partial: patrones byte-enable
    constraint c_force_wstrb_partial {
        is_write == 1'b1 -> wstrb inside {4'b0001, 4'b0010, 4'b0100, 4'b1000,
                                          4'b0011, 4'b1100, 4'b1010, 4'b0101,
                                          4'b0111, 4'b1110};
    }

    constraint c_delay_cycles {delay_cycles in [0:10];}


    // ========================================================
    // set_phase — helper para preparar el item por fase
    // ========================================================
    function void set_phase(string phase);
        // Reset: defaults ON, todos los force_* OFF
        c_default_addr_valid.constraint_mode(1);
        c_default_wstrb.constraint_mode(1);
        c_force_invalid.constraint_mode(0);
        c_force_bank.constraint_mode(0);
        c_force_op_wr.constraint_mode(0);
        c_force_op_rd.constraint_mode(0);
        c_force_addr_lo.constraint_mode(0);
        c_force_wstrb_partial.constraint_mode(0);

        case (phase)
            "GENERAL":      /* defaults */ ;
            "SINGLE_BANK":  c_force_bank.constraint_mode(1);
            "CONFLICT":     c_force_addr_lo.constraint_mode(1);
            "INVALID_ADDR": begin
                c_default_addr_valid.constraint_mode(0);
                c_force_invalid.constraint_mode(1);
            end
            "WSTRB_STRESS": begin
                c_force_op_wr.constraint_mode(1);
                c_default_wstrb.constraint_mode(0);
                c_force_wstrb_partial.constraint_mode(1);
            end
            "WRITES_ONLY":  c_force_op_wr.constraint_mode(1);
            "READS_ONLY":   c_force_op_rd.constraint_mode(1);
            default: `uvm_warning("SEQ_ITEM",
                $sformatf("Unknown phase '%s' — using defaults", phase))
        endcase
    endfunction

endclass
