// ============================================================
//  File    : mem_ctrl_scoreboard.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Scoreboard
//
//  Diseño:
//    - 4 analysis_imps: ap_wr, ap_b, ap_ar, ap_r
//    - Data checker:
//        · ref_mem[bank][offset] shadow memory (byte-arrays)
//        · exp_queue[bank] (SV native queue) por banco
//        · Update en AW+W fire (no en B)
//        · Snapshot en AR fire (no en R)
//    - Order checker:
//        · ar_global_queue: cola única ordenada de AR fires
//        · Pop en R fire; report violation si vacía
//    - Coordinación: en R fire, order_checker reporta {txn_id,bank}
//      al data_checker para localizar la entrada correcta.
//    - Tolerante a SLVERR: solo log informativo, no fail.
//    - Tolerante a addr inválidas: no crean entrada en
//      exp_queue (matchean comportamiento del DUT que descarta).
// ============================================================

`uvm_analysis_imp_decl(_wr)
`uvm_analysis_imp_decl(_b)
`uvm_analysis_imp_decl(_ar)
`uvm_analysis_imp_decl(_r)

class mem_ctrl_scoreboard #(
    parameter int ADDR_W           = 32,
    parameter int DATA_W           = 32,
    parameter int N_BANKS          = 4,
    parameter int BANK_SIZE_BYTES  = 1024
) extends uvm_component;

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS) item_t;

    // ── Analysis imps ────────────────────────────────────
    uvm_analysis_imp_wr #(item_t, mem_ctrl_scoreboard #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES)) ap_wr_imp;
    uvm_analysis_imp_b  #(item_t, mem_ctrl_scoreboard #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES)) ap_b_imp;
    uvm_analysis_imp_ar #(item_t, mem_ctrl_scoreboard #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES)) ap_ar_imp;
    uvm_analysis_imp_r  #(item_t, mem_ctrl_scoreboard #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES)) ap_r_imp;

    // ── Shadow memory ────────────────────────────────────
    localparam int BANK_WORDS = BANK_SIZE_BYTES / (DATA_W / 8);
    bit [DATA_W-1:0] ref_mem [N_BANKS][BANK_WORDS];

    // ── Expected queues (data checker) ───────────────────
    typedef struct {
        int unsigned     txn_id;
        bit [DATA_W-1:0] exp_data;
        int              offset;
    } exp_entry_t;
    exp_entry_t exp_queue [N_BANKS][$];

    // ── AR global queue (order checker) ──────────────────
    typedef struct {
        int unsigned txn_id;
        int          bank;
    } ar_entry_t;
    ar_entry_t ar_global_queue[$];

    // ── Métricas y contadores ────────────────────────────
    int unsigned write_count;
    int unsigned read_count;
    int unsigned b_count;
    int unsigned r_count;
    int unsigned data_mismatches;
    int unsigned order_violations;
    int unsigned addr_invalid_writes;   // M5 esperado
    int unsigned addr_invalid_reads;    // M5 esperado
    int unsigned slverr_count;
    time         sum_lat_wr;
    time         sum_lat_rd;

    `uvm_component_param_utils(mem_ctrl_scoreboard #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES))

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap_wr_imp = new("ap_wr_imp", this);
        ap_b_imp  = new("ap_b_imp",  this);
        ap_ar_imp = new("ap_ar_imp", this);
        ap_r_imp  = new("ap_r_imp",  this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        // Inicializar shadow memory a 0
        foreach (ref_mem[b,o])
            ref_mem[b][o] = '0;
    endfunction


    // ========================================================
    // write_wr — AW+W fire (data checker, actualiza shadow)
    // ========================================================
    function void write_wr(item_t item);
        int  bank, offset;
        bit  valid;
        write_count++;

        valid = is_addr_valid(item.addr, DATA_W, N_BANKS, BANK_SIZE_BYTES);
        if (!valid) begin
            addr_invalid_writes++;
            return;   // DUT descartará, no actualizamos shadow
        end

        bank   = get_bank  (item.addr, DATA_W, N_BANKS);
        offset = get_offset(item.addr, DATA_W, N_BANKS, BANK_SIZE_BYTES);

        // Aplicar wstrb byte-a-byte
        for (int byb = 0; byb < DATA_W/8; byb++) begin
            if (item.wstrb[byb])
                ref_mem[bank][offset][byb*8 +: 8] = item.data[byb*8 +: 8];
        end
    endfunction


    // ========================================================
    // write_b — B fire (verifica BRESP, métrica latencia)
    // ========================================================
    function void write_b(item_t item);
        b_count++;
        sum_lat_wr += (item.t_resp_fire - item.t_req_fire);

        case (item.resp)
            AXI_RESP_OKAY:   /* OK */ ;
            AXI_RESP_SLVERR: begin
                slverr_count++;
                `uvm_info("SB_B", $sformatf(
                    "SLVERR observed (tolerated) txn_id=%0d addr=0x%0h",
                    item.txn_id, item.addr), UVM_LOW)
            end
            default: begin
                `uvm_error("SB_B", $sformatf(
                    "Unexpected BRESP=%0b txn_id=%0d",
                    item.resp, item.txn_id))
            end
        endcase
    endfunction


    // ========================================================
    // write_ar — AR fire (snapshot exp_data, push a queues)
    // ========================================================
    function void write_ar(item_t item);
        int           bank, offset;
        bit           valid;
        exp_entry_t   exp_entry;
        ar_entry_t    ar_entry;
        read_count++;

        valid = is_addr_valid(item.addr, DATA_W, N_BANKS, BANK_SIZE_BYTES);
        if (!valid) begin
            addr_invalid_reads++;
            return;   // DUT descartará, no esperamos R
        end

        bank   = get_bank  (item.addr, DATA_W, N_BANKS);
        offset = get_offset(item.addr, DATA_W, N_BANKS, BANK_SIZE_BYTES);

        // Push a expected queue del banco
        exp_entry.txn_id   = item.txn_id;
        exp_entry.exp_data = ref_mem[bank][offset];
        exp_entry.offset   = offset;
        exp_queue[bank].push_back(exp_entry);

        // Push a global AR queue (order checker)
        ar_entry.txn_id = item.txn_id;
        ar_entry.bank   = bank;
        ar_global_queue.push_back(ar_entry);
    endfunction


    // ========================================================
    // write_r — R fire (order check + data check)
    // ========================================================
    function void write_r(item_t item);
        ar_entry_t  head_ar;
        int         match_idx;
        exp_entry_t matched_exp;
        r_count++;
        sum_lat_rd += (item.t_resp_fire - item.t_req_fire);

        // Tolerancia SLVERR
        if (item.resp == AXI_RESP_SLVERR) begin
            slverr_count++;
            `uvm_info("SB_R", $sformatf(
                "SLVERR observed (tolerated) txn_id=%0d",
                item.txn_id), UVM_LOW)
        end else if (item.resp != AXI_RESP_OKAY) begin
            `uvm_error("SB_R", $sformatf(
                "Unexpected RRESP=%0b txn_id=%0d",
                item.resp, item.txn_id))
        end

        // ── Order check ──
        if (ar_global_queue.size() == 0) begin
            `uvm_error("SB_R", $sformatf(
                "R fire without pending AR (txn_id_observed=%0d)",
                item.txn_id))
            return;
        end
        head_ar = ar_global_queue.pop_front();

        // ── Data check ──
        // Buscamos en exp_queue[bank] la entrada con txn_id matching.
        // En operación normal será la del frente, pero buscamos por
        // si hubiese reorder (lo reportaría como mismatch).
        match_idx = -1;
        foreach (exp_queue[head_ar.bank][i]) begin
            if (exp_queue[head_ar.bank][i].txn_id == head_ar.txn_id) begin
                match_idx = i;
                break;
            end
        end

        if (match_idx == -1) begin
            `uvm_error("SB_DATA", $sformatf(
                "No expected entry for txn_id=%0d in bank=%0d",
                head_ar.txn_id, head_ar.bank))
            return;
        end

        matched_exp = exp_queue[head_ar.bank][match_idx];
        exp_queue[head_ar.bank].delete(match_idx);

        if (item.data !== matched_exp.exp_data) begin
            data_mismatches++;
            `MEM_CTRL_REPORT_MISMATCH("SB_DATA",
                head_ar.txn_id, head_ar.bank, matched_exp.offset,
                matched_exp.exp_data, item.data)
        end

        // ROB ordering: si el txn_id ganador no fue el del frente
        // de exp_queue[bank], es un reorder. Sería raro porque
        // las SVAs del ROB lo previenen, pero lo reportamos.
        if (match_idx != 0) begin
            order_violations++;
            `uvm_error("SB_ORDER", $sformatf(
                "Reorder detected: txn_id=%0d matched at idx=%0d (expected 0)",
                head_ar.txn_id, match_idx))
        end
    endfunction


    // ========================================================
    // report_phase — resumen final + métricas M1/M5
    // ========================================================
    function void report_phase(uvm_phase phase);
        real avg_lat_wr;
        real avg_lat_rd;
        super.report_phase(phase);

        avg_lat_wr = (b_count > 0) ? (real'(sum_lat_wr) / b_count) : 0.0;
        avg_lat_rd = (r_count > 0) ? (real'(sum_lat_rd) / r_count) : 0.0;

        `uvm_info("SB_REPORT", "===== MEM_CTRL SCOREBOARD REPORT =====",  UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Writes issued:        %0d", write_count),         UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Reads issued:         %0d", read_count),          UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("B  responses:         %0d", b_count),             UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("R  responses:         %0d", r_count),             UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Invalid WR addr (M5): %0d", addr_invalid_writes), UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Invalid RD addr (M5): %0d", addr_invalid_reads),  UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Data mismatches:      %0d", data_mismatches),     UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Order violations:     %0d", order_violations),    UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("SLVERR observed:      %0d", slverr_count),        UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Avg WR latency (ns):  %0.2f", avg_lat_wr),        UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Avg RD latency (ns):  %0.2f", avg_lat_rd),        UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("AR queue remaining:   %0d", ar_global_queue.size()), UVM_NONE)
    endfunction

endclass
