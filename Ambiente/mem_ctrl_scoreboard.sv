// ============================================================
//  File    : mem_ctrl_scoreboard.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Scoreboard (hazard-aware + CSV dump)
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
//    - Tolerante a SLVERR: solo log informativo, no fail.
//    - Tolerante a addr inválidas: no crean entrada en exp_queue.
//
//  HAZARD-AWARE (v2):
//    AXI4-Lite no garantiza coherencia read-after-write entre
//    transacciones concurrentes a la misma dirección; el ordering
//    es responsabilidad del master (debe esperar B antes del AR
//    dependiente). En modo paralelo (carga mixta) puede ocurrir
//    que un write a la misma dirección llegue entre el AR fire y
//    el R fire. El read entonces puede devolver legítimamente:
//        · el valor ANTERIOR  (snapshot en AR fire), o
//        · el valor POSTERIOR (ref_mem actual en R fire).
//    El scoreboard tolera AMBOS valores cuando la posición fue
//    modificada (hazard) y los contabiliza como hazards tolerados.
//    Si el dato no coincide con NINGUNO de los dos, es corrupción
//    real y se reporta como error.
//
//  CSV dump (v2): metrics_summary.csv, latencies_detail.csv,
//    bank_utilization.csv (config vía uvm_config_db).
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
    // exp_data : snapshot del valor en AR fire (valor ANTERIOR)
    // bank/off : posición para releer ref_mem en R fire (valor POSTERIOR)
    typedef struct {
        int unsigned     txn_id;
        bit [DATA_W-1:0] exp_data;
        int              bank;
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
    int unsigned hazard_tolerated;      // RAW hazards tolerados (v2)
    real         sum_lat_wr_ns;
    real         sum_lat_rd_ns;

    // ── Métricas extendidas (TFG report) ─────────────────
    real         min_lat_wr_ns;
    real         max_lat_wr_ns;
    real         min_lat_rd_ns;
    real         max_lat_rd_ns;

    // Latencias detalladas por transacción (para CSV)
    typedef struct {
        int unsigned txn_id;
        string       op_type;   // "WR" o "RD"
        int          bank;
        real         latency_ns;
    } lat_entry_t;
    lat_entry_t lat_history[$];

    // Completes por banco (proxy de utilización)
    int unsigned bank_completes_wr [N_BANKS];
    int unsigned bank_completes_rd [N_BANKS];

    // Configuración runtime desde uvm_config_db
    string       csv_dir;
    int          b_bp_value;
    int          r_bp_value;
    string       run_label;

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

        // Inicializar min/max a valores seguros
        min_lat_wr_ns = 1.0e9;
        max_lat_wr_ns = 0.0;
        min_lat_rd_ns = 1.0e9;
        max_lat_rd_ns = 0.0;

        // Inicializar contadores por banco
        foreach (bank_completes_wr[b]) begin
            bank_completes_wr[b] = 0;
            bank_completes_rd[b] = 0;
        end

        // Levantar config desde uvm_config_db (defaults si no se setea)
        if (!uvm_config_db#(string)::get(this, "", "csv_dir", csv_dir))
            csv_dir = ".";
        if (!uvm_config_db#(int)::get(this, "", "b_bp_value", b_bp_value))
            b_bp_value = 0;
        if (!uvm_config_db#(int)::get(this, "", "r_bp_value", r_bp_value))
            r_bp_value = 0;
        if (!uvm_config_db#(string)::get(this, "", "run_label", run_label))
            run_label = "mem_full_test";

        `uvm_info("SB_CSV", $sformatf(
            "CSV config: dir=%s label=%s b_bp=%0d r_bp=%0d",
            csv_dir, run_label, b_bp_value, r_bp_value), UVM_LOW)
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
        real        lat_ns;
        lat_entry_t lat_entry;
        int         bank;
        bit         valid;
        b_count++;
        lat_ns = real'(item.t_resp_fire - item.t_req_fire);
        sum_lat_wr_ns += lat_ns;

        // Min/max
        if (lat_ns < min_lat_wr_ns) min_lat_wr_ns = lat_ns;
        if (lat_ns > max_lat_wr_ns) max_lat_wr_ns = lat_ns;

        // Bank tracking + push a historia (solo si addr válida)
        valid = is_addr_valid(item.addr, DATA_W, N_BANKS, BANK_SIZE_BYTES);
        if (valid) begin
            bank = get_bank(item.addr, DATA_W, N_BANKS);
            bank_completes_wr[bank]++;
        end

        lat_entry.txn_id     = item.txn_id;
        lat_entry.op_type    = "WR";
        lat_entry.bank       = valid ? get_bank(item.addr, DATA_W, N_BANKS) : -1;
        lat_entry.latency_ns = lat_ns;
        lat_history.push_back(lat_entry);

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
        // exp_data = snapshot del valor ANTERIOR (en AR fire)
        // bank/offset = referencia para releer ref_mem en R fire
        exp_entry.txn_id   = item.txn_id;
        exp_entry.exp_data = ref_mem[bank][offset];
        exp_entry.bank     = bank;
        exp_entry.offset   = offset;
        exp_queue[bank].push_back(exp_entry);

        // Push a global AR queue (order checker)
        ar_entry.txn_id = item.txn_id;
        ar_entry.bank   = bank;
        ar_global_queue.push_back(ar_entry);
    endfunction


    // ========================================================
    // write_r — R fire (order check + data check hazard-aware)
    // ========================================================
    function void write_r(item_t item);
        real             lat_ns;
        lat_entry_t      lat_entry;
        ar_entry_t       head_ar;
        int              match_idx;
        exp_entry_t      matched_exp;
        bit [DATA_W-1:0] exp_old;
        bit [DATA_W-1:0] exp_new;
        r_count++;
        lat_ns = real'(item.t_resp_fire - item.t_req_fire);
        sum_lat_rd_ns += lat_ns;

        // Min/max
        if (lat_ns < min_lat_rd_ns) min_lat_rd_ns = lat_ns;
        if (lat_ns > max_lat_rd_ns) max_lat_rd_ns = lat_ns;

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

        // ── HAZARD-AWARE DATA CHECK ──
        // exp_old : valor en el snapshot del AR fire (valor anterior)
        // exp_new : valor actual de ref_mem (posterior, si hubo write
        //           concurrente a la misma posición entre AR y R)
        exp_old = matched_exp.exp_data;
        exp_new = ref_mem[matched_exp.bank][matched_exp.offset];

        if (item.data === exp_old) begin
            // Caso normal: el read vio el valor consistente con el snapshot.
        end
        else if (item.data === exp_new) begin
            // El read vio el valor POSTERIOR: un write concurrente a la
            // misma dirección llegó entre AR y R, y el DUT (arbitraje)
            // procesó el write antes del read. Válido según AXI4-Lite.
            hazard_tolerated++;
            `uvm_info("SB_DATA", $sformatf(
                "RAW hazard tolerated txn_id=%0d bank=%0d offset=0x%0h: read saw post-write value 0x%0h (snapshot was 0x%0h)",
                head_ar.txn_id, head_ar.bank, matched_exp.offset,
                exp_new, exp_old), UVM_MEDIUM)
        end
        else begin
            // No coincide ni con anterior ni con posterior: corrupción real.
            data_mismatches++;
            `uvm_error("SB_DATA", $sformatf(
                "DATA MISMATCH (real) txn_id=%0d bank=%0d offset=0x%0h: exp_old=0x%0h exp_new=0x%0h item.data=0x%0h",
                head_ar.txn_id, head_ar.bank, matched_exp.offset,
                exp_old, exp_new, item.data))
        end

        // ── Order check (reorder intra-banco) ──
        if (match_idx != 0) begin
            order_violations++;
            `uvm_error("SB_ORDER", $sformatf(
                "Reorder detected: txn_id=%0d matched at idx=%0d (expected 0)",
                head_ar.txn_id, match_idx))
        end

        // ── Tracking de latencia y banco ──
        bank_completes_rd[head_ar.bank]++;
        lat_entry.txn_id     = item.txn_id;
        lat_entry.op_type    = "RD";
        lat_entry.bank       = head_ar.bank;
        lat_entry.latency_ns = lat_ns;
        lat_history.push_back(lat_entry);
    endfunction


    // ========================================================
    // report_phase — resumen final + métricas + dump CSV
    // ========================================================
    function void report_phase(uvm_phase phase);
        real avg_lat_wr;
        real avg_lat_rd;
        real throughput_wr;
        real throughput_rd;
        real sim_time_ns;
        super.report_phase(phase);

        avg_lat_wr   = (b_count > 0) ? (sum_lat_wr_ns / b_count) : 0.0;
        avg_lat_rd   = (r_count > 0) ? (sum_lat_rd_ns / r_count) : 0.0;
        sim_time_ns  = real'($time);
        throughput_wr = (sim_time_ns > 0) ? (real'(b_count) * 1000.0 / sim_time_ns) : 0.0;
        throughput_rd = (sim_time_ns > 0) ? (real'(r_count) * 1000.0 / sim_time_ns) : 0.0;

        if (b_count == 0) min_lat_wr_ns = 0.0;
        if (r_count == 0) min_lat_rd_ns = 0.0;

        `uvm_info("SB_REPORT", "===== MEM_CTRL SCOREBOARD REPORT =====",  UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Writes issued:        %0d", write_count),         UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Reads issued:         %0d", read_count),          UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("B  responses:         %0d", b_count),             UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("R  responses:         %0d", r_count),             UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Invalid WR addr (M5): %0d", addr_invalid_writes), UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Invalid RD addr (M5): %0d", addr_invalid_reads),  UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Data mismatches:      %0d", data_mismatches),     UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("RAW hazards tolerated:%0d", hazard_tolerated),    UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Order violations:     %0d", order_violations),    UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("SLVERR observed:      %0d", slverr_count),        UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Avg WR latency (ns):  %0.2f", avg_lat_wr),        UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Min WR latency (ns):  %0.2f", min_lat_wr_ns),     UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Max WR latency (ns):  %0.2f", max_lat_wr_ns),     UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Avg RD latency (ns):  %0.2f", avg_lat_rd),        UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Min RD latency (ns):  %0.2f", min_lat_rd_ns),     UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Max RD latency (ns):  %0.2f", max_lat_rd_ns),     UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Throughput WR (op/us):%0.3f", throughput_wr),     UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Throughput RD (op/us):%0.3f", throughput_rd),     UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("Sim time (ns):        %0.0f", sim_time_ns),       UVM_NONE)
        `uvm_info("SB_REPORT", $sformatf("AR queue remaining:   %0d", ar_global_queue.size()), UVM_NONE)
        for (int b = 0; b < N_BANKS; b++) begin
            `uvm_info("SB_REPORT", $sformatf("Bank %0d: WR=%0d RD=%0d",
                b, bank_completes_wr[b], bank_completes_rd[b]), UVM_NONE)
        end

        // ── Dump a CSVs ──
        dump_summary_csv(avg_lat_wr, avg_lat_rd, throughput_wr, throughput_rd, sim_time_ns);
        dump_latencies_csv();
        dump_bank_util_csv(sim_time_ns);
    endfunction


    // ========================================================
    // Funciones de dump CSV
    // ========================================================
    function void dump_summary_csv(real avg_lat_wr, real avg_lat_rd,
                                    real throughput_wr, real throughput_rd,
                                    real sim_time_ns);
        string  filepath;
        integer fd;
        integer fd_check;

        filepath = {csv_dir, "/metrics_summary.csv"};

        fd_check = $fopen(filepath, "r");
        if (fd_check == 0) begin
            fd = $fopen(filepath, "w");
            $fwrite(fd, "test_name,b_bp,r_bp,writes,reads,b_resp,r_resp,");
            $fwrite(fd, "data_mismatches,hazards_tolerated,order_violations,slverr,");
            $fwrite(fd, "invalid_wr,invalid_rd,");
            $fwrite(fd, "avg_lat_wr,min_lat_wr,max_lat_wr,");
            $fwrite(fd, "avg_lat_rd,min_lat_rd,max_lat_rd,");
            $fwrite(fd, "throughput_wr,throughput_rd,sim_time_ns\n");
        end else begin
            $fclose(fd_check);
            fd = $fopen(filepath, "a");
        end

        $fwrite(fd, "%s,%0d,%0d,", run_label, b_bp_value, r_bp_value);
        $fwrite(fd, "%0d,%0d,%0d,%0d,", write_count, read_count, b_count, r_count);
        $fwrite(fd, "%0d,%0d,%0d,%0d,", data_mismatches, hazard_tolerated, order_violations, slverr_count);
        $fwrite(fd, "%0d,%0d,", addr_invalid_writes, addr_invalid_reads);
        $fwrite(fd, "%0.2f,%0.2f,%0.2f,", avg_lat_wr, min_lat_wr_ns, max_lat_wr_ns);
        $fwrite(fd, "%0.2f,%0.2f,%0.2f,", avg_lat_rd, min_lat_rd_ns, max_lat_rd_ns);
        $fwrite(fd, "%0.3f,%0.3f,%0.0f\n", throughput_wr, throughput_rd, sim_time_ns);
        $fclose(fd);
        `uvm_info("SB_CSV", $sformatf("Summary written to %s", filepath), UVM_LOW)
    endfunction

    function void dump_latencies_csv();
        string  filepath;
        integer fd;
        integer fd_check;

        filepath = {csv_dir, "/latencies_detail.csv"};

        fd_check = $fopen(filepath, "r");
        if (fd_check == 0) begin
            fd = $fopen(filepath, "w");
            $fwrite(fd, "test_name,b_bp,r_bp,txn_id,op_type,bank,latency_ns\n");
        end else begin
            $fclose(fd_check);
            fd = $fopen(filepath, "a");
        end

        foreach (lat_history[i]) begin
            $fwrite(fd, "%s,%0d,%0d,%0d,%s,%0d,%0.2f\n",
                run_label, b_bp_value, r_bp_value,
                lat_history[i].txn_id, lat_history[i].op_type,
                lat_history[i].bank, lat_history[i].latency_ns);
        end
        $fclose(fd);
        `uvm_info("SB_CSV", $sformatf("Latencies written to %s (%0d entries)",
            filepath, lat_history.size()), UVM_LOW)
    endfunction

    function void dump_bank_util_csv(real sim_time_ns);
        string  filepath;
        integer fd;
        integer fd_check;
        int     total_completes;
        real    util_pct;

        filepath = {csv_dir, "/bank_utilization.csv"};

        fd_check = $fopen(filepath, "r");
        if (fd_check == 0) begin
            fd = $fopen(filepath, "w");
            $fwrite(fd, "test_name,b_bp,r_bp,bank_id,");
            $fwrite(fd, "writes_completed,reads_completed,total_completes,");
            $fwrite(fd, "utilization_proxy_pct,sim_time_ns\n");
        end else begin
            $fclose(fd_check);
            fd = $fopen(filepath, "a");
        end

        for (int b = 0; b < N_BANKS; b++) begin
            total_completes = bank_completes_wr[b] + bank_completes_rd[b];
            util_pct = (write_count + read_count > 0) ?
                        (real'(total_completes) * 100.0 / real'(write_count + read_count)) : 0.0;
            $fwrite(fd, "%s,%0d,%0d,%0d,%0d,%0d,%0d,%0.2f,%0.0f\n",
                run_label, b_bp_value, r_bp_value, b,
                bank_completes_wr[b], bank_completes_rd[b], total_completes,
                util_pct, sim_time_ns);
        end
        $fclose(fd);
        `uvm_info("SB_CSV", $sformatf("Bank util written to %s", filepath), UVM_LOW)
    endfunction

endclass
