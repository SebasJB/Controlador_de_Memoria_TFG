// ============================================================
//  File    : mem_ctrl_coverage.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Coverage subscriber
//
//  Tres covergroups:
//    cg_transaction — sample por seq_item (transaction-driven)
//    cg_arbitration — sample por ciclo (cycle-driven, probe)
//    cg_bank_state  — sample por ciclo (cycle-driven, probe)
// ============================================================

`uvm_analysis_imp_decl(_cov_wr)
`uvm_analysis_imp_decl(_cov_ar)

class mem_ctrl_coverage #(
    parameter int ADDR_W           = 32,
    parameter int DATA_W           = 32,
    parameter int N_BANKS          = 4,
    parameter int BANK_SIZE_BYTES  = 1024,
    parameter int WR_FIFO_DEPTH    = 8,
    parameter int RD_FIFO_DEPTH    = 8
) extends uvm_component;

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS) item_t;

    uvm_analysis_imp_cov_wr #(item_t, mem_ctrl_coverage #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES, WR_FIFO_DEPTH, RD_FIFO_DEPTH)) ap_wr_imp;
    uvm_analysis_imp_cov_ar #(item_t, mem_ctrl_coverage #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES, WR_FIFO_DEPTH, RD_FIFO_DEPTH)) ap_ar_imp;

    virtual scheduler_probe_if            sched_vif;
    virtual bank_probe_if #(N_BANKS)      bank_vif;
    virtual fifo_probe_if #(WR_FIFO_DEPTH, RD_FIFO_DEPTH) fifo_vif;

    localparam int BANK_WORDS = BANK_SIZE_BYTES / (DATA_W / 8);

    // ── Sample vars (necesario para que covergroups las vean) ─
    bit              cov_is_write;
    int              cov_bank;
    int              cov_offset;
    bit [DATA_W/8-1:0] cov_wstrb;

    int              cov_busy_count;

    // ========================================================
    // cg_transaction — distribución del estímulo
    // ========================================================
    covergroup cg_transaction;
        option.per_instance = 1;

        cp_op: coverpoint cov_is_write {
            bins write = {1'b1};
            bins read  = {1'b0};
        }
        cp_bank: coverpoint cov_bank {
            bins b[] = {[0:N_BANKS-1]};
        }
        cp_region: coverpoint cov_offset {
            bins low      = {[0                  : BANK_WORDS/4 - 1]};
            bins mid_low  = {[BANK_WORDS/4       : BANK_WORDS/2 - 1]};
            bins mid_high = {[BANK_WORDS/2       : 3*BANK_WORDS/4 - 1]};
            bins high     = {[3*BANK_WORDS/4     : BANK_WORDS - 1]};
            bins invalid  = {[BANK_WORDS         : BANK_WORDS*8]};
        }
        cp_wstrb: coverpoint cov_wstrb {
            bins byte0       = {4'b0001};
            bins byte1       = {4'b0010};
            bins byte2       = {4'b0100};
            bins byte3       = {4'b1000};
            bins half_low    = {4'b0011};
            bins half_high   = {4'b1100};
            bins half_inter  = {4'b1010, 4'b0101};
            bins three_bytes = {4'b0111, 4'b1110, 4'b1101, 4'b1011};
            bins full_word   = {4'b1111};
            bins no_bytes    = {4'b0000};
            // Solo activo cuando is_write — los reads ignoran wstrb
        }

        cr_op_bank: cross cp_op, cp_bank;
    endgroup

    // ========================================================
    // cg_arbitration — comportamiento del scheduler
    // ========================================================
    bit cov_grant_wr, cov_grant_rd;
    bit cov_wr_pndng, cov_rd_pndng;
    bit cov_same_bank;
    bit cov_wr_resp_full;
    bit cov_rob_tag_free;
    bit cov_wr_discard, cov_rd_discard;

    covergroup cg_arbitration;
        option.per_instance = 1;

        cp_grant: coverpoint {cov_grant_wr, cov_grant_rd} {
            bins idle    = {2'b00};
            bins wr_only = {2'b10};
            bins rd_only = {2'b01};
            bins both    = {2'b11};
        }
        cp_conflict: coverpoint {cov_wr_pndng, cov_rd_pndng, cov_same_bank} {
            bins no_pending  = {3'b000, 3'b001};
            bins only_wr     = {3'b100, 3'b101};
            bins only_rd     = {3'b010, 3'b011};
            bins both_diff   = {3'b110};
            bins both_same   = {3'b111};
        }
        cp_bp_b:   coverpoint cov_wr_resp_full;
        cp_bp_rob: coverpoint cov_rob_tag_free {
            bins free = {1'b1};
            bins busy = {1'b0};
        }
        cp_discard: coverpoint {cov_wr_discard, cov_rd_discard} {
            bins none       = {2'b00};
            bins wr_discard = {2'b10};
            bins rd_discard = {2'b01};
            bins both       = {2'b11};
        }

        cr_grant_conflict: cross cp_grant, cp_conflict;
    endgroup

    // ========================================================
    // cg_bank_state — paralelismo y FIFOs
    // ========================================================
    int unsigned cov_wr_fifo_occ;
    int unsigned cov_rd_fifo_occ;

    covergroup cg_bank_state;
        option.per_instance = 1;

        cp_busy_count: coverpoint cov_busy_count {
            bins zero  = {0};
            bins one   = {1};
            bins two   = {2};
            bins three = {3};
            bins all   = {N_BANKS};
        }
        cp_wr_fifo: coverpoint cov_wr_fifo_occ {
            bins empty   = {0};
            bins q1      = {[1                : WR_FIFO_DEPTH/4]};
            bins q2      = {[WR_FIFO_DEPTH/4+1: WR_FIFO_DEPTH/2]};
            bins q3      = {[WR_FIFO_DEPTH/2+1: 3*WR_FIFO_DEPTH/4]};
            bins full    = {[3*WR_FIFO_DEPTH/4+1: WR_FIFO_DEPTH]};
        }
        cp_rd_fifo: coverpoint cov_rd_fifo_occ {
            bins empty   = {0};
            bins q1      = {[1                : RD_FIFO_DEPTH/4]};
            bins q2      = {[RD_FIFO_DEPTH/4+1: RD_FIFO_DEPTH/2]};
            bins q3      = {[RD_FIFO_DEPTH/2+1: 3*RD_FIFO_DEPTH/4]};
            bins full    = {[3*RD_FIFO_DEPTH/4+1: RD_FIFO_DEPTH]};
        }
    endgroup

    `uvm_component_param_utils(mem_ctrl_coverage #(ADDR_W, DATA_W, N_BANKS, BANK_SIZE_BYTES, WR_FIFO_DEPTH, RD_FIFO_DEPTH))

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap_wr_imp = new("ap_wr_imp", this);
        ap_ar_imp = new("ap_ar_imp", this);
        cg_transaction = new();
        cg_arbitration = new();
        cg_bank_state  = new();
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual scheduler_probe_if)::get(
                this, "", "sched_vif", sched_vif))
            `uvm_warning("COV", "sched_vif not set — cg_arbitration disabled")
        if (!uvm_config_db#(virtual bank_probe_if #(N_BANKS))::get(
                this, "", "bank_vif", bank_vif))
            `uvm_warning("COV", "bank_vif not set — cg_bank_state partial")
        if (!uvm_config_db#(virtual fifo_probe_if #(WR_FIFO_DEPTH, RD_FIFO_DEPTH))::get(
                this, "", "fifo_vif", fifo_vif))
            `uvm_warning("COV", "fifo_vif not set — cg_bank_state partial")
    endfunction

    // ── Sample por transaction (WR) ──────────────────────
    function void write_cov_wr(item_t item);
        cov_is_write = 1'b1;
        cov_bank     = get_bank  (item.addr, DATA_W, N_BANKS);
        cov_offset   = get_offset(item.addr, DATA_W, N_BANKS, BANK_SIZE_BYTES);
        cov_wstrb    = item.wstrb;
        cg_transaction.sample();
    endfunction

    // ── Sample por transaction (AR) ──────────────────────
    function void write_cov_ar(item_t item);
        cov_is_write = 1'b0;
        cov_bank     = get_bank  (item.addr, DATA_W, N_BANKS);
        cov_offset   = get_offset(item.addr, DATA_W, N_BANKS, BANK_SIZE_BYTES);
        cov_wstrb    = '0;
        cg_transaction.sample();
    endfunction

    // ── Sample por ciclo (probes) ────────────────────────
    task run_phase(uvm_phase phase);
        if (sched_vif == null) return;
        wait (sched_vif.rst_n === 1'b1);
        forever begin
            @(posedge sched_vif.clk);
            // Sample arbitration
            cov_grant_wr     = sched_vif.grant_wr;
            cov_grant_rd     = sched_vif.grant_rd;
            cov_wr_pndng     = sched_vif.wr_req_pndng;
            cov_rd_pndng     = sched_vif.rd_req_pndng;
            cov_same_bank    = sched_vif.same_bank;
            cov_wr_resp_full = sched_vif.wr_resp_full;
            cov_rob_tag_free = sched_vif.rob_tag_free;
            cov_wr_discard   = sched_vif.wr_discard;
            cov_rd_discard   = sched_vif.rd_discard;
            cg_arbitration.sample();

            // Sample bank state
            if (bank_vif != null) begin
                int busy_count = 0;
                for (int b = 0; b < N_BANKS; b++)
                    if (bank_vif.bank_busy[b]) busy_count++;
                cov_busy_count = busy_count;
            end
            if (fifo_vif != null) begin
                cov_wr_fifo_occ = fifo_vif.wr_req_count;
                cov_rd_fifo_occ = fifo_vif.rd_req_count;
            end
            cg_bank_state.sample();
        end
    endtask

    function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("COV_REPORT", $sformatf(
            "cg_transaction: %0.2f%% | cg_arbitration: %0.2f%% | cg_bank_state: %0.2f%%",
            cg_transaction.get_coverage(),
            cg_arbitration.get_coverage(),
            cg_bank_state.get_coverage()), UVM_NONE)
    endfunction

endclass
