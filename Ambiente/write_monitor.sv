// ============================================================
//  File    : write_monitor.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Write Monitor — observa AW, W, B (passive)
//
//  Analysis ports:
//    - ap_wr : dispara en AW+W fire (ambos handshakes completos)
//    - ap_b  : dispara en B fire
//
//  Implementación:
//    - Tres hilos paralelos observan los handshakes AW, W, B.
//    - Cuando AW fire ocurre, captura {addr}. Cuando W fire
//      ocurre, captura {data, wstrb}. Cuando ambos ya fueron
//      capturados, emite el item por ap_wr.
//    - El monitor mantiene un pequeño buffer de items en vuelo
//      para correlacionar AW con su B correspondiente vía
//      orden de llegada (FIFO).
// ============================================================

class write_monitor #(
    parameter int ADDR_W  = 32,
    parameter int DATA_W  = 32,
    parameter int N_BANKS = 4
) extends uvm_monitor;

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS) item_t;

    virtual mem_bank_interface #(ADDR_W, DATA_W) vif;

    uvm_analysis_port #(item_t) ap_wr;
    uvm_analysis_port #(item_t) ap_b;

    // Cola FIFO de items pendientes de B
    item_t pending_b[$];

    `uvm_component_param_utils(write_monitor #(ADDR_W, DATA_W, N_BANKS))

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap_wr = new("ap_wr", this);
        ap_b  = new("ap_b",  this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual mem_bank_interface #(ADDR_W, DATA_W))::get(
                this, "", "axi_vif", vif))
            `uvm_fatal("WR_MON", "axi_vif not set")
    endfunction

    // ── Run phase ────────────────────────────────────────
    task run_phase(uvm_phase phase);
        wait (vif.rst_n === 1'b1);
        @(vif.monitor_cb);
        fork
            monitor_aw_w();
            monitor_b();
        join
    endtask

    // ── Observa AW+W fire en paralelo ────────────────────
    // AW y W pueden hacer fire en distinto ciclo; correlaciona-
    // mos por orden (FIFO): el primer AW va con el primer W.
    task monitor_aw_w();
        bit [ADDR_W-1:0]    aw_q[$];
        bit [DATA_W-1:0]    wd_q[$];
        bit [DATA_W/8-1:0]  ws_q[$];

        fork
            // Captura AW fire
            forever begin
                @(vif.monitor_cb);
                if (vif.monitor_cb.awvalid === 1'b1 &&
                    vif.monitor_cb.awready === 1'b1) begin
                    aw_q.push_back(vif.monitor_cb.awaddr);
                end
            end
            // Captura W fire
            forever begin
                @(vif.monitor_cb);
                if (vif.monitor_cb.wvalid === 1'b1 &&
                    vif.monitor_cb.wready === 1'b1) begin
                    wd_q.push_back(vif.monitor_cb.wdata);
                    ws_q.push_back(vif.monitor_cb.wstrb);
                end
            end
            // Emite item cuando ambas colas tienen al menos 1
            forever begin
                @(vif.monitor_cb);
                while (aw_q.size() > 0 && wd_q.size() > 0) begin
                    item_t item = item_t::type_id::create("wr_obs");
                    item.is_write   = 1'b1;
                    item.addr       = aw_q.pop_front();
                    item.data       = wd_q.pop_front();
                    item.wstrb      = ws_q.pop_front();
                    item.t_req_fire = $time;
                    ap_wr.write(item);
                    pending_b.push_back(item);
                end
            end
        join
    endtask

    // ── Observa B fire ───────────────────────────────────
    task monitor_b();
        forever begin
            @(vif.monitor_cb);
            if (vif.monitor_cb.bvalid === 1'b1 &&
                vif.monitor_cb.bready === 1'b1) begin
                if (pending_b.size() == 0) begin
                    `uvm_error("WR_MON",
                        "B fire without pending AW+W in queue")
                end else begin
                    item_t b_item;
                    b_item = pending_b.pop_front();
                    b_item.resp        = vif.monitor_cb.bresp;
                    b_item.t_resp_fire = $time;
                    ap_b.write(b_item);
                end
            end
        end
    endtask

endclass
