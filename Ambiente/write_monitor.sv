// ============================================================
//  File    : write_monitor.sv
//  Project : Banked Memory Controller — TFG ITCR
//
//  Verbosity: avisos UVM_HIGH para AW/W/B fires y emisión.
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

    item_t pending_b[$];
    mem_ctrl_hazard_tracker hazard;

    `uvm_component_param_utils(write_monitor #(ADDR_W, DATA_W, N_BANKS))

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap_wr = new("ap_wr", this);
        ap_b  = new("ap_b",  this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual mem_bank_interface #(ADDR_W, DATA_W))::get(this, "", "axi_vif", vif))
            `uvm_fatal("WR_MON", "axi_vif not set")
        if (!uvm_config_db#(mem_ctrl_hazard_tracker)::get(this, "", "hazard_tracker", hazard))
            `uvm_fatal("HAZARD", "No hazard_tracker in config_db")
    endfunction

    task run_phase(uvm_phase phase);
        wait (vif.rst_n === 1'b1);
        @(vif.monitor_cb);
        `uvm_info("WR_MON", "Reset released — monitor ready", UVM_HIGH)
        fork
            monitor_aw_w();
            monitor_b();
        join
    endtask

    task monitor_aw_w();
        bit [ADDR_W-1:0]    aw_q[$];
        bit [DATA_W-1:0]    wd_q[$];
        bit [DATA_W/8-1:0]  ws_q[$];
        int                 aw_idx = 0;
        int                 w_idx  = 0;
        int                 wr_idx = 0;

        fork
            // Captura AW fire
            forever begin
                @(vif.monitor_cb);
                if (vif.monitor_cb.awvalid === 1'b1 &&
                    vif.monitor_cb.awready === 1'b1) begin
                    aw_q.push_back(vif.monitor_cb.awaddr);
                    `uvm_info("WR_MON_AW",
                        $sformatf("AW fire #%0d addr=0x%08h @ %0t",
                                  aw_idx, vif.monitor_cb.awaddr, $time),
                        UVM_HIGH)
                    aw_idx++;
                end
            end
            // Captura W fire
            forever begin
                @(vif.monitor_cb);
                if (vif.monitor_cb.wvalid === 1'b1 &&
                    vif.monitor_cb.wready === 1'b1) begin
                    wd_q.push_back(vif.monitor_cb.wdata);
                    ws_q.push_back(vif.monitor_cb.wstrb);
                    `uvm_info("WR_MON_W",
                        $sformatf("W fire #%0d data=0x%08h wstrb=0x%h @ %0t",
                                  w_idx, vif.monitor_cb.wdata,
                                  vif.monitor_cb.wstrb, $time),
                        UVM_HIGH)
                    w_idx++;
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
                    `uvm_info("WR_MON_TXN",
                        $sformatf("emit txn #%0d txn_id=%0d addr=0x%08h data=0x%08h wstrb=0x%h",
                                  wr_idx, item.txn_id, item.addr,
                                  item.data, item.wstrb),
                        UVM_HIGH)
                    ap_wr.write(item);
                    pending_b.push_back(item);
                    // Registrar la dirección en el hazard tracker para
                    // que las RD posteriores la excluyan durante la
                    // ventana de HAZARD_WINDOW ciclos (restricción
                    // AXI4-Lite §8.2 del master).
                    if (hazard != null)
                        hazard.add_recent_wr(item.addr);
                    wr_idx++;
                end
            end
        join
    endtask

    task monitor_b();
        int b_idx = 0;
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
                    `uvm_info("WR_MON_B",
                        $sformatf("B fire #%0d txn_id=%0d bresp=%0b lat=%0t @ %0t",
                                  b_idx, b_item.txn_id, b_item.resp,
                                  b_item.t_resp_fire - b_item.t_req_fire, $time),
                        UVM_HIGH)
                    ap_b.write(b_item);
                    b_idx++;
                end
            end
        end
    endtask

endclass
