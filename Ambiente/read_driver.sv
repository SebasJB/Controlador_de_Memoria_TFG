// ============================================================
//  File    : read_driver.sv
//  Project : Banked Memory Controller — TFG ITCR
//
//  Verbosity: avisos UVM_HIGH para handshake AR y R.
// ============================================================

class read_driver #(
    parameter int ADDR_W  = 32,
    parameter int DATA_W  = 32,
    parameter int N_BANKS = 4
) extends uvm_driver #(mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS));

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS) item_t;

    virtual mem_bank_interface #(ADDR_W, DATA_W) vif;

    int r_backpressure_cycles = 0;

    `uvm_component_param_utils(read_driver #(ADDR_W, DATA_W, N_BANKS))

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual mem_bank_interface #(ADDR_W, DATA_W))::get(
                this, "", "axi_vif", vif))
            `uvm_fatal("RD_DRV", "axi_vif not set")
        void'(uvm_config_db#(int)::get(this, "", "r_backpressure_cycles",
                                       r_backpressure_cycles));
        `uvm_info("RD_DRV", $sformatf("build_phase: r_backpressure_cycles=%0d",
                                       r_backpressure_cycles), UVM_HIGH)
    endfunction

    task run_phase(uvm_phase phase);
        vif.master_read_cb.arvalid <= 1'b0;
        vif.master_read_cb.rready  <= 1'b0;
        vif.master_read_cb.araddr  <= '0;

        wait (vif.rst_n === 1'b1);
        @(vif.master_read_cb);
        `uvm_info("RD_DRV", "Reset released — driver ready", UVM_HIGH)

        fork
            accept_r_responses();
        join_none

        forever begin
            item_t req;
            seq_item_port.get_next_item(req);
            `uvm_info("RD_DRV_REQ",
                $sformatf("got seq_item txn_id=%0d addr=0x%08h",
                          req.txn_id, req.addr),
                UVM_HIGH)
            repeat (req.delay_cycles) @(vif.master_read_cb.clk);
            drive_ar(req);
            seq_item_port.item_done();
        end
    endtask

    task drive_ar(item_t item);
        @(vif.master_read_cb);
        vif.master_read_cb.arvalid <= 1'b1;
        vif.master_read_cb.araddr  <= item.addr;
        do @(vif.master_read_cb); while (vif.master_read_cb.arready !== 1'b1);
        vif.master_read_cb.arvalid <= 1'b0;
        item.t_req_fire = $time;
        `uvm_info("RD_DRV_AR",
            $sformatf("AR fire txn_id=%0d addr=0x%08h @ %0t",
                      item.txn_id, item.addr, item.t_req_fire),
            UVM_HIGH)
    endtask

    task accept_r_responses();
    int r_idx = 0;
    forever begin
        // Esperar a que rvalid sea visible
        @(vif.master_read_cb);
        if (vif.master_read_cb.rvalid !== 1'b1) continue;

        // Backpressure opcional: stall con rready=0 mientras
        // rvalid se mantiene alto (master no acepta todavía)
        if (r_backpressure_cycles > 0) begin
            `uvm_info("RD_DRV_R",
                $sformatf("R stall %0d cycles before accepting @ %0t",
                          r_backpressure_cycles, $time),
                UVM_HIGH)
            repeat (r_backpressure_cycles) @(vif.master_read_cb);
        end

        // Aceptar: rready=1 por exactamente 1 ciclo.
        // Sample rdata/rresp ANTES del flanco para evitar
        // que el DUT cambie el dato cuando ve rready=1.
        vif.master_read_cb.rready <= 1'b1;
        @(vif.master_read_cb);
        // En este punto el handshake fire ocurrió. Bajar rready.
        vif.master_read_cb.rready <= 1'b0;

        `uvm_info("RD_DRV_R",
            $sformatf("R accepted #%0d rdata=0x%08h rresp=%0b @ %0t",
                      r_idx, vif.master_read_cb.rdata,
                      vif.master_read_cb.rresp, $time),
            UVM_HIGH)
        r_idx++;
    end
endtask

endclass
