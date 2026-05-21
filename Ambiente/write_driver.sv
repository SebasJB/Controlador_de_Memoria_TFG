// ============================================================
//  File    : write_driver.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Write Driver
//
//  Responsabilidades:
//    - Recibir seq_item del write_sequencer
//    - Drive AW + W en paralelo (fork) hasta handshake fire
//    - Aceptar B con knob de throttling (b_backpressure_cycles)
//    - Devolver el item completado al sequencer
//
//  Knobs configurables vía uvm_config_db:
//    - b_backpressure_cycles : ciclos con bready=0 antes de
//                              aceptar B. Default 0 = sin stall.
// ============================================================

class write_driver #(
    parameter int ADDR_W  = 32,
    parameter int DATA_W  = 32,
    parameter int N_BANKS = 4
) extends uvm_driver #(mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS));

    typedef mem_ctrl_seq_item #(ADDR_W, DATA_W, N_BANKS) item_t;

    virtual mem_bank_interface #(ADDR_W, DATA_W) vif;

    // Knob de backpressure
    int b_backpressure_cycles = 0;

    `uvm_component_param_utils(write_driver #(ADDR_W, DATA_W, N_BANKS))

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual mem_bank_interface #(ADDR_W, DATA_W))::get(
                this, "", "axi_vif", vif))
            `uvm_fatal("WR_DRV", "axi_vif not set")
        void'(uvm_config_db#(int)::get(this, "", "b_backpressure_cycles",
                                       b_backpressure_cycles));
    endfunction

    // ── Run phase ────────────────────────────────────────
    task run_phase(uvm_phase phase);
        // Inicializar salidas a 0
        vif.master_write_cb.awvalid <= 1'b0;
        vif.master_write_cb.wvalid  <= 1'b0;
        vif.master_write_cb.bready  <= 1'b0;
        vif.master_write_cb.awaddr  <= '0;
        vif.master_write_cb.wdata   <= '0;
        vif.master_write_cb.wstrb   <= '0;

        wait (vif.rst_n === 1'b1);
        @(vif.master_write_cb);

        // Hilo paralelo para aceptar B en background
        fork
            accept_b_responses();
        join_none

        forever begin
            item_t req;
            seq_item_port.get_next_item(req);
            drive_aw_w(req);
            seq_item_port.item_done();
        end
    endtask

    // ── Drive AW y W en paralelo ─────────────────────────
    task drive_aw_w(item_t item);
        fork
            drive_aw(item);
            drive_w(item);
        join
        item.t_req_fire = $time;
    endtask

    task drive_aw(item_t item);
        vif.master_write_cb.awvalid <= 1'b1;
        vif.master_write_cb.awaddr  <= item.addr;
        do @(vif.master_write_cb); while (vif.master_write_cb.awready !== 1'b1);
        vif.master_write_cb.awvalid <= 1'b0;
    endtask

    task drive_w(item_t item);
        vif.master_write_cb.wvalid <= 1'b1;
        vif.master_write_cb.wdata  <= item.data;
        vif.master_write_cb.wstrb  <= item.wstrb;
        do @(vif.master_write_cb); while (vif.master_write_cb.wready !== 1'b1);
        vif.master_write_cb.wvalid <= 1'b0;
    endtask

    // ── Acepta B (background) ────────────────────────────
    // El monitor es quien observa la transacción; el driver
    // solo se encarga del handshake bready.
    task accept_b_responses();
        forever begin
            @(vif.master_write_cb);
            if (vif.master_write_cb.bvalid === 1'b1) begin
                if (b_backpressure_cycles > 0)
                    repeat (b_backpressure_cycles) @(vif.master_write_cb);
                vif.master_write_cb.bready <= 1'b1;
                @(vif.master_write_cb);
                vif.master_write_cb.bready <= 1'b0;
            end
        end
    endtask

endclass
