// ============================================================
//  File    : tb_sram_bank_controller_uvm.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : SRAM Bank Controller — UVM Monitor Testbench
//
//  Descripción:
//    Testbench UVM "ligero" para análisis rápido del módulo
//    sram_bank_controller. Incluye:
//
//      ┌─────────────────────────────────────────┐
//      │  uvm_test  (bank_ctrl_test)             │
//      │    └─ uvm_env  (bank_ctrl_env)          │
//      │         └─ uvm_monitor (bank_ctrl_mon)  │
//      └─────────────────────────────────────────┘
//
//    El monitor observa la interfaz y construye transacciones
//    completas (request + respuesta) que imprime con uvm_info.
//    No hay scoreboard ni coverage — solo observación.
//
//    Las secuencias directed replican los 7 casos del TB plano
//    previo para comparar resultados directamente.
//
//  Compilación VCS:
//    vcs -sverilog -ntb_opts uvm-1.2           \
//        sram_bank_controller.sv               \
//        tb_sram_bank_controller_uvm.sv        \
//        +incdir+$VCS_HOME/etc/uvm-1.2/src     \
//        -o simv_uvm && ./simv_uvm             \
//        +UVM_VERBOSITY=UVM_MEDIUM             \
//        +UVM_TESTNAME=bank_ctrl_test
//
//  Parámetros editables:
//    READ_LATENCY — cambiar a 4 para segunda run
//
//  Clocking : 100 MHz (período 10 ns)
//  Reset    : síncrono, 5 ciclos rst_n=0
// ============================================================

`include "uvm_macros.svh"
import uvm_pkg::*;

// ============================================================
// PARÁMETROS GLOBALES
// ============================================================
`define AXI_DATA_WIDTH  32
`define BANK_ADDR_WIDTH 10
`define N_BANKS         4
`define READ_LATENCY    1     // <<< cambiar a 4 para segunda run
`define LAT_CNT_W       8
`define STRB_W          (`AXI_DATA_WIDTH/8)
`define TAG_W           ($clog2(`N_BANKS))

// ============================================================
// MODELO DE SRAM SÍNCRONA (idéntico al TB plano)
// ============================================================
module sram_sync_model #(
    parameter AXI_DATA_WIDTH  = 32,
    parameter BANK_ADDR_WIDTH = 10,
    parameter READ_LATENCY    = 1
)(
    input  wire                        clk,
    input  wire                        en,
    input  wire                        we,
    input  wire [BANK_ADDR_WIDTH-1:0]  addr,
    input  wire [AXI_DATA_WIDTH-1:0]   din,
    input  wire [AXI_DATA_WIDTH/8-1:0] wstrb,
    output reg  [AXI_DATA_WIDTH-1:0]   dout
);
    localparam MEM_DEPTH = (1 << BANK_ADDR_WIDTH);
    localparam STRB_W    = AXI_DATA_WIDTH / 8;

    reg [AXI_DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];
    reg [AXI_DATA_WIDTH-1:0] rd_pipe [0:READ_LATENCY-1];

    integer j, b;

    initial begin
        for (j = 0; j < MEM_DEPTH; j = j + 1)
            mem[j] = {AXI_DATA_WIDTH{1'b0}};
        for (j = 0; j < READ_LATENCY; j = j + 1)
            rd_pipe[j] = {AXI_DATA_WIDTH{1'b0}};
        dout = {AXI_DATA_WIDTH{1'b0}};
    end

    always @(posedge clk) begin
        if (en) begin
            if (we) begin
                for (b = 0; b < STRB_W; b = b + 1)
                    if (wstrb[b]) mem[addr][b*8 +: 8] <= din[b*8 +: 8];
            end else begin
                rd_pipe[0] <= mem[addr];
            end
        end else begin
            rd_pipe[0] <= rd_pipe[0];
        end
        for (j = 1; j < READ_LATENCY; j = j + 1)
            rd_pipe[j] <= rd_pipe[j-1];
    end

    always @(*) dout = rd_pipe[READ_LATENCY-1];

endmodule


// ============================================================
// INTERFAZ — conecta DUT, TB y monitor sin pasar señales
//             manualmente por cada capa UVM
// ============================================================
interface bank_ctrl_if #(
    parameter AXI_DATA_WIDTH  = 32,
    parameter BANK_ADDR_WIDTH = 10,
    parameter N_BANKS         = 4
)(
    input logic clk
);
    localparam STRB_W = AXI_DATA_WIDTH / 8;
    localparam TAG_W  = $clog2(N_BANKS);

    // Entradas al DUT (driver las maneja)
    logic                        rst_n;
    logic                        bank_req_valid;
    logic                        bank_req_op;
    logic [BANK_ADDR_WIDTH-1:0]  bank_req_addr;
    logic [AXI_DATA_WIDTH-1:0]   bank_req_wdata;
    logic [STRB_W-1:0]           bank_req_wstrb;
    logic [TAG_W-1:0]            bank_req_tag;
    logic [AXI_DATA_WIDTH-1:0]   sram_dout;

    // Salidas del DUT (monitor las observa)
    logic                        sram_en;
    logic                        sram_we;
    logic [BANK_ADDR_WIDTH-1:0]  sram_addr;
    logic [AXI_DATA_WIDTH-1:0]   sram_din;
    logic [STRB_W-1:0]           sram_wstrb;
    logic                        bank_busy;
    logic                        wr_resp_valid;
    logic [AXI_DATA_WIDTH-1:0]   rd_resp_data;
    logic [TAG_W-1:0]            rd_resp_tag;
    logic                        rd_resp_valid;

    // Clocking block para el monitor (muestreo post-posedge)
    clocking mon_cb @(posedge clk);
        default input #1;
        input bank_req_valid, bank_req_op, bank_req_addr;
        input bank_req_wdata, bank_req_wstrb, bank_req_tag;
        input sram_en, sram_we, sram_addr;
        input bank_busy;
        input wr_resp_valid;
        input rd_resp_data, rd_resp_tag, rd_resp_valid;
    endclocking

    // Clocking block para el driver (setup antes de posedge)
    clocking drv_cb @(posedge clk);
        default output #1;
        output rst_n;
        output bank_req_valid, bank_req_op, bank_req_addr;
        output bank_req_wdata, bank_req_wstrb, bank_req_tag;
    endclocking

endinterface


// ============================================================
// UVM TRANSACTION
//   Representa una transacción completa: request + respuesta.
//   El monitor la construye en dos fases:
//     fase 1 — captura el request cuando bank_req_valid=1
//     fase 2 — captura la respuesta cuando wr/rd_resp_valid=1
// ============================================================
class bank_ctrl_trans extends uvm_sequence_item;
    `uvm_object_utils(bank_ctrl_trans)

    // ── Campos del request ───────────────────────────────
    bit                              is_write;
    bit [`BANK_ADDR_WIDTH-1:0]       addr;
    bit [`AXI_DATA_WIDTH-1:0]        wdata;
    bit [`STRB_W-1:0]                wstrb;
    bit [`TAG_W-1:0]                 tag;

    // ── Campos de la respuesta ───────────────────────────
    bit [`AXI_DATA_WIDTH-1:0]        rd_data;   // solo lecturas
    bit [`TAG_W-1:0]                 rd_tag;    // solo lecturas
    bit                              resp_ok;   // respuesta recibida

    // ── Métricas de timing ───────────────────────────────
    int unsigned req_cycle;   // ciclo en que llegó el request
    int unsigned resp_cycle;  // ciclo en que llegó la respuesta
    int unsigned latency;     // resp_cycle - req_cycle

    function new(string name = "bank_ctrl_trans");
        super.new(name);
        resp_ok = 0;
    endfunction

    // Impresión legible para uvm_info
    function string convert2string();
        string s;
        if (is_write) begin
            s = $sformatf(
                "WR  addr=0x%03h data=0x%08h strb=%04b tag=%0d | latencia=%0d ciclos",
                addr, wdata, wstrb, tag, latency);
        end else begin
            s = $sformatf(
                "RD  addr=0x%03h tag_req=%0d | rdata=0x%08h tag_resp=%0d | latencia=%0d ciclos",
                addr, tag, rd_data, rd_tag, latency);
        end
        return s;
    endfunction

endclass


// ============================================================
// UVM MONITOR
//   Observa la interfaz pasivamente. Por cada transacción
//   completa (request → respuesta) construye un objeto
//   bank_ctrl_trans, lo imprime y lo broadcast por el
//   analysis port para uso futuro (scoreboard, coverage).
// ============================================================
class bank_ctrl_monitor extends uvm_monitor;
    `uvm_component_utils(bank_ctrl_monitor)

    // Analysis port — disponible para scoreboard futuro
    uvm_analysis_port #(bank_ctrl_trans) ap;

    // Handle a la interfaz (se asigna en connect_phase)
    virtual bank_ctrl_if #(
        .AXI_DATA_WIDTH (`AXI_DATA_WIDTH),
        .BANK_ADDR_WIDTH(`BANK_ADDR_WIDTH),
        .N_BANKS        (`N_BANKS)
    ) vif;

    // Contador de ciclos compartido con el testbench
    int unsigned cycle_cnt;

    // Estadísticas de sesión
    int unsigned total_writes;
    int unsigned total_reads;
    int unsigned total_latency;

    function new(string name, uvm_component parent);
        super.new(name, parent);
        total_writes  = 0;
        total_reads   = 0;
        total_latency = 0;
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        ap = new("ap", this);
        // Obtener la interfaz desde el config_db
        if (!uvm_config_db #(virtual bank_ctrl_if #(
                .AXI_DATA_WIDTH (`AXI_DATA_WIDTH),
                .BANK_ADDR_WIDTH(`BANK_ADDR_WIDTH),
                .N_BANKS        (`N_BANKS)
            ))::get(this, "", "vif", vif))
            `uvm_fatal("MON", "No se encontró vif en config_db")
    endfunction

    // run_phase: loop infinito — una tarea por transacción
    task run_phase(uvm_phase phase);
        bank_ctrl_trans trans;
        forever begin
            collect_transaction(trans);
            ap.write(trans);
        end
    endtask

    // Recopila una transacción completa
    task collect_transaction(output bank_ctrl_trans trans);
        trans = bank_ctrl_trans::new("mon_trans");

        // ── Fase 1: esperar request válido ───────────────
        @(vif.mon_cb);
        while (!vif.mon_cb.bank_req_valid) @(vif.mon_cb);

        trans.is_write  = vif.mon_cb.bank_req_op;
        trans.addr      = vif.mon_cb.bank_req_addr;
        trans.wdata     = vif.mon_cb.bank_req_wdata;
        trans.wstrb     = vif.mon_cb.bank_req_wstrb;
        trans.tag       = vif.mon_cb.bank_req_tag;
        trans.req_cycle = cycle_cnt;

        `uvm_info("MON",
            $sformatf("[C=%0d] REQ capturado — %s addr=0x%03h tag=%0d",
                cycle_cnt,
                trans.is_write ? "WR" : "RD",
                trans.addr,
                trans.tag),
            UVM_HIGH)

        // ── Fase 2: esperar respuesta ────────────────────
        @(vif.mon_cb);
        while (!vif.mon_cb.wr_resp_valid && !vif.mon_cb.rd_resp_valid)
            @(vif.mon_cb);

        trans.resp_cycle = cycle_cnt;
        trans.latency    = trans.resp_cycle - trans.req_cycle;
        trans.resp_ok    = 1;

        if (vif.mon_cb.rd_resp_valid) begin
            trans.rd_data = vif.mon_cb.rd_resp_data;
            trans.rd_tag  = vif.mon_cb.rd_resp_tag;
        end

        // Actualizar estadísticas
        if (trans.is_write) total_writes++;
        else                total_reads++;
        total_latency += trans.latency;

        // Imprimir transacción completa en MEDIUM para verla siempre
        `uvm_info("MON", $sformatf("[C=%0d] TRANS completa — %s",
            cycle_cnt, trans.convert2string()), UVM_MEDIUM)

        // Alerta si latencia es mayor a la esperada
        if (trans.latency > `READ_LATENCY + 4)
            `uvm_warning("MON",
                $sformatf("Latencia inusual: %0d ciclos (READ_LATENCY=%0d)",
                    trans.latency, `READ_LATENCY))

    endtask

    // Reporte de estadísticas al final
    function void report_phase(uvm_phase phase);
        int unsigned avg_lat;
        avg_lat = (total_writes + total_reads > 0) ?
                  total_latency / (total_writes + total_reads) : 0;
        `uvm_info("MON", "─────────────────────────────────────────", UVM_NONE)
        `uvm_info("MON", $sformatf("  Total writes      : %0d", total_writes),  UVM_NONE)
        `uvm_info("MON", $sformatf("  Total reads       : %0d", total_reads),   UVM_NONE)
        `uvm_info("MON", $sformatf("  Latencia promedio : %0d ciclos", avg_lat),UVM_NONE)
        `uvm_info("MON", $sformatf("  READ_LATENCY param: %0d", `READ_LATENCY), UVM_NONE)
        `uvm_info("MON", "─────────────────────────────────────────", UVM_NONE)
    endfunction

endclass


// ============================================================
// UVM ENV — mínimo: solo instancia el monitor
// ============================================================
class bank_ctrl_env extends uvm_env;
    `uvm_component_utils(bank_ctrl_env)

    bank_ctrl_monitor mon;

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        mon = bank_ctrl_monitor::type_id::create("mon", this);
    endfunction

endclass


// ============================================================
// SECUENCIAS DIRECTED — replican los 7 casos del TB plano
//
//   Cada secuencia opera sobre la interfaz directamente
//   (no hay driver/sequencer desacoplado para mantenerlo
//   ligero). El body() recibe el vif vía campo público.
// ============================================================

// ── Secuencia base con helpers ───────────────────────────────
class bank_ctrl_seq_base extends uvm_sequence;
    `uvm_object_utils(bank_ctrl_seq_base)

    // Interfaz compartida con el test
    virtual bank_ctrl_if #(
        .AXI_DATA_WIDTH (`AXI_DATA_WIDTH),
        .BANK_ADDR_WIDTH(`BANK_ADDR_WIDTH),
        .N_BANKS        (`N_BANKS)
    ) vif;

    // Contador de errores — acumulado entre secuencias
    static int unsigned errors = 0;

    function new(string name = "bank_ctrl_seq_base");
        super.new(name);
    endfunction

    // ── Task: emitir write ────────────────────────────────
    task do_write(
        input [`BANK_ADDR_WIDTH-1:0] addr,
        input [`AXI_DATA_WIDTH-1:0]  data,
        input [`STRB_W-1:0]          strb,
        input [`TAG_W-1:0]           tag
    );
        @(vif.drv_cb);
        vif.drv_cb.bank_req_valid <= 1'b1;
        vif.drv_cb.bank_req_op    <= 1'b1;
        vif.drv_cb.bank_req_addr  <= addr;
        vif.drv_cb.bank_req_wdata <= data;
        vif.drv_cb.bank_req_wstrb <= strb;
        vif.drv_cb.bank_req_tag   <= tag;
        @(vif.drv_cb);
        vif.drv_cb.bank_req_valid <= 1'b0;
        vif.drv_cb.bank_req_op    <= 1'b0;
        vif.drv_cb.bank_req_wdata <= {`AXI_DATA_WIDTH{1'b0}};
        vif.drv_cb.bank_req_wstrb <= {`STRB_W{1'b0}};
        wait_idle();
    endtask

    // ── Task: emitir read ─────────────────────────────────
    task do_read(
        input [`BANK_ADDR_WIDTH-1:0] addr,
        input [`TAG_W-1:0]           tag
    );
        @(vif.drv_cb);
        vif.drv_cb.bank_req_valid <= 1'b1;
        vif.drv_cb.bank_req_op    <= 1'b0;
        vif.drv_cb.bank_req_addr  <= addr;
        vif.drv_cb.bank_req_wdata <= {`AXI_DATA_WIDTH{1'b0}};
        vif.drv_cb.bank_req_wstrb <= {`STRB_W{1'b0}};
        vif.drv_cb.bank_req_tag   <= tag;
        @(vif.drv_cb);
        vif.drv_cb.bank_req_valid <= 1'b0;
        wait_idle();
    endtask

    // ── Task: esperar IDLE ────────────────────────────────
    task wait_idle();
        @(vif.drv_cb);
        while (vif.bank_busy) @(vif.drv_cb);
        @(vif.drv_cb);
    endtask

    // ── Check helper ─────────────────────────────────────
    function void chk(
        input bit        cond,
        input int        tnum,
        input string     msg
    );
        if (!cond) begin
            errors++;
            `uvm_error($sformatf("T%0d", tnum), msg)
        end else begin
            `uvm_info($sformatf("T%0d", tnum),
                $sformatf("OK — %s", msg), UVM_HIGH)
        end
    endfunction

endclass


// ── T1: Write simple ─────────────────────────────────────────
class seq_t1_write extends bank_ctrl_seq_base;
    `uvm_object_utils(seq_t1_write)
    function new(string name = "seq_t1_write"); super.new(name); endfunction

    task body();
        bit wr_seen, rd_seen, busy_seen;
        wr_seen = 0; rd_seen = 0; busy_seen = 0;

        `uvm_info("T1", "Write simple — addr=0x010 data=0xDEADBEEF strb=F", UVM_MEDIUM)

        fork
            begin
                repeat (50) begin
                    @(vif.mon_cb);
                    if (vif.mon_cb.bank_busy)      busy_seen = 1;
                    if (vif.mon_cb.wr_resp_valid)  wr_seen   = 1;
                    if (vif.mon_cb.rd_resp_valid)  rd_seen   = 1;
                end
            end
            begin
                do_write(10'h010, 32'hDEAD_BEEF, 4'b1111, 2'd0);
            end
        join

        chk(wr_seen,   1, "wr_resp_valid pulsó");
        chk(!rd_seen,  1, "sin rd_resp_valid espurio");
        chk(busy_seen, 1, "bank_busy subió durante write");
    endtask
endclass


// ── T2: Read simple ──────────────────────────────────────────
class seq_t2_read extends bank_ctrl_seq_base;
    `uvm_object_utils(seq_t2_read)
    function new(string name = "seq_t2_read"); super.new(name); endfunction

    task body();
        bit rd_seen, wr_seen;
        bit [`AXI_DATA_WIDTH-1:0] cap_data;
        bit [`TAG_W-1:0]          cap_tag;
        rd_seen = 0; wr_seen = 0;

        `uvm_info("T2", "Read simple — addr=0x010 tag=2 (esperado 0xDEADBEEF)", UVM_MEDIUM)

        fork
            begin
                repeat (50) begin
                    @(vif.mon_cb);
                    if (vif.mon_cb.rd_resp_valid) begin
                        rd_seen  = 1;
                        cap_data = vif.mon_cb.rd_resp_data;
                        cap_tag  = vif.mon_cb.rd_resp_tag;
                    end
                    if (vif.mon_cb.wr_resp_valid) wr_seen = 1;
                end
            end
            begin
                do_read(10'h010, 2'd2);
            end
        join

        chk(rd_seen,                   2, "rd_resp_valid pulsó");
        chk(!wr_seen,                  2, "sin wr_resp_valid espurio");
        chk(cap_data == 32'hDEAD_BEEF, 2,
            $sformatf("rdata correcto (got=0x%08h)", cap_data));
        chk(cap_tag == 2'd2,           2,
            $sformatf("tag correcto (got=%0d)", cap_tag));
    endtask
endclass


// ── T3: Byte strobes ─────────────────────────────────────────
class seq_t3_strobes extends bank_ctrl_seq_base;
    `uvm_object_utils(seq_t3_strobes)
    function new(string name = "seq_t3_strobes"); super.new(name); endfunction

    task body();
        bit rd_seen;
        bit [`AXI_DATA_WIDTH-1:0] cap_data;
        rd_seen = 0;

        `uvm_info("T3", "Byte strobes — WR addr=0x020 data=0xAABBCCDD strb=0011", UVM_MEDIUM)
        do_write(10'h020, 32'hAABB_CCDD, 4'b0011, 2'd0);

        fork
            begin
                repeat (50) begin
                    @(vif.mon_cb);
                    if (vif.mon_cb.rd_resp_valid) begin
                        rd_seen  = 1;
                        cap_data = vif.mon_cb.rd_resp_data;
                    end
                end
            end
            begin
                do_read(10'h020, 2'd1);
            end
        join

        chk(rd_seen,                    3, "rd_resp_valid pulsó");
        chk(cap_data == 32'h0000_CCDD,  3,
            $sformatf("byte strobes OK (got=0x%08h expected=0x0000CCDD)", cap_data));
    endtask
endclass


// ── T4: Back-to-back writes ───────────────────────────────────
class seq_t4_btb_writes extends bank_ctrl_seq_base;
    `uvm_object_utils(seq_t4_btb_writes)
    function new(string name = "seq_t4_btb_writes"); super.new(name); endfunction

    task body();
        int pulse_count;
        pulse_count = 0;

        `uvm_info("T4", "Back-to-back writes x3", UVM_MEDIUM)

        fork
            begin
                repeat (150) begin
                    @(vif.mon_cb);
                    if (vif.mon_cb.wr_resp_valid) pulse_count++;
                end
            end
            begin
                do_write(10'h030, 32'h1111_1111, 4'b1111, 2'd0);
                do_write(10'h034, 32'h2222_2222, 4'b1111, 2'd0);
                do_write(10'h038, 32'h3333_3333, 4'b1111, 2'd0);
            end
        join

        chk(pulse_count == 3, 4,
            $sformatf("3 pulsos wr_resp_valid (got=%0d)", pulse_count));
    endtask
endclass


// ── T5: WAR hazard ───────────────────────────────────────────
class seq_t5_war extends bank_ctrl_seq_base;
    `uvm_object_utils(seq_t5_war)
    function new(string name = "seq_t5_war"); super.new(name); endfunction

    task body();
        bit [`AXI_DATA_WIDTH-1:0] rd1, rd2;
        int rd_count;
        rd1 = 0; rd2 = 0; rd_count = 0;

        `uvm_info("T5", "WAR hazard — RD(0x040) WR(0x040,0x12345678) RD(0x040)", UVM_MEDIUM)

        fork
            begin
                repeat (200) begin
                    @(vif.mon_cb);
                    if (vif.mon_cb.rd_resp_valid) begin
                        rd_count++;
                        if (rd_count == 1) rd1 = vif.mon_cb.rd_resp_data;
                        if (rd_count == 2) rd2 = vif.mon_cb.rd_resp_data;
                    end
                end
            end
            begin
                do_read (10'h040, 2'd0);
                do_write(10'h040, 32'h1234_5678, 4'b1111, 2'd0);
                do_read (10'h040, 2'd1);
            end
        join

        chk(rd_count >= 2,              5, "dos lecturas completadas");
        chk(rd1 == 32'h0000_0000,       5,
            $sformatf("1er RD = 0 (got=0x%08h)", rd1));
        chk(rd2 == 32'h1234_5678,       5,
            $sformatf("2do RD refleja write (got=0x%08h)", rd2));
    endtask
endclass


// ── T6: Latencia variable ─────────────────────────────────────
class seq_t6_latency extends bank_ctrl_seq_base;
    `uvm_object_utils(seq_t6_latency)
    function new(string name = "seq_t6_latency"); super.new(name); endfunction

    // El test recibe el cycle_cnt del monitor por referencia
    int unsigned start_cycle;
    int unsigned end_cycle;

    task body();
        bit resp_seen;
        int elapsed, exp_min, exp_max;
        resp_seen = 0;
        exp_min = `READ_LATENCY + 1;
        exp_max = `READ_LATENCY + 3;

        `uvm_info("T6", $sformatf("Latencia — READ_LATENCY=%0d", `READ_LATENCY), UVM_MEDIUM)

        fork
            begin
                repeat (100) begin
                    @(vif.mon_cb);
                    if (vif.mon_cb.rd_resp_valid && !resp_seen) begin
                        resp_seen = 1;
                        end_cycle = $time / 10; // ciclos desde tiempo 0
                    end
                end
            end
            begin
                @(vif.drv_cb);
                start_cycle = $time / 10;
                vif.drv_cb.bank_req_valid <= 1'b1;
                vif.drv_cb.bank_req_op    <= 1'b0;
                vif.drv_cb.bank_req_addr  <= 10'h050;
                vif.drv_cb.bank_req_tag   <= 2'd0;
                @(vif.drv_cb);
                vif.drv_cb.bank_req_valid <= 1'b0;
                wait_idle();
            end
        join

        elapsed = end_cycle - start_cycle;
        chk(resp_seen, 6, "rd_resp_valid visto");
        chk(elapsed >= exp_min && elapsed <= exp_max, 6,
            $sformatf("latencia=%0d ciclos (esperado %0d..%0d)",
                      elapsed, exp_min, exp_max));

        `uvm_info("T6", $sformatf("elapsed=%0d ciclos", elapsed), UVM_MEDIUM)
    endtask
endclass


// ── T7: Tag preservation ──────────────────────────────────────
class seq_t7_tag extends bank_ctrl_seq_base;
    `uvm_object_utils(seq_t7_tag)
    function new(string name = "seq_t7_tag"); super.new(name); endfunction

    task body();
        bit rd_seen;
        bit [`TAG_W-1:0] cap_tag;
        rd_seen = 0;

        `uvm_info("T7", "Tag preservation — WR(tag=0) RD(tag=3)", UVM_MEDIUM)
        do_write(10'h060, 32'hFACE_CAFE, 4'b1111, 2'd0);

        fork
            begin
                repeat (50) begin
                    @(vif.mon_cb);
                    if (vif.mon_cb.rd_resp_valid) begin
                        rd_seen = 1;
                        cap_tag = vif.mon_cb.rd_resp_tag;
                    end
                end
            end
            begin
                do_read(10'h060, 2'd3);
            end
        join

        chk(rd_seen,           7, "rd_resp_valid pulsó");
        chk(cap_tag == 2'd3,   7,
            $sformatf("tag correcto (got=%0d expected=3)", cap_tag));
    endtask
endclass


// ============================================================
// UVM TEST
//   Construye el env, obtiene la interfaz del config_db,
//   y lanza las 7 secuencias en orden.
// ============================================================
class bank_ctrl_test extends uvm_test;
    `uvm_component_utils(bank_ctrl_test)

    bank_ctrl_env env;

    // Handle a la interfaz para pasarla a las secuencias
    virtual bank_ctrl_if #(
        .AXI_DATA_WIDTH (`AXI_DATA_WIDTH),
        .BANK_ADDR_WIDTH(`BANK_ADDR_WIDTH),
        .N_BANKS        (`N_BANKS)
    ) vif;

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        env = bank_ctrl_env::type_id::create("env", this);
        if (!uvm_config_db #(virtual bank_ctrl_if #(
                .AXI_DATA_WIDTH (`AXI_DATA_WIDTH),
                .BANK_ADDR_WIDTH(`BANK_ADDR_WIDTH),
                .N_BANKS        (`N_BANKS)
            ))::get(this, "", "vif", vif))
            `uvm_fatal("TEST", "No se encontró vif en config_db")
    endfunction

    task run_phase(uvm_phase phase);
        // Macros para crear y lanzar cada secuencia
        seq_t1_write    t1;
        seq_t2_read     t2;
        seq_t3_strobes  t3;
        seq_t4_btb_writes t4;
        seq_t5_war      t5;
        seq_t6_latency  t6;
        seq_t7_tag      t7;

        phase.raise_objection(this);

        `uvm_info("TEST", $sformatf(
            "=== Inicio UVM — READ_LATENCY=%0d ===", `READ_LATENCY), UVM_NONE)

        // Pasar la interfaz a cada secuencia antes de lanzarla
        t1 = seq_t1_write::type_id::create("t1");
        t1.vif = vif;
        t1.start(null);

        t2 = seq_t2_read::type_id::create("t2");
        t2.vif = vif;
        t2.start(null);

        t3 = seq_t3_strobes::type_id::create("t3");
        t3.vif = vif;
        t3.start(null);

        t4 = seq_t4_btb_writes::type_id::create("t4");
        t4.vif = vif;
        t4.start(null);

        t5 = seq_t5_war::type_id::create("t5");
        t5.vif = vif;
        t5.start(null);

        t6 = seq_t6_latency::type_id::create("t6");
        t6.vif = vif;
        t6.start(null);

        t7 = seq_t7_tag::type_id::create("t7");
        t7.vif = vif;
        t7.start(null);

        `uvm_info("TEST", $sformatf(
            "=== Fin — %0d error(es) ===",
            bank_ctrl_seq_base::errors), UVM_NONE)

        // Propagar resultado al exit code de VCS
        if (bank_ctrl_seq_base::errors > 0)
            `uvm_error("TEST", "Simulación terminó con errores")

        phase.drop_objection(this);
    endtask

endclass


// ============================================================
// TOP — módulo de simulación
//   Instancia DUT, modelo SRAM e interfaz.
//   Conecta clk/reset, configura el config_db y lanza UVM.
// ============================================================
module tb_sram_bank_controller_uvm;

    localparam CLK_HALF = 5; // 5 ns → 100 MHz
    localparam TIMEOUT  = 10_000;

    // ── Reloj ────────────────────────────────────────────
    logic clk;
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    // ── Interfaz ─────────────────────────────────────────
    bank_ctrl_if #(
        .AXI_DATA_WIDTH (`AXI_DATA_WIDTH),
        .BANK_ADDR_WIDTH(`BANK_ADDR_WIDTH),
        .N_BANKS        (`N_BANKS)
    ) bif (.clk(clk));

    // ── DUT ──────────────────────────────────────────────
    sram_bank_controller #(
        .AXI_DATA_WIDTH (`AXI_DATA_WIDTH),
        .BANK_ADDR_WIDTH(`BANK_ADDR_WIDTH),
        .N_BANKS        (`N_BANKS),
        .READ_LATENCY   (`READ_LATENCY),
        .LAT_CNT_W      (`LAT_CNT_W)
    ) dut (
        .clk            (clk),
        .rst_n          (bif.rst_n),
        .bank_req_valid (bif.bank_req_valid),
        .bank_req_op    (bif.bank_req_op),
        .bank_req_addr  (bif.bank_req_addr),
        .bank_req_wdata (bif.bank_req_wdata),
        .bank_req_wstrb (bif.bank_req_wstrb),
        .bank_req_tag   (bif.bank_req_tag),
        .sram_en        (bif.sram_en),
        .sram_we        (bif.sram_we),
        .sram_addr      (bif.sram_addr),
        .sram_din       (bif.sram_din),
        .sram_wstrb     (bif.sram_wstrb),
        .sram_dout      (bif.sram_dout),
        .bank_busy      (bif.bank_busy),
        .wr_resp_valid  (bif.wr_resp_valid),
        .rd_resp_data   (bif.rd_resp_data),
        .rd_resp_tag    (bif.rd_resp_tag),
        .rd_resp_valid  (bif.rd_resp_valid)
    );

    // ── Modelo SRAM ──────────────────────────────────────
    sram_sync_model #(
        .AXI_DATA_WIDTH (`AXI_DATA_WIDTH),
        .BANK_ADDR_WIDTH(`BANK_ADDR_WIDTH),
        .READ_LATENCY   (`READ_LATENCY)
    ) sram_model (
        .clk  (clk),
        .en   (bif.sram_en),
        .we   (bif.sram_we),
        .addr (bif.sram_addr),
        .din  (bif.sram_din),
        .wstrb(bif.sram_wstrb),
        .dout (bif.sram_dout)
    );

    // ── Waveform dump ────────────────────────────────────
    initial begin
        $dumpfile("tb_sram_bank_controller_uvm.vcd");
        $dumpvars(0, tb_sram_bank_controller_uvm);
    end

    // ── Reset síncrono: 5 ciclos ─────────────────────────
    initial begin
        bif.rst_n          = 1'b0;
        bif.bank_req_valid = 1'b0;
        bif.bank_req_op    = 1'b0;
        bif.bank_req_addr  = '0;
        bif.bank_req_wdata = '0;
        bif.bank_req_wstrb = '0;
        bif.bank_req_tag   = '0;
        repeat (5) @(posedge clk);
        @(posedge clk);
        bif.rst_n = 1'b1;
        @(posedge clk);
    end

    // ── Watchdog ─────────────────────────────────────────
    initial begin
        repeat (TIMEOUT) @(posedge clk);
        `uvm_fatal("WATCHDOG", $sformatf("Timeout — %0d ciclos", TIMEOUT))
    end

    // ── Contador de ciclos para el monitor ───────────────
    // Se actualiza en el top y el monitor accede via config_db
    int unsigned cycle_cnt;
    always @(posedge clk) cycle_cnt++;

    // ── Arranque UVM ─────────────────────────────────────
    initial begin
        // Registrar la interfaz en el config_db
        uvm_config_db #(virtual bank_ctrl_if #(
            .AXI_DATA_WIDTH (`AXI_DATA_WIDTH),
            .BANK_ADDR_WIDTH(`BANK_ADDR_WIDTH),
            .N_BANKS        (`N_BANKS)
        ))::set(null, "uvm_test_top.*", "vif", bif);

        // Lanzar el test (nombre vía +UVM_TESTNAME o hardcodeado)
        run_test("bank_ctrl_test");
    end

endmodule
