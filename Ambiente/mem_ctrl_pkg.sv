// ============================================================
//  File    : mem_ctrl_pkg.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : UVM Verification Environment — package
//
//  Propósito:
//    Package que importa UVM y carga todas las clases del
//    environment en orden de dependencia. Cualquier archivo
//    que quiera usar clases UVM hace `import mem_ctrl_pkg::*`.
//
//  Orden de includes (importa por forward declarations):
//    1. defines.svh          — parámetros, typedefs, helpers
//    2. seq_item             — base de todas las transacciones
//    3. sequences            — usan seq_item
//    4. write_agent (3)      — driver, monitor, agent
//    5. read_agent  (3)      — driver, monitor, agent
//    6. coverage             — suscribe a analysis_ports de monitors
//    7. scoreboard           — suscribe a analysis_ports de monitors
//    8. env                  — instancia agents + scoreboard + coverage
//    9. tests                — instancian env, lanzan sequences
// ============================================================

`ifndef MEM_CTRL_PKG_SV
`define MEM_CTRL_PKG_SV

package mem_ctrl_pkg;

    // ── UVM imports ──────────────────────────────────────
    import uvm_pkg::*;
    `include "uvm_macros.svh"

    // ── Defines globales (parámetros default + helpers) ──
    `include "mem_ctrl_defines.svh"

    // ── Phase 2: estímulo ────────────────────────────────
    `include "mem_ctrl_seq_item.sv"
    `include "mem_ctrl_sequences.sv"

    // ── Phase 2: agentes ─────────────────────────────────
    `include "write_driver.sv"
    `include "write_monitor.sv"
    `include "write_agent.sv"

    `include "read_driver.sv"
    `include "read_monitor.sv"
    `include "read_agent.sv"

    // ── Phase 3: checking + cobertura ────────────────────
    `include "mem_ctrl_coverage.sv"
    `include "mem_ctrl_scoreboard.sv"
    `include "mem_ctrl_env.sv"

    // ── Phase 4: tests ───────────────────────────────────
    `include "mem_ctrl_tests.sv"

endpackage : mem_ctrl_pkg

`endif // MEM_CTRL_PKG_SV
