// ============================================================
//  File    : mem_ctrl_hazard_tracker.sv
//  Project : Banked Memory Controller — TFG ITCR
//  Block   : RAW hazard tracker — restricción AXI4-Lite §8.2
//
//  Por especificación AXI4-Lite el master es responsable de
//  mantener el orden WR/RD a la misma dirección. Este tracker
//  registra direcciones recientemente escritas dentro de una
//  ventana de HAZARD_WINDOW ciclos, y las sequences de RD las
//  excluyen al randomizar.
//
//  Singleton: una instancia publicada en uvm_config_db; las
//  sequences y el monitor de WR la consultan/actualizan.
// ============================================================

class mem_ctrl_hazard_tracker extends uvm_object;
    `uvm_object_utils(mem_ctrl_hazard_tracker)

    int unsigned hazard_window = 10;   // ciclos

    bit [31:0] recent_wr_addrs[$];
    time       recent_wr_times[$];

    function new(string name = "mem_ctrl_hazard_tracker");
        super.new(name);
    endfunction

    function void add_recent_wr(bit [31:0] addr);
        recent_wr_addrs.push_back(addr);
        recent_wr_times.push_back($time);
        prune();
    endfunction

    function void prune();
        while (recent_wr_addrs.size() > 0 &&
               ($time - recent_wr_times[0]) > hazard_window) begin
            recent_wr_addrs.pop_front();
            recent_wr_times.pop_front();
        end
    endfunction

    function bit is_recent_wr(bit [31:0] addr);
        prune();
        foreach (recent_wr_addrs[i])
            if (recent_wr_addrs[i] == addr)
                return 1'b1;
        return 1'b0;
    endfunction
endclass