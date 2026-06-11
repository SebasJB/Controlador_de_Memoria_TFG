wvCreateWindow
wvConvertFile -win $_nWave2 -o \
           "/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/Simulaciones/tb_mem_ctrl.vcd.fsdb" \
           "/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/Simulaciones/tb_mem_ctrl.vcd"
wvSetPosition -win $_nWave2 {("G1" 0)}
wvOpenFile -win $_nWave2 \
           {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/Simulaciones/tb_mem_ctrl.vcd.fsdb}
wvSetPosition -win $_nWave2 {("G1" 0)}
nMemSetPreference
srcSetDisplayAttr -font {-fromGUI}
srcSetDisplayAttr -annotFont {Helvetica 10}
wvGetSignalOpen -win $_nWave2
wvGetSignalSetScope -win $_nWave2 "/tb_top"
wvGetSignalSetScope -win $_nWave2 "/tb_top/dut/u_scheduler"
wvGetSignalSetScope -win $_nWave2 "/tb_top/dut"
wvGetSignalSetScope -win $_nWave2 "/tb_top/axi_if"
wvGetSignalSetScope -win $_nWave2 "/tb_top/dut/u_rd_req_fifo"
wvGetSignalSetScope -win $_nWave2 "/tb_top/dut/u_rd_response_path"
wvGetSignalSetScope -win $_nWave2 "/tb_top/dut/u_wr_response_path"
wvGetSignalSetScope -win $_nWave2 "/tb_top/dut/u_rd_response_path"
wvGetSignalSetScope -win $_nWave2 "/tb_top/dut/u_scheduler"
wvSetPosition -win $_nWave2 {("G6" 2)}
wvSetPosition -win $_nWave2 {("G6" 2)}
wvAddSignal -win $_nWave2 -clear
wvAddSignal -win $_nWave2 -group {"G1" \
{/tb_top/dut/u_scheduler/rd_discard} \
{/tb_top/dut/u_scheduler/wr_discard} \
}
wvAddSignal -win $_nWave2 -group {"G2" \
{/tb_top/dut/u_scheduler/wr_req_pndng} \
{/tb_top/dut/u_scheduler/rd_req_pndng} \
}
wvAddSignal -win $_nWave2 -group {"G3" \
{/tb_top/dut/u_scheduler/rd_req_pop} \
{/tb_top/dut/u_scheduler/wr_req_pop} \
}
wvAddSignal -win $_nWave2 -group {"G4" \
{/tb_top/dut/u_scheduler/grant_rd} \
{/tb_top/dut/u_scheduler/grant_wr} \
}
wvAddSignal -win $_nWave2 -group {"G5" \
{/tb_top/dut/u_wr_response_path/s_axi_bvalid} \
{/tb_top/dut/u_rd_response_path/s_axi_rvalid} \
}
wvAddSignal -win $_nWave2 -group {"G6" \
{/tb_top/dut/u_scheduler/rd_err_cnt\[15:0\]} \
{/tb_top/dut/u_scheduler/wr_err_cnt\[15:0\]} \
}
wvAddSignal -win $_nWave2 -group {"G7" \
}
wvSelectSignal -win $_nWave2 {( "G6" 1 2 )} 
wvSetPosition -win $_nWave2 {("G6" 2)}
wvSetPosition -win $_nWave2 {("G6" 2)}
wvSetPosition -win $_nWave2 {("G6" 2)}
wvAddSignal -win $_nWave2 -clear
wvAddSignal -win $_nWave2 -group {"G1" \
{/tb_top/dut/u_scheduler/rd_discard} \
{/tb_top/dut/u_scheduler/wr_discard} \
}
wvAddSignal -win $_nWave2 -group {"G2" \
{/tb_top/dut/u_scheduler/wr_req_pndng} \
{/tb_top/dut/u_scheduler/rd_req_pndng} \
}
wvAddSignal -win $_nWave2 -group {"G3" \
{/tb_top/dut/u_scheduler/rd_req_pop} \
{/tb_top/dut/u_scheduler/wr_req_pop} \
}
wvAddSignal -win $_nWave2 -group {"G4" \
{/tb_top/dut/u_scheduler/grant_rd} \
{/tb_top/dut/u_scheduler/grant_wr} \
}
wvAddSignal -win $_nWave2 -group {"G5" \
{/tb_top/dut/u_wr_response_path/s_axi_bvalid} \
{/tb_top/dut/u_rd_response_path/s_axi_rvalid} \
}
wvAddSignal -win $_nWave2 -group {"G6" \
{/tb_top/dut/u_scheduler/rd_err_cnt\[15:0\]} \
{/tb_top/dut/u_scheduler/wr_err_cnt\[15:0\]} \
}
wvAddSignal -win $_nWave2 -group {"G7" \
}
wvSelectSignal -win $_nWave2 {( "G6" 1 2 )} 
wvSetPosition -win $_nWave2 {("G6" 2)}
wvSetPosition -win $_nWave2 {("G6" 2)}
wvSetPosition -win $_nWave2 {("G6" 2)}
wvAddSignal -win $_nWave2 -clear
wvAddSignal -win $_nWave2 -group {"G1" \
{/tb_top/dut/u_scheduler/rd_discard} \
{/tb_top/dut/u_scheduler/wr_discard} \
}
wvAddSignal -win $_nWave2 -group {"G2" \
{/tb_top/dut/u_scheduler/wr_req_pndng} \
{/tb_top/dut/u_scheduler/rd_req_pndng} \
}
wvAddSignal -win $_nWave2 -group {"G3" \
{/tb_top/dut/u_scheduler/rd_req_pop} \
{/tb_top/dut/u_scheduler/wr_req_pop} \
}
wvAddSignal -win $_nWave2 -group {"G4" \
{/tb_top/dut/u_scheduler/grant_rd} \
{/tb_top/dut/u_scheduler/grant_wr} \
}
wvAddSignal -win $_nWave2 -group {"G5" \
{/tb_top/dut/u_wr_response_path/s_axi_bvalid} \
{/tb_top/dut/u_rd_response_path/s_axi_rvalid} \
}
wvAddSignal -win $_nWave2 -group {"G6" \
{/tb_top/dut/u_scheduler/rd_err_cnt\[15:0\]} \
{/tb_top/dut/u_scheduler/wr_err_cnt\[15:0\]} \
}
wvAddSignal -win $_nWave2 -group {"G7" \
}
wvSelectSignal -win $_nWave2 {( "G6" 1 2 )} 
wvSetPosition -win $_nWave2 {("G6" 2)}
wvGetSignalClose -win $_nWave2
verdiDockWidgetMaximize -dock windowDock_nWave_2
wvSetCursor -win $_nWave2 34762953.823039 -snap {("G3" 1)}
wvSetCursor -win $_nWave2 46442207.083744 -snap {("G3" 1)}
wvZoomIn -win $_nWave2
wvZoomIn -win $_nWave2
wvZoomIn -win $_nWave2
wvZoomIn -win $_nWave2
wvZoomIn -win $_nWave2
wvZoomIn -win $_nWave2
wvZoomIn -win $_nWave2
wvZoomIn -win $_nWave2
wvSetCursor -win $_nWave2 963285384.704198 -snap {("G6" 0)}
verdiWindowResize -win $_Verdi_1 "842" "171" "900" "700"
wvSetCursor -win $_nWave2 731216768.894708 -snap {("G6" 0)}
wvSetCursor -win $_nWave2 731215158.703541 -snap {("G6" 1)}
wvZoomIn -win $_nWave2
wvZoomOut -win $_nWave2
wvSaveSignal -win $_nWave2 \
           "/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/Simulaciones/verdiLog/novas_autosave.ses.wave.0"
debExit
