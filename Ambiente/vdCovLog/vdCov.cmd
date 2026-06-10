gui_set_pref_value -category {coveragesetting} -key {geninfodumping} -value 1
gui_exclusion -set_force true
gui_assert_mode -mode flat
gui_class_mode -mode hier
gui_excl_mgr_flat_list -on  0
gui_covdetail_select -id  CovDetail.1   -name   Line
verdiWindowWorkMode -win $_vdCoverage_1 -coverageAnalysis
gui_open_cov  -hier Simulaciones.vdb -testdir {} -test {Simulaciones/cov_merged} -merge MergedTest -db_max_tests 10 -fsm transition
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut  -column {FSM} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut  {tb_top.dut.gen_bank[0].u_bank}   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank}
gui_list_expand -id CoverageTable.1   {tb_top.dut.gen_bank[0].u_bank}
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank}  -column {FSM} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[0].u_bank}  {tb_top.dut.gen_bank[0].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank.u_fsm}  -column {} 
gui_covdetail_select -id  CovDetail.1   -fsmMode   Sequence
gui_covdetail_select -id  CovDetail.1   -fsmMode   Transition
gui_tbl_select -id CovDetail.1   { {3,1,3,1} }
gui_tbl_select -id CovDetail.1   { {2,1,2,1} }
gui_exclude_items -id  CovDetail.1  -table { fsmTable }  -selected
gui_tbl_select -id CovDetail.1   { {3,1,3,1} }
gui_exclude_items -id  CovDetail.1  -table { fsmTable }  -selected
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank}
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[0].u_bank.u_fsm}  {tb_top.dut.gen_bank[0].u_bank}   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[0].u_bank}  {tb_top.dut.gen_sram[0].u_sram}   }
gui_exclude_items -id  CoverageTable.1  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_sram[0].u_sram}  {tb_top.dut.gen_sram[1].u_sram}   }
gui_exclude_items -id  CoverageTable.1  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_sram[1].u_sram}  {tb_top.dut.gen_sram[2].u_sram}   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_sram[3].u_sram}   }
gui_exclude_items -id  CoverageTable.1  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_sram[2].u_sram}  {tb_top.dut.gen_sram[3].u_sram}  {tb_top.dut.gen_bank[3].u_bank}   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank}
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[3].u_bank}  {tb_top.dut.gen_bank[1].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank.u_fsm}  -column {FSM} 
gui_tbl_select -id CovDetail.1   { {2,1,2,1} }
gui_tbl_select -id CovDetail.1   { {2,1,2,1} {3,1,3,1} }
gui_exclude_items -id  CovDetail.1  -table { fsmTable }  -selected
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank}
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[2].u_bank}
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[1].u_bank.u_fsm}  {tb_top.dut.gen_bank[2].u_bank.u_lat_cnt}   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[2].u_bank.u_lat_cnt}  {tb_top.dut.gen_bank[2].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[2].u_bank.u_fsm}  -column {FSM} 
gui_tbl_select -id CovDetail.1   { {3,1,3,1} }
gui_tbl_select -id CovDetail.1   { {3,1,3,1} {2,1,2,1} }
gui_exclude_items -id  CovDetail.1  -table { fsmTable }  -selected
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[2].u_bank}
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[2].u_bank.u_fsm}  {tb_top.dut.gen_bank[2].u_bank}   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[3].u_bank}
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[2].u_bank}  {tb_top.dut.gen_bank[3].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[3].u_bank.u_fsm}  -column {FSM} 
gui_tbl_select -id CovDetail.1   { {2,1,2,1} }
gui_tbl_select -id CovDetail.1   { {2,1,2,1} {3,1,3,1} }
gui_exclude_items -id  CovDetail.1  -table { fsmTable }  -selected
gui_exclusion_file -save -file { /mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el } -module -incremental -format newformat
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[3].u_bank.u_fsm}  {tb_top.dut.gen_bank[3].u_bank.u_lat_cnt}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[3].u_bank.u_lat_cnt}  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tgl { rst_n  {cnt[7:0]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {cnt[7:2]}  -tgl_transition 1to0
gui_list_select -id CovDetail.1 -list tglDetail { {cnt[7:2]}   }
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[3].u_bank}
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[3].u_bank.u_lat_cnt}  {tb_top.dut.gen_bank[3].u_bank}   }
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[3].u_bank}  tb_top   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top  uvm_custom_install_recording   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { uvm_custom_install_verdi_recording   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { prll_d_ltch_no_rst   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { prll_d_ltch   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { fifo_flops   }
gui_exclude_items -id  CoverageTable.1  -selected
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
gui_list_select -id CovDetail.1 -list tgl { {cnt[7:0]}  rst_n   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { fifo_flops  prll_d_ltch  prll_d_ltch_no_rst  uvm_custom_install_recording  uvm_custom_install_verdi_recording   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.fifo_probe   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_req_fifo   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_req_fifo   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_response_path
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_response_path.u_rd_resp_fifo   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_response_path.u_rd_response_path_sva   }
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_response_path
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_response_path
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_response_path
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_scheduler
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_response_path
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_scheduler.u_scheduler_sva   }
gui_exclude_items -id  CoverageTable.1  -selected
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_req_fifo  tb_top.dut.u_rd_response_path.u_rd_resp_fifo  tb_top.dut.u_rd_response_path.u_rd_response_path_sva  tb_top.dut.u_scheduler.u_scheduler_sva  tb_top.dut.u_wr_req_fifo  tb_top.fifo_probe  tb_top.dut   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut  tb_top.sched_probe   }
gui_exclude_items -id  CoverageTable.1  -selected
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_response_path
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.sched_probe  tb_top.dut.u_wr_response_path.u_wr_response_path_sva   }
gui_exclude_items -id  CoverageTable.1  -selected
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_scheduler
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_response_path
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_response_path.u_wr_response_path_sva  tb_top.dut.u_axi_front_end.u_axi4_lite_front_end_sva   }
gui_exclude_items -id  CoverageTable.1  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_axi_front_end.u_axi4_lite_front_end_sva  tb_top.dut.u_axi_front_end   }
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_axi_front_end  tb_top.bank_probe   }
gui_exclude_items -id  CoverageTable.1  -selected
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.bank_probe   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder  tb_top.dut.u_wr_addr_decoder.u_bank_selector   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_bank_selector  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[29]}
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[29:14]}
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29:14]}  -tgl_transition 0to1
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29:14]}  -tgl_transition 1to0
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29:14]}  -tgl_transition 0to1  -include
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29:14]}  -tgl_transition 0to1
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29:14]}  -tgl_transition 1to0  -include
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29:14]}  -tgl_transition 0to1  -include
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29:14]}  -tgl_transition 1to0
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_bank_selector  tb_top.dut.u_wr_addr_decoder   }
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_response_path
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder  tb_top.dut.u_rd_addr_decoder.u_bank_selector   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder.u_bank_selector  tb_top.dut.u_rd_addr_decoder   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder  tb_top.dut.u_rd_addr_decoder.u_bank_selector   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder.u_bank_selector  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_exclusion_file -save -file { /mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el } -module -incremental -format newformat
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder.u_bank_selector  tb_top.dut.u_wr_addr_decoder.u_bank_selector   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_bank_selector  -column {Toggle} 
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29:14]}  -tgl_transition 0to1
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29:14]}  -tgl_transition 0to1  -include
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29:14]}  -tgl_transition 0to1
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[29]}
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29]}  -tgl_transition 0to1  -include
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail } { {req_word_addr[29]} }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -include { {req_word_addr[29]} }
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29]}  -tgl_transition 1to0  -include
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[29]}
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[29]}
gui_list_sort -id  CovDetail.1   -list {tglDetail} { }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail } { {req_word_addr[29]} }
gui_list_sort -id  CovDetail.1   -list {tglDetail} { }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_bank_selector  tb_top.dut.u_wr_addr_decoder.u_offset_remover   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_offset_remover  -column {Toggle} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_offset_remover  tb_top.dut.u_wr_addr_decoder.u_bank_selector   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_bank_selector  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[29:14]}  -tgl_transition 1to0  -include
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[29]}
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[29:14]}
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_bank_selector  tb_top.dut.u_rd_addr_decoder.u_bank_selector   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder.u_bank_selector  -column {Toggle} 
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder.u_bank_selector  tb_top.dut.u_rd_addr_decoder   }
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder  tb_top.dut.u_wr_addr_decoder.u_bank_selector   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_bank_selector  -column {Toggle} 
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[28:14]}  -tgl_transition 1to0
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[28:14]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[28:14]}  -tgl_transition 1to0  -include
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[28]}
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[28]}  -tgl_transition 1to0
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[28]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[28]}
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[29]}
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -include { {req_word_addr[29]} }
gui_exclude_items -id  CovDetail.1  -list { tglDetail } { {req_word_addr[29]} }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -include { {req_word_addr[29]} }
gui_exclude_items -id  CovDetail.1  -list { tglDetail } { {req_word_addr[29]} }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -include { {req_word_addr[29]} }
gui_list_action -id  CovDetail.1 -list {tgl} {req_word_addr[29:0]}
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[14]}  -tgl_transition 1to0
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[14]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[14]}  -tgl_transition 1to0  -include
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[14]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[15]}  -tgl_transition 1to0
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[15]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_word_addr[16]}  -tgl_transition 1to0
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[16]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[14]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[17]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[18]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[19]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[20]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[21]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[22]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[23]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[24]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[25]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[26]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[26]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[27]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[27]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[25]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[26]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[25]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[27]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[28]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_bank_selector  tb_top.dut.u_wr_addr_decoder.u_offset_remover   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_offset_remover  -column {Toggle} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_offset_remover  tb_top.dut.u_wr_addr_decoder.u_range_checker   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_range_checker  -column {Toggle} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_range_checker  tb_top.dut.u_wr_addr_decoder.u_offset_remover   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_offset_remover  -column {Toggle} 
gui_exclude_items  -id CovDetail.1  -list tglDetail {req_addr[31:18]}  -tgl_transition 1to0
gui_list_select -id CovDetail.1 -list tglDetail { {req_addr[31:18]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_offset_remover  tb_top.dut.u_wr_addr_decoder.u_range_checker   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_range_checker  tb_top.dut.u_wr_addr_decoder.u_local_addr_gen   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_local_addr_gen  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_local_addr_gen  tb_top.dut.u_rd_addr_decoder.u_range_checker   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder.u_range_checker  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder.u_range_checker  tb_top.dut.u_rd_addr_decoder.u_offset_remover   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder.u_offset_remover  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_addr[31:18]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder.u_offset_remover  tb_top.dut.u_rd_addr_decoder.u_local_addr_gen   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder.u_local_addr_gen  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder.u_local_addr_gen  tb_top.dut.u_rd_addr_decoder.u_offset_remover   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder.u_offset_remover  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder.u_offset_remover  tb_top.dut.u_wr_addr_decoder.u_offset_remover   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_offset_remover  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_addr[31:18]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_offset_remover  tb_top.dut.u_wr_addr_decoder.u_range_checker   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_range_checker  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_range_checker  tb_top.dut.u_rd_addr_decoder   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_addr[31:16]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_addr[31]}
gui_list_select -id CovDetail.1 -list tglDetail { {req_addr[31]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_addr[31:16]}
gui_list_select -id CovDetail.1 -list tglDetail { {req_addr[31:16]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_addr[31:16]}  {req_addr[15:0]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {req_addr[15:0]}  {req_addr[31:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder  tb_top.dut.u_wr_addr_decoder   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder
gui_list_expand -id CoverageTable.1   tb_top.dut.u_wr_addr_decoder
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_addr[31:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CovDetail.1 -list tgl { {req_addr[31:0]}  {req_word_addr[29:0]}   }
gui_list_action -id  CovDetail.1 -list {tgl} {req_word_addr[29:0]}
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder  tb_top.dut.u_wr_addr_decoder.u_offset_remover   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_offset_remover  -column {Toggle} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_offset_remover  tb_top.dut.u_rd_addr_decoder   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_expand -id CoverageTable.1   tb_top.dut.u_rd_addr_decoder
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[29]}
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {req_word_addr[29:14]}
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder  tb_top.dut.u_wr_addr_decoder.u_offset_remover   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder.u_offset_remover  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder.u_offset_remover  tb_top.dut.u_wr_addr_decoder   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder  tb_top.dut.u_axi_front_end.u_aw_capture_reg   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_axi_front_end.u_aw_capture_reg  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tgl { rst_n  {s_axi_awaddr[31:0]}   }
gui_list_select -id CovDetail.1 -list tgl { {s_axi_awaddr[31:0]}  {aw_hold_addr[31:0]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {aw_hold_addr[31:18]}   }
gui_list_select -id CovDetail.1 -list tgl { {aw_hold_addr[31:0]}  {s_axi_awaddr[31:0]}   }
gui_list_action -id  CovDetail.1 -list {tgl} {s_axi_awaddr[31:0]}
gui_list_select -id CovDetail.1 -list tglDetail { {s_axi_awaddr[31:18]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CovDetail.1 -list tgl { {s_axi_awaddr[31:0]}  {aw_hold_addr[31:0]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {aw_hold_addr[31:18]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CovDetail.1 -list tgl { {aw_hold_addr[31:0]}  {s_axi_awaddr[31:0]}   }
gui_list_select -id CovDetail.1 -list tgl { {s_axi_awaddr[31:0]}  rst_n   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_axi_front_end.u_aw_capture_reg  tb_top.dut.u_axi_front_end.u_rd_formatter   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_axi_front_end.u_rd_formatter  -column {Toggle} 
gui_exclude_items  -id CovDetail.1  -list tglDetail {s_axi_araddr[31:18]}  -tgl_transition 0to1
gui_list_select -id CovDetail.1 -list tglDetail { {s_axi_araddr[31:18]}   }
gui_exclude_items  -id CovDetail.1  -list tglDetail {s_axi_araddr[31:18]}  -tgl_transition 0to1  -include
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CovDetail.1 -list tgl { {s_axi_araddr[31:0]}  {rd_req_data[32:0]}   }
gui_list_action -id  CovDetail.1 -list {tgl} {rd_req_data[32:0]}
gui_list_select -id CovDetail.1 -list tglDetail { {rd_req_data[32:18]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {rd_req_data[32]}
gui_list_select -id CovDetail.1 -list tglDetail { {rd_req_data[32]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {rd_req_data[32:18]}
gui_list_select -id CovDetail.1 -list tglDetail { {rd_req_data[32:18]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_axi_front_end.u_rd_formatter  tb_top.dut.u_axi_front_end.u_wr_assembler   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end.u_wr_assembler
gui_list_expand -id CoverageTable.1   tb_top.dut.u_axi_front_end.u_wr_assembler
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_axi_front_end.u_wr_assembler  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {aw_hold_addr[31:18]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_exclusion_file -save -file { /mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el } -module -incremental -format newformat
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_axi_front_end.u_wr_assembler  tb_top.dut.u_axi_front_end   }
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_axi_front_end  tb_top.dut   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut  {tb_top.dut.gen_bank[0].u_bank}   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank}
gui_list_expand -id CoverageTable.1   {tb_top.dut.gen_bank[0].u_bank}
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank}  -column {Branch} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[0].u_bank}  {tb_top.dut.gen_bank[0].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank.u_fsm}  -column {Branch} 
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6  Branch0.10   }
gui_list_select -id CovDetail.1 -list branch { Branch0.10  MISSING_ELSE0.6   }
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6  Branch0.10   }
gui_list_select -id CovDetail.1 -list branch { Branch0.10  Branch0.9   }
gui_list_select -id CovDetail.1 -list branch { Branch0.9  Branch0.10   }
gui_list_select -id CovDetail.1 -list branch { Branch0.10  MISSING_ELSE0.6   }
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6  Branch0.10   }
gui_list_collapse -id  CovDetail.1   -list {branch} Item0
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[0].u_bank.u_fsm}  {tb_top.dut.gen_bank[1].u_bank}   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank}
gui_list_expand -id CoverageTable.1   {tb_top.dut.gen_bank[1].u_bank}
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank}  -column {Branch} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[1].u_bank}  {tb_top.dut.gen_bank[1].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank.u_fsm}  -column {Branch} 
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank.u_fsm}  -column {Branch} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[1].u_bank.u_fsm}  {tb_top.dut.gen_bank[1].u_bank}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank}  -column {Branch} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[1].u_bank}  {tb_top.dut.gen_bank[2].u_bank}   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[2].u_bank}
gui_list_expand -id CoverageTable.1   {tb_top.dut.gen_bank[2].u_bank}
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[2].u_bank}  -column {Branch} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[2].u_bank}  {tb_top.dut.gen_bank[3].u_bank}   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} {tb_top.dut.gen_bank[3].u_bank}
gui_list_expand -id CoverageTable.1   {tb_top.dut.gen_bank[3].u_bank}
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[3].u_bank}  -column {Branch} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[3].u_bank}  {tb_top.dut.gen_bank[3].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[3].u_bank.u_fsm}  -column {Branch} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[3].u_bank.u_fsm}  {tb_top.dut.gen_bank[0].u_bank}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank}  -column {Branch} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[0].u_bank}  {tb_top.dut.gen_bank[0].u_bank.u_capture}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank.u_capture}  -column {Branch} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[0].u_bank.u_capture}  {tb_top.dut.gen_bank[0].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank.u_fsm}  -column {Branch} 
gui_covtable_show -show  { Design Hierarchy } -id  CoverageTable.1  -test  MergedTest
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6  Branch0.10   }
gui_list_select -id CovDetail.1 -list branch { Branch0.10  MISSING_ELSE0.6   }
gui_list_action -id  CovDetail.1 -list {branch} MISSING_ELSE0.6  -column {Name} 
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6   }
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6   }
gui_list_action -id  CovDetail.1 -list {branch} MISSING_ELSE0.6  -column {Name} 
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6   }
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6   }
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6  Branch0.10   }
gui_list_action -id  CovDetail.1 -list {branch} Branch0.10  -column {Coverage} 
gui_list_select -id CovDetail.1 -list branch { Branch0.10   }
gui_list_select -id CovDetail.1 -list branch { Branch0.10   }
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6   }
gui_exclude_items -id  CovDetail.1  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[0].u_bank.u_fsm}  {tb_top.dut.gen_bank[1].u_bank.u_fsm}   }
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6   }
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank.u_fsm}  -column {Branch} 
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6   }
gui_list_select -id CovDetail.1 -list branch { MISSING_ELSE0.6   }
gui_list_select -id CovDetail.1 -list branch { Branch0.10   }
gui_exclude_items -id  CovDetail.1  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[1].u_bank.u_fsm}  {tb_top.dut.gen_bank[2].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[2].u_bank.u_fsm}  -column {Branch} 
gui_list_select -id CovDetail.1 -list branch { Branch0.10   }
gui_exclude_items -id  CovDetail.1  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[2].u_bank.u_fsm}  {tb_top.dut.gen_bank[3].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[3].u_bank.u_fsm}  -column {Branch} 
gui_list_select -id CovDetail.1 -list branch { Branch0.10   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[3].u_bank.u_fsm}  -column {Branch} 
gui_list_select -id CovDetail.1 -list branch { Branch0.10   }
gui_exclude_items -id  CovDetail.1  -selected
gui_exclusion_file -save -file { /mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el } -module -incremental -format newformat
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[3].u_bank.u_fsm}  {tb_top.dut.gen_bank[0].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank.u_fsm}  -column {Branch} 
gui_list_select -id CovDetail.1 -list branch { Branch1.4  Branch1.2   }
gui_list_select -id CovDetail.1 -list branch { Branch1.2  Branch1.4   }
gui_list_action -id  CovDetail.1 -list {branch} Branch1.4  -column {Coverage} 
gui_list_select -id CovDetail.1 -list branch { Branch1.4   }
gui_list_select -id CovDetail.1 -list branch { Branch1.4   }
gui_list_action -id  CovDetail.1 -list {branch} Branch1.4  -column {Coverage} 
gui_list_select -id CovDetail.1 -list branch { Branch1.4   }
gui_list_select -id CovDetail.1 -list branch { Branch1.4   }
gui_exclude_items -id  CovDetail.1  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[0].u_bank.u_fsm}  {tb_top.dut.gen_bank[3].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[3].u_bank.u_fsm}  -column {Branch} 
gui_exclude_items -id  CovDetail.1  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[3].u_bank.u_fsm}  {tb_top.dut.gen_bank[2].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[2].u_bank.u_fsm}  -column {Branch} 
gui_exclude_items -id  CovDetail.1  -selected
gui_exclusion_file -save -file { /mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el } -module -incremental -format newformat
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[2].u_bank.u_fsm}  {tb_top.dut.gen_bank[1].u_bank.u_fsm}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank.u_fsm}  -column {Branch} 
gui_list_action -id  CovDetail.1 -list {branch} Branch1.4  -column {Coverage} 
gui_list_select -id CovDetail.1 -list branch { Branch1.4   }
gui_list_select -id CovDetail.1 -list branch { Branch1.4   }
gui_exclude_items -id  CovDetail.1  -selected
gui_exclusion_file -save -file { /mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el } -module -incremental -format newformat
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[1].u_bank.u_fsm}  tb_top   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top  -column {Branch} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top  {tb_top.dut.gen_bank[0].u_bank.u_lat_cnt}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank.u_lat_cnt}  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tgl { rst_n  {cnt[7:0]}   }
gui_list_action -id  CovDetail.1 -list {tgl} {cnt[7:0]}
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank.u_lat_cnt}  -column {} 
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank.u_lat_cnt}  -column {Toggle} 
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank.u_lat_cnt}  -column {Toggle} 
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank.u_lat_cnt}  -column {Toggle} 
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[0].u_bank.u_lat_cnt}  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tgl { rst_n  {cnt[7:0]}   }
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[0].u_bank.u_lat_cnt}  tb_top.dut   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut  {tb_top.dut.gen_bank[1].u_bank}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank}  -column {} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { {tb_top.dut.gen_bank[1].u_bank}  {tb_top.dut.gen_bank[1].u_bank.u_lat_cnt}   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} {tb_top.dut.gen_bank[1].u_bank.u_lat_cnt}  -column {} 
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
vdCovExit -noprompt
