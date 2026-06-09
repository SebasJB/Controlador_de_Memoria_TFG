gui_set_pref_value -category {coveragesetting} -key {geninfodumping} -value 1
gui_exclusion -set_force true
gui_assert_mode -mode flat
gui_class_mode -mode hier
gui_excl_mgr_flat_list -on  0
gui_covdetail_select -id  CovDetail.1   -name   Line
verdiWindowWorkMode -win $_vdCoverage_1 -coverageAnalysis
gui_open_cov  -hier cov_n4_lat1.vdb -testdir {} -test {cov_n4_lat1/mem_full_test_CFG-A1} -merge MergedTest -db_max_tests 10 -fsm transition
verdiDockWidgetDisplay -dock widgetDock_Message
verdiDockWidgetHide -dock widgetDock_Message
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top
gui_exclusion_file -load -file { /mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el }
verdiDockWidgetDisplay -dock widgetDock_Message
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_axi_front_end.u_rd_formatter   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_axi_front_end.u_rd_formatter  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tgl { {s_axi_araddr[31:0]}  rd_req_full   }
gui_list_select -id CovDetail.1 -list tgl { rd_req_full  {rd_req_data[32:0]}   }
gui_list_action -id  CovDetail.1 -list {tgl} {rd_req_data[32:0]}
gui_list_select -id CovDetail.1 -list tglDetail { {rd_req_data[32:16]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {rd_req_data[32]}
gui_list_select -id CovDetail.1 -list tglDetail { {rd_req_data[32]}   }
gui_list_action -id  CovDetail.1 -list {tglDetail} {rd_req_data[32:16]}
gui_list_select -id CovDetail.1 -list tglDetail { {rd_req_data[32:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_axi_front_end.u_rd_formatter  tb_top.dut.u_rd_addr_decoder   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_rd_addr_decoder  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_addr[31:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CovDetail.1 -list tgl { {req_addr[31:0]}  {req_word_addr[29:0]}   }
gui_list_action -id  CovDetail.1 -list {tgl} {req_word_addr[29:0]}
gui_exclude_items -id  CovDetail.1  -list { tgl }  -selected
gui_exclude_items -id  CovDetail.1  -list { tgl }  -selected  -include
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_rd_addr_decoder  tb_top.dut.u_wr_addr_decoder   }
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder
gui_list_expand -id CoverageTable.1   tb_top.dut.u_wr_addr_decoder
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_wr_addr_decoder  -column {Toggle} 
gui_list_select -id CovDetail.1 -list tglDetail { {req_addr[31:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CovDetail.1 -list tgl { {req_addr[31:0]}  {req_word_addr[29:0]}   }
gui_list_action -id  CovDetail.1 -list {tgl} {req_word_addr[29:0]}
gui_list_sort -id  CovDetail.1   -list {tglDetail} {Variable}
gui_list_select -id CovDetail.1 -list tglDetail { {req_word_addr[29:14]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
verdiDockWidgetHide -dock widgetDock_Message
verdiDockWidgetHide -dock widgetDock_<CovSrc.1>
gui_list_collapse -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end
gui_list_expand -id  CoverageTable.1   -list {covtblInstancesList} tb_top.dut.u_axi_front_end
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder  tb_top.dut.u_axi_front_end   }
gui_list_action -id  CoverageTable.1 -list {covtblInstancesList} tb_top.dut.u_axi_front_end  -column {Toggle} 
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_axi_front_end  tb_top.dut.u_wr_addr_decoder   }
gui_list_select -id CovDetail.1 -list tgl { rst_n  {aw_hold_addr[31:0]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {aw_hold_addr[31:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_list_select -id CovDetail.1 -list tgl { {aw_hold_addr[31:0]}  {rd_req_data[32:0]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {rd_req_data[32:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
verdiDockWidgetDisplay -dock widgetDock_Message
gui_list_select -id CovDetail.1 -list tgl { {rd_req_data[32:0]}  {aw_hold_addr[31:0]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {aw_hold_addr[31:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected  -include
gui_list_select -id CovDetail.1 -list tgl { {aw_hold_addr[31:0]}  {rd_req_data[32:0]}   }
gui_list_select -id CovDetail.1 -list tglDetail { {rd_req_data[32:16]}   }
gui_exclude_items -id  CovDetail.1  -list { tglDetail }  -selected  -include
gui_exclusion_file -save_all -file {/mnt/vol_NFS_rh003/estudiantes/TFG_Sebastian_Barrantes_2026/Controlador_de_Memoria_TFG/Ambiente/cov_exclusions.el}
verdiDockWidgetHide -dock widgetDock_<CovSrc.1>
verdiDockWidgetHide -dock widgetDock_Message
gui_list_select -id CovDetail.1 -list tgl { {rd_req_data[32:0]}  {aw_hold_addr[31:0]}   }
gui_list_select -id CoverageTable.1 -list covtblInstancesList { tb_top.dut.u_wr_addr_decoder   }
vdCovExit -noprompt
