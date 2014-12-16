#ChipScope Core Inserter Project File Version 3.0
#Tue Dec 16 15:49:21 MSK 2014
Project.device.designInputFile=/home/vladimir/CERN/CERNLAST/v6pcieDMA_cs.ngc
Project.device.designOutputFile=/home/vladimir/CERN/CERNLAST/v6pcieDMA_cs.ngc
Project.device.deviceFamily=21
Project.device.enableRPMs=true
Project.device.outputDirectory=/home/vladimir/CERN/CERNLAST/_ngo
Project.device.useSRL16=true
Project.filter.dimension=19
Project.filter<0>=BUFG
Project.filter<10>=serdes_clk_buf
Project.filter<11>=clk_fb
Project.filter<12>=dco_clk
Project.filter<13>=serdes_clk
Project.filter<14>=trn_clk
Project.filter<15>=reg08_rv
Project.filter<16>=reg08_td
Project.filter<17>=reg08_rd
Project.filter<18>=reg08_tv
Project.filter<1>=
Project.filter<2>=send_ack
Project.filter<3>=control_iic<0>
Project.filter<4>=reg07_rv
Project.filter<5>=reg07_tv
Project.filter<6>=reset
Project.filter<7>=indicator
Project.filter<8>=reg08_td<0>
Project.filter<9>=fs_clk
Project.icon.boundaryScanChain=1
Project.icon.enableExtTriggerIn=false
Project.icon.enableExtTriggerOut=false
Project.icon.triggerInPinName=
Project.icon.triggerOutPinName=
Project.unit.dimension=1
Project.unit<0>.clockChannel=cmp_fmc_adc_100Ms_core clk_fb
Project.unit<0>.clockEdge=Rising
Project.unit<0>.dataChannel<0>=cmp_fmc_adc_100Ms_core send_ack
Project.unit<0>.dataChannel<10>=cmp_fmc_adc_100Ms_core reg07_rd_6
Project.unit<0>.dataChannel<11>=cmp_fmc_adc_100Ms_core reg07_rd_5
Project.unit<0>.dataChannel<12>=cmp_fmc_adc_100Ms_core reg07_rd_4
Project.unit<0>.dataChannel<13>=cmp_fmc_adc_100Ms_core reg07_rd_3
Project.unit<0>.dataChannel<14>=cmp_fmc_adc_100Ms_core reg07_rd_2
Project.unit<0>.dataChannel<15>=cmp_fmc_adc_100Ms_core reg07_rd_1
Project.unit<0>.dataChannel<16>=cmp_fmc_adc_100Ms_core reg07_rd_0
Project.unit<0>.dataChannel<17>=cmp_fmc_adc_100Ms_core send_ack
Project.unit<0>.dataChannel<18>=cmp_fmc_adc_100Ms_core start
Project.unit<0>.dataChannel<19>=cmp_fmc_adc_100Ms_core stop
Project.unit<0>.dataChannel<1>=cmp_fmc_adc_100Ms_core reg07_td<7>
Project.unit<0>.dataChannel<20>=cmp_fmc_adc_100Ms_core read
Project.unit<0>.dataChannel<21>=cmp_fmc_adc_100Ms_core write
Project.unit<0>.dataChannel<22>=cmp_fmc_adc_100Ms_core free
Project.unit<0>.dataChannel<23>=cmp_fmc_adc_100Ms_core rec_ack
Project.unit<0>.dataChannel<24>=cmp_fmc_adc_100Ms_core ready
Project.unit<0>.dataChannel<25>=cmp_fmc_adc_100Ms_core i2c_master_si570 i_sda_mstr
Project.unit<0>.dataChannel<26>=cmp_fmc_adc_100Ms_core i2c_master_si570 i_scl_mstr
Project.unit<0>.dataChannel<27>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<7>
Project.unit<0>.dataChannel<28>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<6>
Project.unit<0>.dataChannel<29>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<5>
Project.unit<0>.dataChannel<2>=cmp_fmc_adc_100Ms_core reg07_td<6>
Project.unit<0>.dataChannel<30>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<4>
Project.unit<0>.dataChannel<31>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<3>
Project.unit<0>.dataChannel<32>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<2>
Project.unit<0>.dataChannel<33>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<1>
Project.unit<0>.dataChannel<34>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<0>
Project.unit<0>.dataChannel<3>=cmp_fmc_adc_100Ms_core reg07_td<5>
Project.unit<0>.dataChannel<4>=cmp_fmc_adc_100Ms_core reg07_td<4>
Project.unit<0>.dataChannel<5>=cmp_fmc_adc_100Ms_core reg07_td<3>
Project.unit<0>.dataChannel<6>=cmp_fmc_adc_100Ms_core reg07_td<2>
Project.unit<0>.dataChannel<7>=cmp_fmc_adc_100Ms_core reg07_td<1>
Project.unit<0>.dataChannel<8>=cmp_fmc_adc_100Ms_core reg07_td<0>
Project.unit<0>.dataChannel<9>=cmp_fmc_adc_100Ms_core reg07_rd_7
Project.unit<0>.dataDepth=1024
Project.unit<0>.dataEqualsTrigger=true
Project.unit<0>.dataPortWidth=35
Project.unit<0>.enableGaps=false
Project.unit<0>.enableStorageQualification=true
Project.unit<0>.enableTimestamps=false
Project.unit<0>.timestampDepth=0
Project.unit<0>.timestampWidth=0
Project.unit<0>.triggerChannel<0><0>=cmp_fmc_adc_100Ms_core send_ack
Project.unit<0>.triggerChannel<1><0>=cmp_fmc_adc_100Ms_core reg07_td<7>
Project.unit<0>.triggerChannel<1><1>=cmp_fmc_adc_100Ms_core reg07_td<6>
Project.unit<0>.triggerChannel<1><2>=cmp_fmc_adc_100Ms_core reg07_td<5>
Project.unit<0>.triggerChannel<1><3>=cmp_fmc_adc_100Ms_core reg07_td<4>
Project.unit<0>.triggerChannel<1><4>=cmp_fmc_adc_100Ms_core reg07_td<3>
Project.unit<0>.triggerChannel<1><5>=cmp_fmc_adc_100Ms_core reg07_td<2>
Project.unit<0>.triggerChannel<1><6>=cmp_fmc_adc_100Ms_core reg07_td<1>
Project.unit<0>.triggerChannel<1><7>=cmp_fmc_adc_100Ms_core reg07_td<0>
Project.unit<0>.triggerChannel<2><0>=cmp_fmc_adc_100Ms_core reg07_rd_7
Project.unit<0>.triggerChannel<2><1>=cmp_fmc_adc_100Ms_core reg07_rd_6
Project.unit<0>.triggerChannel<2><2>=cmp_fmc_adc_100Ms_core reg07_rd_5
Project.unit<0>.triggerChannel<2><3>=cmp_fmc_adc_100Ms_core reg07_rd_4
Project.unit<0>.triggerChannel<2><4>=cmp_fmc_adc_100Ms_core reg07_rd_3
Project.unit<0>.triggerChannel<2><5>=cmp_fmc_adc_100Ms_core reg07_rd_2
Project.unit<0>.triggerChannel<2><6>=cmp_fmc_adc_100Ms_core reg07_rd_1
Project.unit<0>.triggerChannel<2><7>=cmp_fmc_adc_100Ms_core reg07_rd_0
Project.unit<0>.triggerChannel<3><0>=cmp_fmc_adc_100Ms_core send_ack
Project.unit<0>.triggerChannel<3><1>=cmp_fmc_adc_100Ms_core start
Project.unit<0>.triggerChannel<3><2>=cmp_fmc_adc_100Ms_core stop
Project.unit<0>.triggerChannel<3><3>=cmp_fmc_adc_100Ms_core read
Project.unit<0>.triggerChannel<3><4>=cmp_fmc_adc_100Ms_core write
Project.unit<0>.triggerChannel<3><5>=cmp_fmc_adc_100Ms_core free
Project.unit<0>.triggerChannel<3><6>=cmp_fmc_adc_100Ms_core rec_ack
Project.unit<0>.triggerChannel<3><7>=cmp_fmc_adc_100Ms_core ready
Project.unit<0>.triggerChannel<3><8>=cmp_fmc_adc_100Ms_core i2c_master_si570 i_sda_mstr
Project.unit<0>.triggerChannel<3><9>=cmp_fmc_adc_100Ms_core i2c_master_si570 i_scl_mstr
Project.unit<0>.triggerChannel<4><0>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<7>
Project.unit<0>.triggerChannel<4><1>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<6>
Project.unit<0>.triggerChannel<4><2>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<5>
Project.unit<0>.triggerChannel<4><3>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<4>
Project.unit<0>.triggerChannel<4><4>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<3>
Project.unit<0>.triggerChannel<4><5>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<2>
Project.unit<0>.triggerChannel<4><6>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<1>
Project.unit<0>.triggerChannel<4><7>=cmp_fmc_adc_100Ms_core i2c_master_si570 mstr_dout<0>
Project.unit<0>.triggerConditionCountWidth=0
Project.unit<0>.triggerMatchCount<0>=1
Project.unit<0>.triggerMatchCount<1>=1
Project.unit<0>.triggerMatchCount<2>=1
Project.unit<0>.triggerMatchCount<3>=1
Project.unit<0>.triggerMatchCount<4>=1
Project.unit<0>.triggerMatchCountWidth<0><0>=0
Project.unit<0>.triggerMatchCountWidth<1><0>=0
Project.unit<0>.triggerMatchCountWidth<2><0>=0
Project.unit<0>.triggerMatchCountWidth<3><0>=0
Project.unit<0>.triggerMatchCountWidth<4><0>=0
Project.unit<0>.triggerMatchType<0><0>=1
Project.unit<0>.triggerMatchType<1><0>=1
Project.unit<0>.triggerMatchType<2><0>=1
Project.unit<0>.triggerMatchType<3><0>=1
Project.unit<0>.triggerMatchType<4><0>=1
Project.unit<0>.triggerPortCount=5
Project.unit<0>.triggerPortIsData<0>=true
Project.unit<0>.triggerPortIsData<1>=true
Project.unit<0>.triggerPortIsData<2>=true
Project.unit<0>.triggerPortIsData<3>=true
Project.unit<0>.triggerPortIsData<4>=true
Project.unit<0>.triggerPortWidth<0>=1
Project.unit<0>.triggerPortWidth<1>=8
Project.unit<0>.triggerPortWidth<2>=8
Project.unit<0>.triggerPortWidth<3>=10
Project.unit<0>.triggerPortWidth<4>=8
Project.unit<0>.triggerSequencerLevels=16
Project.unit<0>.triggerSequencerType=1
Project.unit<0>.type=ilapro