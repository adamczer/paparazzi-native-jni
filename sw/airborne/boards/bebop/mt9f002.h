#ifndef MT9F002_H
#define MT9F002_H

#define MT9F002_NUM_OF_REGISTERS 367

#define MT9F002_MODEL_ID     0x0000
#define MT9F002_REVISION_NUMBER     0x0002
#define MT9F002_MANUFACTURER_ID     0x0003
#define MT9F002_SMIA_VERSION     0x0004
#define MT9F002_FRAME_COUNT     0x0005
#define MT9F002_PIXEL_ORDER     0x0006
#define MT9F002_DATA_PEDESTAL     0x0008
#define MT9F002_FRAME_FORMAT_MODEL_TYPE     0x0040
#define MT9F002_FRAME_FORMAT_MODEL_SUBTYPE     0x0041
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_0     0x0042
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_1     0x0044
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_2     0x0046
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_3     0x0048
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_4     0x004A
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_5     0x004C
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_6     0x004E
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_7     0x0050
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_8     0x0052
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_9     0x0054
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_10     0x0056
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_11     0x0058
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_12     0x005A
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_13     0x005C
#define MT9F002_FRAME_FORMAT_DESCRIPTOR_14     0x005E
#define MT9F002_ANALOG_GAIN_CAPABILITY     0x0080
#define MT9F002_ANALOG_GAIN_CODE_MIN     0x0084
#define MT9F002_ANALOG_GAIN_CODE_MAX     0x0086
#define MT9F002_ANALOG_GAIN_CODE_STEP     0x0088
#define MT9F002_ANALOG_GAIN_TYPE     0x008A
#define MT9F002_ANALOG_GAIN_M0     0x008C
#define MT9F002_ANALOG_GAIN_C0     0x008E
#define MT9F002_ANALOG_GAIN_M1     0x0090
#define MT9F002_ANALOG_GAIN_C1     0x0092
#define MT9F002_DATA_FORMAT_MODEL_TYPE     0x00C0
#define MT9F002_DATA_FORMAT_MODEL_SUBTYPE     0x00C1
#define MT9F002_DATA_FORMAT_DESCRIPTOR_0     0x00C2
#define MT9F002_DATA_FORMAT_DESCRIPTOR_1     0x00C4
#define MT9F002_DATA_FORMAT_DESCRIPTOR_2     0x00C6
#define MT9F002_DATA_FORMAT_DESCRIPTOR_3     0x00C8
#define MT9F002_DATA_FORMAT_DESCRIPTOR_4     0x00CA
#define MT9F002_DATA_FORMAT_DESCRIPTOR_5     0x00CC
#define MT9F002_DATA_FORMAT_DESCRIPTOR_6     0x00CE
#define MT9F002_MODE_SELECT     0x0100
#define MT9F002_IMAGE_ORIENTATION     0x0101
#define MT9F002_SOFTWARE_RESET     0x0103
#define MT9F002_GROUPED_PARAMETER_HOLD     0x0104
#define MT9F002_MASK_CORRUPTED_FRAMES     0x0105
#define MT9F002_CPP_CHANNEL_IDENTIFIER     0x0110
#define MT9F002_CPP2_SIGNALLING_MODE     0x0111
#define MT9F002_CPP_DATA_FORMAT     0x0112
#define MT9F002_GAIN_MODE     0x0120
#define MT9F002_FINE_INTEGRATION_TIME     0x0200
#define MT9F002_COARSE_INTEGRATION_TIME     0x0202
#define MT9F002_ANALOG_GAIN_CODE_GLOBAL     0x0204
#define MT9F002_ANALOG_GAIN_CODE_GREENR     0x0206
#define MT9F002_ANALOG_GAIN_CODE_RED     0x0208
#define MT9F002_ANALOG_GAIN_CODE_BLUE     0x020A
#define MT9F002_ANALOG_GAIN_CODE_GREENB     0x020C
#define MT9F002_DIGITAL_GAIN_GREENR     0x020E
#define MT9F002_DIGITAL_GAIN_RED     0x0210
#define MT9F002_DIGITAL_GAIN_BLUE     0x0212
#define MT9F002_DIGITAL_GAIN_GREENB     0x0214
#define MT9F002_VT_PIX_CLK_DIV     0x0300
#define MT9F002_VT_SYS_CLK_DIV     0x0302
#define MT9F002_PRE_PLL_CLK_DIV     0x0304
#define MT9F002_PLL_MULTIPLIER     0x0306
#define MT9F002_OP_PIX_CLK_DIV     0x0308
#define MT9F002_OP_SYS_CLK_DIV     0x030A
#define MT9F002_FRAME_LENGTH_LINES     0x0340
#define MT9F002_LINE_LENGTH_PCK     0x0342
#define MT9F002_X_ADDR_START     0x0344
#define MT9F002_Y_ADDR_START     0x0346
#define MT9F002_X_ADDR_END     0x0348
#define MT9F002_Y_ADDR_END     0x034A
#define MT9F002_X_OUTPUT_SIZE     0x034C
#define MT9F002_Y_OUTPUT_SIZE     0x034E
#define MT9F002_X_EVEN_INC     0x0380
#define MT9F002_X_ODD_INC     0x0382
#define MT9F002_Y_EVEN_INC     0x0384
#define MT9F002_Y_ODD_INC     0x0386
#define MT9F002_SCALING_MODE     0x0400
#define MT9F002_SPATIAL_SAMPLING     0x0402
#define MT9F002_SCALE_M     0x0404
#define MT9F002_SCALE_N     0x0406
#define MT9F002_COMPRESSION_MODE     0x0500
#define MT9F002_TEST_PATTERN_MODE     0x0600
#define MT9F002_TEST_DATA_RED     0x0602
#define MT9F002_TEST_DATA_GREENR     0x0604
#define MT9F002_TEST_DATA_BLUE     0x0606
#define MT9F002_TEST_DATA_GREENB     0x0608
#define MT9F002_HORIZONTAL_CURSOR_WIDTH     0x060A
#define MT9F002_HORIZONTAL_CURSOR_POSITION     0x060C
#define MT9F002_VERTICAL_CURSOR_WIDTH     0x060E
#define MT9F002_VERTICAL_CURSOR_POSITION     0x0610
#define MT9F002_INTEGRATION_TIME_CAPABILITY     0x1000
#define MT9F002_COARSE_INTEGRATION_TIME_MIN     0x1004
#define MT9F002_COARSE_INTEGRATION_TIME_MAX_MARGIN     0x1006
#define MT9F002_FINE_INTEGRATION_TIME_MIN     0x1008
#define MT9F002_FINE_INTEGRATION_TIME_MAX_MARGIN     0x100A
#define MT9F002_DIGITAL_GAIN_CAPABILITY     0x1080
#define MT9F002_DIGITAL_GAIN_MIN     0x1084
#define MT9F002_DIGITAL_GAIN_MAX     0x1086
#define MT9F002_DIGITAL_GAIN_STEP_SIZE     0x1088
#define MT9F002_MIN_EXT_CLK_FREQ_MHZ     0x1100
#define MT9F002_MAX_EXT_CLK_FREQ_MHZ     0x1104
#define MT9F002_MIN_PRE_PLL_CLK_DIV     0x1108
#define MT9F002_MAX_PRE_PLL_CLK_DIV     0x110A
#define MT9F002_MIN_PLL_IP_FREQ_MHZ     0x110C
#define MT9F002_MAX_PLL_IP_FREQ_MHZ     0x1110
#define MT9F002_MIN_PLL_MULTIPLIER     0x1114
#define MT9F002_MAX_PLL_MULTIPLIER     0x1116
#define MT9F002_MIN_PLL_OP_FREQ_MHZ     0x1118
#define MT9F002_MAX_PLL_OP_FREQ_MHZ     0x111C
#define MT9F002_MIN_VT_SYS_CLK_DIV     0x1120
#define MT9F002_MAX_VT_SYS_CLK_DIV     0x1122
#define MT9F002_MIN_VT_SYS_CLK_FREQ_MHZ     0x1124
#define MT9F002_MAX_VT_SYS_CLK_FREQ_MHZ     0x1128
#define MT9F002_MIN_VT_PIX_CLK_FREQ_MHZ     0x112C
#define MT9F002_MAX_VT_PIX_CLK_FREQ_MHZ     0x1130
#define MT9F002_MIN_VT_PIX_CLK_DIV     0x1134
#define MT9F002_MAX_VT_PIX_CLK_DIV     0x1136
#define MT9F002_MIN_FRAME_LENGTH_LINES     0x1140
#define MT9F002_MAX_FRAME_LENGTH_LINES     0x1142
#define MT9F002_MIN_LINE_LENGTH_PCK     0x1144
#define MT9F002_MAX_LINE_LENGTH_PCK     0x1146
#define MT9F002_MIN_LINE_BLANKING_PCK     0x1148
#define MT9F002_MIN_FRAME_BLANKING_LINES     0x114A
#define MT9F002_MIN_OP_SYS_CLK_DIV     0x1160
#define MT9F002_MAX_OP_SYS_CLK_DIV     0x1162
#define MT9F002_MIN_OP_SYS_CLK_FREQ_MHZ     0x1164
#define MT9F002_MAX_OP_SYS_CLK_FREQ_MHZ     0x1168
#define MT9F002_MIN_OP_PIX_CLK_DIV     0x116C
#define MT9F002_MAX_OP_PIX_CLK_DIV     0x116E
#define MT9F002_MIN_OP_PIX_CLK_FREQ_MHZ     0x1170
#define MT9F002_MAX_OP_PIX_CLK_FREQ_MHZ     0x1174
#define MT9F002_X_ADDR_MIN     0x1180
#define MT9F002_Y_ADDR_MIN     0x1182
#define MT9F002_X_ADDR_MAX     0x1184
#define MT9F002_Y_ADDR_MAX     0x1186
#define MT9F002_MIN_EVEN_INC     0x11C0
#define MT9F002_MAX_EVEN_INC     0x11C2
#define MT9F002_MIN_ODD_INC     0x11C4
#define MT9F002_MAX_ODD_INC     0x11C6
#define MT9F002_SCALING_CAPABILITY     0x1200
#define MT9F002_SCALER_M_MIN     0x1204
#define MT9F002_SCALER_M_MAX     0x1206
#define MT9F002_SCALER_N_MIN     0x1208
#define MT9F002_SCALER_N_MAX     0x120A
#define MT9F002_COMPRESSION_CAPABILITY     0x1300
#define MT9F002_MATRIX_ELEMENT_REDINRED     0x1400
#define MT9F002_MATRIX_ELEMENT_GREENINRED     0x1402
#define MT9F002_MATRIX_ELEMENT_BLUEINRED     0x1404
#define MT9F002_MATRIX_ELEMENT_REDINGREEN     0x1406
#define MT9F002_MATRIX_ELEMENT_GREENINGREEN     0x1408
#define MT9F002_MATRIX_ELEMENT_BLUEINGREEN     0x140A
#define MT9F002_MATRIX_ELEMENT_REDINBLUE     0x140C
#define MT9F002_MATRIX_ELEMENT_GREENINBLUE     0x0140E
#define MT9F002_MATRIX_ELEMENT_BLUEINBLUE     0x1410
#define MT9F002_MODEL_ID_     0x3000
#define MT9F002_Y_ADDR_START_     0x3002
#define MT9F002_X_ADDR_START_     0x3004
#define MT9F002_Y_ADDR_END_     0x3006
#define MT9F002_X_ADDR_END_     0x3008
#define MT9F002_FRAME_LENGTH_LINES_     0x300A
#define MT9F002_LINE_LENGTH_PCK_     0x300C
#define MT9F002_FINE_CORRECTION     0x3010
#define MT9F002_COARSE_INTEGRATION_TIME_     0x3012
#define MT9F002_FINE_INTEGRATION_TIME_     0x3014
#define MT9F002_ROW_SPEED     0x3016
#define MT9F002_EXTRA_DELAY     0x3018
#define MT9F002_RESET_REGISTER     0x301A
#define MT9F002_MODE_SELECT_     0x301C
#define MT9F002_IMAGE_ORIENTATION_     0x301D
#define MT9F002_DATA_PEDESTAL_     0x301E
#define MT9F002_SOFTWARE_RESET_     0x3021
#define MT9F002_GROUPED_PARAMETER_HOLD_     0x3022
#define MT9F002_MASK_CORRUPTED_FRAMES_     0x3023
#define MT9F002_PIXEL_ORDER_     0x3024
#define MT9F002_GPI_STATUS     0x3026
#define MT9F002_ANALOG_GAIN_CODE_GLOBAL_     0x3028
#define MT9F002_ANALOG_GAIN_CODE_GREENR_     0x302A
#define MT9F002_ANALOG_GAIN_CODE_RED_     0x302C
#define MT9F002_ANALOG_GAIN_CODE_BLUE_     0x302E
#define MT9F002_ANALOG_GAIN_CODE_GREENB_     0x3030
#define MT9F002_DIGITAL_GAIN_GREENR_     0x3032
#define MT9F002_DIGITAL_GAIN_RED_     0x3036
#define MT9F002_DIGITAL_GAIN_BLUE_     0x3038
#define MT9F002_DIGITAL_GAIN_GREENB_     0x3038
#define MT9F002_SMIA_VERSION_     0x303A
#define MT9F002_FRAME_COUNT_     0x303B
#define MT9F002_FRAME_STATUS     0x303C
#define MT9F002_READ_MODE     0x3040
#define MT9F002_FLASH     0x3046
#define MT9F002_FLASH_COUNT     0x3048
#define MT9F002_GREEN1_GAIN     0x3056
#define MT9F002_BLUE_GAIN     0x3058
#define MT9F002_RED_GAIN     0x305A
#define MT9F002_GREEN2_GAIN     0x305C
#define MT9F002_GLOBAL_GAIN     0x305E
#define MT9F002_DATAPATH_STATUS     0x306A
#define MT9F002_DATAPATH_SELECT     0x306E
#define MT9F002_TEST_PATTERN_MODE_     0x3070
#define MT9F002_TEST_DATA_RED_     0x3072
#define MT9F002_TEST_DATA_GREENR_     0x3074
#define MT9F002_TEST_DATA_BLUE_     0x3076
#define MT9F002_TEST_DATA_GREENB_     0x3078
#define MT9F002_TEST_RAW_MODE     0x307A
#define MT9F002_X_EVEN_INC_     0x30A0
#define MT9F002_X_ODD_INC_     0x30A2
#define MT9F002_Y_EVEN_INC_     0x30A4
#define MT9F002_Y_ODD_INC_     0x30A6
#define MT9F002_CALIB_GREEN1_ASC1     0x30A8
#define MT9F002_CALIB_BLUE_ASC1     0x30AA
#define MT9F002_CALIB_RED_ASC1     0x30AC
#define MT9F002_CALIB_GREEN2_ASC1     0x30AE
#define MT9F002_CALIB_GLOBAL     0x30BC
#define MT9F002_CALIB_CONTROL     0x30C0
#define MT9F002_CALIB_GREEN1     0x30C2
#define MT9F002_CALIB_BLUE     0x30C4
#define MT9F002_CALIB_RED     0x30C6
#define MT9F002_CALIB_GREEN2     0x30C8
#define MT9F002_CTX_CONTROL_REG     0x30E8
#define MT9F002_CTX_WR_DATA_REG     0x30EA
#define MT9F002_CTX_RD_DATA_REG     0x30EC
#define MT9F002_DARK_CONTROL3     0x30EE
#define MT9F002_OTPM_TCFG_READ_4B     0x3138
#define MT9F002_OTPM_CFG     0x3140
#define MT9F002_GLOBAL_FLASH_START     0x315A
#define MT9F002_GLOBAL_SEQ_TRIGGER     0x315E
#define MT9F002_GLOBAL_RST_END     0x3160
#define MT9F002_GLOBAL_SHUTTER_START1     0x3162
#define MT9F002_GLOBAL_SHUTTER_START2     0x3164
#define MT9F002_GLOBAL_READ_START     0x3166
#define MT9F002_GLOBAL_READ_START2     0x3168
#define MT9F002_DAC_RSTLO     0x316A
#define MT9F002_ANALOG_CONTROL5     0x3178
#define MT9F002_SERIAL_FORMAT_DESCRIPTOR_0     0x31A0
#define MT9F002_SERIAL_FORMAT_DESCRIPTOR_1     0x31A2
#define MT9F002_SERIAL_FORMAT_DESCRIPTOR_2     0x31A4
#define MT9F002_SERIAL_FORMAT_DESCRIPTOR_3     0x31A6
#define MT9F002_SERIAL_FORMAT_DESCRIPTOR_4     0x31A8
#define MT9F002_SERIAL_FORMAT_DESCRIPTOR_5     0x31AA
#define MT9F002_SERIAL_FORMAT_DESCRIPTOR_6     0x31AC
#define MT9F002_SERIAL_FORMAT     0x31AE
#define MT9F002_FRAME_PREAMBLE     0x31B0
#define MT9F002_LINE_PREAMBLE     0x31B2
#define MT9F002_MIPI_TIMING_0     0x31B4
#define MT9F002_MIPI_TIMING_1     0x31B6
#define MT9F002_MIPI_TIMING_2     0x31B8
#define MT9F002_MIPI_TIMING_3     0x31BA
#define MT9F002_MIPI_TIMING_4     0x31BC
#define MT9F002_HISPI_TIMING     0x31C0
#define MT9F002_HISPI_CONTROL_STATUS     0x31C6
#define MT9F002_HORIZONTAL_CURSOR_POSITION_     0x31E8
#define MT9F002_VERTICAL_CURSOR_POSITION_     0x31EA
#define MT9F002_HORIZONTAL_CURSOR_WIDTH_     0x31EC
#define MT9F002_VERTICAL_CURSOR_WIDTH_     0x31EE
#define MT9F002_I2C_IDS_MIPI_DEFAULT     0x31F2
#define MT9F002_I2C_IDS     0x31FC
#define MT9F002_P_GR_P0Q0     0x3600
#define MT9F002_P_GR_P0Q1     0x3602
#define MT9F002_P_GR_P0Q2     0x3604
#define MT9F002_P_GR_P0Q3     0x3606
#define MT9F002_P_GR_P0Q4     0x3608
#define MT9F002_P_RD_P0Q0     0x360A
#define MT9F002_P_RD_P0Q1     0x360C
#define MT9F002_P_RD_P0Q2     0x360E
#define MT9F002_P_RD_P0Q3     0x3610
#define MT9F002_P_RD_P0Q4     0x3612
#define MT9F002_P_BL_P0Q0     0x3614
#define MT9F002_P_BL_P0Q1     0x3616
#define MT9F002_P_BL_P0Q2     0x3618
#define MT9F002_P_BL_P0Q3     0x361A
#define MT9F002_P_BL_P0Q4     0x361C
#define MT9F002_P_GB_P0Q0     0x361E
#define MT9F002_P_GB_P0Q1     0x3620
#define MT9F002_P_GB_P0Q2     0x3622
#define MT9F002_P_GB_P0Q3     0x3624
#define MT9F002_P_GB_P0Q4     0x3626
#define MT9F002_P_GR_P1Q0     0x3640
#define MT9F002_P_GR_P1Q1     0x3642
#define MT9F002_P_GR_P1Q2     0x3644
#define MT9F002_P_GR_P1Q3     0x3646
#define MT9F002_P_GR_P1Q4     0x3648
#define MT9F002_P_RD_P1Q0     0x364A
#define MT9F002_P_RD_P1Q1     0x364C
#define MT9F002_P_RD_P1Q2     0x364E
#define MT9F002_P_RD_P1Q3     0x3650
#define MT9F002_P_RD_P1Q4     0x3652
#define MT9F002_P_BL_P1Q0     0x3654
#define MT9F002_P_BL_P1Q1     0x3656
#define MT9F002_P_BL_P1Q2     0x3658
#define MT9F002_P_BL_P1Q3     0x365A
#define MT9F002_P_BL_P1Q4     0x365C
#define MT9F002_P_GB_P1Q0     0x365E
#define MT9F002_P_GB_P1Q1     0x3660
#define MT9F002_P_GB_P1Q2     0x3662
#define MT9F002_P_GB_P1Q3     0x3664
#define MT9F002_P_GB_P1Q4     0x3666
#define MT9F002_P_GR_P2Q0     0x3680
#define MT9F002_P_GR_P2Q1     0x3682
#define MT9F002_P_GR_P2Q2     0x3684
#define MT9F002_P_GR_P2Q3     0x3686
#define MT9F002_P_GR_P2Q4     0x3688
#define MT9F002_P_RD_P2Q0     0x368A
#define MT9F002_P_RD_P2Q1     0x368C
#define MT9F002_P_RD_P2Q2     0x368E
#define MT9F002_P_RD_P2Q3     0x3690
#define MT9F002_P_RD_P2Q4     0x3692
#define MT9F002_P_BL_P2Q0     0x3694
#define MT9F002_P_BL_P2Q1     0x3696
#define MT9F002_P_BL_P2Q2     0x3698
#define MT9F002_P_BL_P2Q3     0x369A
#define MT9F002_P_BL_P2Q4     0x369C
#define MT9F002_P_GB_P2Q0     0x369E
#define MT9F002_P_GB_P2Q1     0x36A0
#define MT9F002_P_GB_P2Q2     0x36A2
#define MT9F002_P_GB_P2Q3     0x36A4
#define MT9F002_P_GB_P2Q4     0x36A6
#define MT9F002_P_GR_P3Q0     0x36C0
#define MT9F002_P_GR_P3Q1     0x36C2
#define MT9F002_P_GR_P3Q2     0x36C4
#define MT9F002_P_GR_P3Q3     0x36C6
#define MT9F002_P_GR_P3Q4     0x36C8
#define MT9F002_P_RD_P3Q0     0x36CA
#define MT9F002_P_RD_P3Q1     0x36CC
#define MT9F002_P_RD_P3Q2     0x36CE
#define MT9F002_P_RD_P3Q3     0x36D0
#define MT9F002_P_RD_P3Q4     0x36D2
#define MT9F002_P_BL_P3Q0     0x36D4
#define MT9F002_P_BL_P3Q1     0x36D6
#define MT9F002_P_BL_P3Q2     0x36D8
#define MT9F002_P_BL_P3Q3     0x36DA
#define MT9F002_P_BL_P3Q4     0x36DC
#define MT9F002_P_GB_P3Q0     0x36DE
#define MT9F002_P_GB_P3Q1     0x36E0
#define MT9F002_P_GB_P3Q2     0x36E2
#define MT9F002_P_GB_P3Q3     0x36E4
#define MT9F002_P_GB_P3Q4     0x36E6
#define MT9F002_P_GR_P4Q0     0x3700
#define MT9F002_P_GR_P4Q1     0x3702
#define MT9F002_P_GR_P4Q2     0x3704
#define MT9F002_P_GR_P4Q3     0x3706
#define MT9F002_P_GR_P4Q4     0x3708
#define MT9F002_P_RD_P4Q0     0x370A
#define MT9F002_P_RD_P4Q1     0x370C
#define MT9F002_P_RD_P4Q2     0x370E
#define MT9F002_P_RD_P4Q3     0x3710
#define MT9F002_P_RD_P4Q4     0x3712
#define MT9F002_P_BL_P4Q0     0x3714
#define MT9F002_P_BL_P4Q1     0x3716
#define MT9F002_P_BL_P4Q2     0x3718
#define MT9F002_P_BL_P4Q3     0x371A
#define MT9F002_P_BL_P4Q4     0x371C
#define MT9F002_P_GB_P4Q0     0x371E
#define MT9F002_P_GB_P4Q1     0x3720
#define MT9F002_P_GB_P4Q2     0x3722
#define MT9F002_P_GB_P4Q3     0x3724
#define MT9F002_P_GB_P4Q4     0x3726
#define MT9F002_POLY_SC_ENABLE     0x3780
#define MT9F002_POLY_ORIGIN_C     0x3782
#define MT9F002_POLY_ORIGIN_R     0x3784
#define MT9F002_P_GR_Q5     0x37C0
#define MT9F002_P_RD_Q5     0x37C2
#define MT9F002_P_BL_Q5     0x37C4
#define MT9F002_P_GB_Q5     0x37C6
#define MT9F002_DAC_ID_FBIAS     0x3EF8

#endif /* MT9F002_H */
