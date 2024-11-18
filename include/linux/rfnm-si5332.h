// SPDX-License-Identifier: GPL-2.0+
/* Si5332-GM1 Rev D Configuration Register Export Header File
 *
 * This file represents a series of Skyworks Si5332-GM1 Rev D
 * register writes that can be performed to load a single configuration
 * on a device. It was created by a Skyworks ClockBuilder Pro
 * export tool.
 *
 * Part:                                                      Si5332-GM1 Rev D
 * Design ID:
 * Includes Pre/Post Download Control Register Writes: Yes
 * Created By:                                         ClockBuilder Pro v4.12 [2024-01-19]
 * Timestamp:                                          2024-10-09 10:32:18 GMT-05:00
 *
 * A complete design report corresponding to this export is included at the end
 * of this header file.
 *
 */

#ifndef SI5332_GM1_REVD_REG_CONFIG_HEADER
#define SI5332_GM1_REVD_REG_CONFIG_HEADER

#define SI5332_GM1_REVD_REG_CONFIG_NUM_REGS                            71

typedef struct
{
       uint8_t address; /* 8-bit register address */
       uint8_t value; /* 8-bit register data */

} si5332_gm1_revd_register_t;

si5332_gm1_revd_register_t const si5332_gm1_revd_registers[SI5332_GM1_REVD_REG_CONFIG_NUM_REGS] =
{

       /* Start configuration preamble */
       /*    Set device in Ready mode */
       { 0x0006, 0x01 },
       /* End configuration preamble */

       /* Start configuration registers */
       { 0x0017, 0x00 },
       { 0x0018, 0x00 },
       { 0x0019, 0x00 },
       { 0x001A, 0x00 },
       { 0x001B, 0x00 },
       { 0x001C, 0x00 },
       { 0x0021, 0x6A },
       { 0x0024, 0x02 },
       { 0x0025, 0x00 },
       { 0x0026, 0x00 },
       { 0x0027, 0x00 },
       { 0x0028, 0x50 },
       { 0x0029, 0x50 },
       { 0x002A, 0x00 },
       { 0x002B, 0x14 },
       { 0x0036, 0x0C },
       { 0x0037, 0x49 },
       { 0x0038, 0x00 },
       { 0x0039, 0x5B },
       { 0x003A, 0x00 },
       { 0x003B, 0x7D },
       { 0x003C, 0x00 },
       { 0x0048, 0x00 },
       { 0x0054, 0x00 },
       { 0x0060, 0x00 },
       { 0x0067, 0x20 },
       { 0x0068, 0x00 },
       { 0x0069, 0x00 },
       { 0x006A, 0x00 },
       { 0x006B, 0x00 },
       { 0x006C, 0x01 },
       { 0x0073, 0x03 },
       { 0x0074, 0x00 },
       { 0x0075, 0x01 },
       { 0x007A, 0x07 },
       { 0x007B, 0x01 },
       { 0x007C, 0x00 },
       { 0x007D, 0x00 },
       { 0x007F, 0x07 },
       { 0x0080, 0x01 },
       { 0x0081, 0x00 },
       { 0x0082, 0x00 },
       { 0x0089, 0x07 },
       { 0x008A, 0x10 },
       { 0x008B, 0x00 },
       { 0x008C, 0x00 },
       { 0x0098, 0x09 },
       { 0x0099, 0x01 },
       { 0x009A, 0x00 },
       { 0x009B, 0x00 },
       { 0x00A7, 0x09 },
       { 0x00A8, 0x01 },
       { 0x00A9, 0x00 },
       { 0x00AA, 0x00 },
       { 0x00AC, 0x07 },
       { 0x00AD, 0x01 },
       { 0x00AE, 0x00 },
       { 0x00AF, 0x00 },
       { 0x00B6, 0x4B },
       { 0x00B7, 0x06 },
       { 0x00B9, 0x01 },
       { 0x00BA, 0x5E },
       { 0x00BB, 0x00 },
       { 0x00BC, 0x00 },
       { 0x00BD, 0x00 },
       { 0x00BE, 0x20 },
       { 0x00BF, 0x00 },
       { 0x00C0, 0x00 },
       { 0x00C1, 0x00 },
       /* End configuration registers */

       /* Start configuration postamble */
       /*    Set device in Active mode */
       { 0x0006, 0x02 },
       /* End configuration postamble */

};

/*
 * Design Report
 *
 * Overview
 * ========
 *
 * Part:               Si5332AB-GM1 Rev D
 * Project File:       C:\Users\Davide\Desktop\Skyworks\Seeve-Si5332.slabtimeproj
 * Design ID:          <none>
 * Created By:         ClockBuilder Pro v4.12 [2024-01-19]
 * Timestamp:          2024-10-09 10:32:18 GMT-05:00
 *
 * Design Rule Check
 * =================
 *
 * Errors:
 * - No errors
 *
 * Warnings:
 * - No warnings
 *
 * Device Grade
 * ============
 * Maximum Output Frequency: 122.88 MHz
 * Frequency Synthesis Mode: Fractional
 * Frequency Plan Grade:     B
 * Minimum Base OPN:         Si5332B*
 *
 * Base       Output Clock         Supported Frequency Synthesis Modes
 * OPN Grade  Frequency Range      (Typical Jitter)
 * ---------  ------------------  --------------------------------------------
 * Si5332A    5 MHz to 334 MHz    Integer (~230 fs) and fractional (~500 fs)
 * Si5332B*   5 MHz to 200 MHz    "
 * Si5332C    5 MHz to 334 MHz    Integer only (~230 fs)
 * Si5332D    5 MHz to 200 MHz    "
 *
 * * Based on your calculated frequency plan, a Si5332B grade device is
 * sufficient for your design. For more in-system configuration flexibility
 * (higher frequencies and/or to enable fractional synthesis), consider
 * selecting device grade Si5332A when specifying an ordering part number (OPN)
 * for your application. See the datasheet Ordering Guide for more information.
 *
 * Design
 * ======
 * Base I2C Address: 0x6A
 *
 * Universal Hardware Input Pins:
 *    INPUT1 (P8) : None
 *    INPUT2 (P17): None
 *    INPUT3 (P24): None
 *    INPUT4 (P28): None
 *    INPUT5 (P29): None
 *
 * Inputs:
 *    XAXB: Unused
 *  CLKIN2: 38.4 MHz LVCMOS
 *
 * Outputs:
 *    OUT0: 122.88 MHz LVDS Fast 1.8 V, Disabled-State: Stop Low
 *          Power-up state: Enabled
 *    OUT1: 122.88 MHz LVDS Fast 1.8 V, Disabled-State: Stop Low
 *          Power-up state: Enabled
 *    OUT2: 7.68 MHz LVDS Fast 1.8 V, Disabled-State: Stop Low
 *          Power-up state: Enabled
 *    OUT3: 100 MHz HCSL 1.8 V 50 Ohms - Internal, Disabled-State: Stop Low
 *          Power-up state: Enabled
 *    OUT4: 100 MHz HCSL 1.8 V 50 Ohms - Internal, Disabled-State: Stop Low
 *          Power-up state: Enabled
 *    OUT5: 122.88 MHz LVDS Fast 1.8 V, Disabled-State: Stop Low
 *          Power-up state: Enabled
 *
 * Frequency Plan
 * ==============
 *
 * Fpfd = 38.4 MHz
 * Fvco = 2.4576 GHz
 *
 * P divider = 1
 * M = 64
 * N dividers:
 *    N0:
 *       Value: 24.576
 *       OUT3: 100 MHz, Error: 0 ppm
 *       OUT4: 100 MHz, Error: 0 ppm
 *    N1:
 *       Unused
 *
 * O dividers:
 *    O0:
 *       Value: 20
 *       OUT0: 122.88 MHz, Error: 0 ppm
 *       OUT1: 122.88 MHz, Error: 0 ppm
 *       OUT2: 7.68 MHz, Error: 0 ppm
 *       OUT5: 122.88 MHz, Error: 0 ppm
 *    O1:
 *       Unused
 *    O2:
 *       Unused
 *    O3:
 *       Unused
 *    O4:
 *       Unused
 *
 * R dividers:
 *    R0 = 1
 *    R1 = 1
 *    R2 = 16
 *    R3 = 1
 *    R4 = 1
 *    R5 = 1
 *
 * Estimated Power
 * ===============
 * Assumptions:
 *
 * VDD:      1.8 V
 * Ta:       25 �C
 * Theta-JA: 23.00 �C/W (JEDEC Board with 2 m/s airflow)
 *
 *                               Overall  On Chip
 * Condition                     Power    Power    Ta    Tj
 * ----------------------------  -------  -------  ----  ----
 * Typical Ta, Voltage, Current  216 mW   210 mW   25 C  29 C
 *
 *                                         -----------------------
 *                                                 Typical
 *                                         -----------------------
 *                                         Voltage  Current  Power
 *           Output  Frequency   Format      (V)     (mA)    (mW)
 *           ------  ----------  --------  -------  -------  -----
 * VDDA                                       1.80       28     50
 * VDD Dig                                    1.80        6     11
 * VDD Xtal                                   1.80        5      8
 * VDDO0     OUT0    122.88 MHz  LVDSFast     1.80       11     20
 * VDDO1     OUT1    122.88 MHz  LVDSFast     1.80       11     20
 * VDDO2     OUT2      7.68 MHz  LVDSFast     1.80       10     18
 * VDDO3     OUT3       100 MHz      HCSL     1.80       20     35
 * VDDO4     OUT4       100 MHz      HCSL     1.80       20     35
 * VDDO5     OUT5    122.88 MHz  LVDSFast     1.80       11     20
 *                                         -------  -------  -----
 *                                            1.80      120    216
 *                                         -------  -------  -----
 *                                           Total      120    216
 *                                         -------  -------  -----
 *
 * Note:
 *
 * - Tj is junction temperature. Tj must be less than 125 �C (on Si5332-GM1
 *   Revision D) for device to comply with datasheet specifications. Tj = Ta +
 *   Theta_JA*On_Chip_Power.
 * - Overall power includes on-chip power dissipation and adds differential load
 *   power dissipation to estimate total power requirements.
 * - Above are estimates only: power and temperature should be measured on your
 *   PCB.
 *
 * Settings
 * ========
 *
 * Location    Setting Name     Decimal Value      Hex Value
 * ----------  ---------------  -----------------  -----------------
 * 0x17[7:0]   DESIGN_ID0       0                  0x00
 * 0x18[7:0]   DESIGN_ID1       0                  0x00
 * 0x19[7:0]   DESIGN_ID2       0                  0x00
 * 0x1A[7:0]   DESIGN_ID3       0                  0x00
 * 0x1B[7:0]   DESIGN_ID4       0                  0x00
 * 0x1C[7:0]   DESIGN_ID5       0                  0x00
 * 0x21[6:0]   I2C_ADDR         106                0x6A
 * 0x24[1:0]   IMUX_SEL         2                  0x2
 * 0x25[1:0]   OMUX0_SEL0       0                  0x0
 * 0x25[6:4]   OMUX0_SEL1       0                  0x0
 * 0x26[1:0]   OMUX1_SEL0       0                  0x0
 * 0x26[6:4]   OMUX1_SEL1       0                  0x0
 * 0x27[1:0]   OMUX2_SEL0       0                  0x0
 * 0x27[6:4]   OMUX2_SEL1       0                  0x0
 * 0x28[1:0]   OMUX3_SEL0       0                  0x0
 * 0x28[6:4]   OMUX3_SEL1       5                  0x5
 * 0x29[1:0]   OMUX4_SEL0       0                  0x0
 * 0x29[6:4]   OMUX4_SEL1       5                  0x5
 * 0x2A[1:0]   OMUX5_SEL0       0                  0x0
 * 0x2A[6:4]   OMUX5_SEL1       0                  0x0
 * 0x2B[7:0]   HSDIV0A_DIV      20                 0x14
 * 0x36[14:0]  ID0A_INTG        3145               0x0C49
 * 0x38[14:0]  ID0A_RES         91                 0x005B
 * 0x3A[14:0]  ID0A_DEN         125                0x007D
 * 0x3C[0]     ID0A_SS_ENA      0                  0x0
 * 0x3C[2:1]   ID0A_SS_MODE     0                  0x0
 * 0x48[0]     ID0B_SS_ENA      0                  0x0
 * 0x48[2:1]   ID0B_SS_MODE     0                  0x0
 * 0x54[0]     ID1A_SS_ENA      0                  0x0
 * 0x54[2:1]   ID1A_SS_MODE     0                  0x0
 * 0x60[0]     ID1B_SS_ENA      0                  0x0
 * 0x60[2:1]   ID1B_SS_MODE     0                  0x0
 * 0x67[14:0]  IDPA_INTG        8192               0x2000
 * 0x69[14:0]  IDPA_RES         0                  0x0000
 * 0x6B[14:0]  IDPA_DEN         1                  0x0001
 * 0x73[1:0]   CLKIN_2_CLK_SEL  3                  0x3
 * 0x74[1:0]   CLKIN_3_CLK_SEL  0                  0x0
 * 0x75[4:0]   P_VAL            1                  0x01
 * 0x7A[3:0]   OUT0_MODE        7                  0x7
 * 0x7B[5:0]   OUT0_DIV         1                  0x01
 * 0x7C[2:0]   OUT0_SKEW        0                  0x0
 * 0x7D[0]     OUT0_STOP_HIGHZ  0                  0x0
 * 0x7D[5:4]   OUT0_CMOS_INV    0                  0x0
 * 0x7D[6]     OUT0_DIFF_INV    0                  0x0
 * 0x7F[3:0]   OUT1_MODE        7                  0x7
 * 0x80[5:0]   OUT1_DIV         1                  0x01
 * 0x81[2:0]   OUT1_SKEW        0                  0x0
 * 0x82[0]     OUT1_STOP_HIGHZ  0                  0x0
 * 0x82[5:4]   OUT1_CMOS_INV    0                  0x0
 * 0x82[6]     OUT1_DIFF_INV    0                  0x0
 * 0x89[3:0]   OUT2_MODE        7                  0x7
 * 0x8A[5:0]   OUT2_DIV         16                 0x10
 * 0x8B[2:0]   OUT2_SKEW        0                  0x0
 * 0x8C[0]     OUT2_STOP_HIGHZ  0                  0x0
 * 0x8C[5:4]   OUT2_CMOS_INV    0                  0x0
 * 0x8C[6]     OUT2_DIFF_INV    0                  0x0
 * 0x98[3:0]   OUT3_MODE        9                  0x9
 * 0x99[5:0]   OUT3_DIV         1                  0x01
 * 0x9A[2:0]   OUT3_SKEW        0                  0x0
 * 0x9B[0]     OUT3_STOP_HIGHZ  0                  0x0
 * 0x9B[5:4]   OUT3_CMOS_INV    0                  0x0
 * 0x9B[6]     OUT3_DIFF_INV    0                  0x0
 * 0xA7[3:0]   OUT4_MODE        9                  0x9
 * 0xA8[5:0]   OUT4_DIV         1                  0x01
 * 0xA9[2:0]   OUT4_SKEW        0                  0x0
 * 0xAA[0]     OUT4_STOP_HIGHZ  0                  0x0
 * 0xAA[5:4]   OUT4_CMOS_INV    0                  0x0
 * 0xAA[6]     OUT4_DIFF_INV    0                  0x0
 * 0xAC[3:0]   OUT5_MODE        7                  0x7
 * 0xAD[5:0]   OUT5_DIV         1                  0x01
 * 0xAE[2:0]   OUT5_SKEW        0                  0x0
 * 0xAF[0]     OUT5_STOP_HIGHZ  0                  0x0
 * 0xAF[5:4]   OUT5_CMOS_INV    0                  0x0
 * 0xAF[6]     OUT5_DIFF_INV    0                  0x0
 * 0xB6[0]     OUT0_OE          1                  0x1
 * 0xB6[1]     OUT1_OE          1                  0x1
 * 0xB6[3]     OUT2_OE          1                  0x1
 * 0xB6[6]     OUT3_OE          1                  0x1
 * 0xB7[1]     OUT4_OE          1                  0x1
 * 0xB7[2]     OUT5_OE          1                  0x1
 * 0xB9[0]     XOSC_DIS         1                  0x1
 * 0xB9[1]     IBUF0_DIS        0                  0x0
 * 0xB9[3]     IMUX_DIS         0                  0x0
 * 0xB9[4]     PDIV_DIS         0                  0x0
 * 0xB9[5]     PLL_DIS          0                  0x0
 * 0xBA[5]     ID0_DIS          0                  0x0
 * 0xBA[6]     ID1_DIS          1                  0x1
 * 0xBA[0]     HSDIV0_DIS       0                  0x0
 * 0xBA[1]     HSDIV1_DIS       1                  0x1
 * 0xBA[2]     HSDIV2_DIS       1                  0x1
 * 0xBA[3]     HSDIV3_DIS       1                  0x1
 * 0xBA[4]     HSDIV4_DIS       1                  0x1
 * 0xBB[0]     OMUX0_DIS        0                  0x0
 * 0xBB[1]     OMUX1_DIS        0                  0x0
 * 0xBB[2]     OMUX2_DIS        0                  0x0
 * 0xBB[3]     OMUX3_DIS        0                  0x0
 * 0xBB[4]     OMUX4_DIS        0                  0x0
 * 0xBB[5]     OMUX5_DIS        0                  0x0
 * 0xBC[0]     OUT0_DIS         0                  0x0
 * 0xBC[1]     OUT1_DIS         0                  0x0
 * 0xBC[3]     OUT2_DIS         0                  0x0
 * 0xBC[6]     OUT3_DIS         0                  0x0
 * 0xBD[1]     OUT4_DIS         0                  0x0
 * 0xBD[2]     OUT5_DIS         0                  0x0
 * 0xBE[7:0]   PLL_MODE         32                 0x20
 * 0xBF[0]     XOSC_CINT_ENA    0                  0x0
 * 0xC0[5:0]   XOSC_CTRIM_XIN   0                  0x00
 * 0xC1[5:0]   XOSC_CTRIM_XOUT  0                  0x00
 *
 *
 */

#endif
