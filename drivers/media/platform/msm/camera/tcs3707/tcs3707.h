/*
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for TCS3707 als/ps sensor chip.
 */
#ifndef __TCS3707_H__
#define __TCS3707_H__

//#define TCS3707_I2C 

#include <linux/ioctl.h>

/* Register map */
#define TCS3707_ENABLE_REG              0x80
#define TCS3707_ATIME_REG               0x81
#define TCS3707_PTIME_REG               0x82
#define TCS3707_WTIME_REG               0x83
#define TCS3707_AILTL_REG               0x84
#define TCS3707_AILTH_REG               0x85
#define TCS3707_AIHTL_REG               0x86
#define TCS3707_AIHTH_REG               0x87
#define TCS3707_PILT0L_REG              0x88
#define TCS3707_PILT0H_REG              0x89
#define TCS3707_PILT1L_REG              0x8A
#define TCS3707_PILT1H_REG              0x8B
#define TCS3707_PIHT0L_REG              0x8C
#define TCS3707_PIHT0H_REG              0x8D
#define TCS3707_PIHT1L_REG              0x8E
#define TCS3707_PIHT1H_REG              0x8F
#define TCS3707_AUXID_REG               0x90
#define TCS3707_REVID_REG               0x91
#define TCS3707_ID_REG                  0x92
#define TCS3707_STATUS_REG              0x93
#define TCS3707_ASTATUS_REG             0x94
#define TCS3707_ADATA0L_REG             0x95
#define TCS3707_ADATA0H_REG             0x96
#define TCS3707_ADATA1L_REG             0x97
#define TCS3707_ADATA1H_REG             0x98
#define TCS3707_ADATA2L_REG             0x99
#define TCS3707_ADATA2H_REG             0x9A
#define TCS3707_ADATA3L_REG             0x9B
#define TCS3707_ADATA3H_REG             0x9C
#define TCS3707_ADATA4L_REG             0x9D
#define TCS3707_ADATA4H_REG             0x9E
#define TCS3707_ADATA5L_REG             0x9F
#define TCS3707_ADATA5H_REG             0xA0
#define TCS3707_PDATAL_REG              0xA1
#define TCS3707_PDATAH_REG              0xA2
#define TCS3707_STATUS2_REG             0xA3
#define TCS3707_STATUS3_REG             0xA4
#define TCS3707_STATUS5_REG             0xA6
#define TCS3707_STATUS6_REG             0xA7
#define TCS3707_CFG0_REG                0xA9
#define TCS3707_CFG1_REG                0xAA
#define TCS3707_CFG3_REG                0xAC
#define TCS3707_CFG4_REG                0xAD

#define TCS3707_CFG8_REG                0xB1

#define TCS3707_CFG10_REG               0xB3
#define TCS3707_CFG11_REG               0xB4
#define TCS3707_CFG12_REG               0xB5
#define TCS3707_CFG14_REG               0xB7
#define TCS3707_PCFG1_REG               0xB8
#define TCS3707_PCFG2_REG               0xB9
#define TCS3707_PCFG4_REG               0xBB
#define TCS3707_PCFG5_REG               0xBC
#define TCS3707_PERS_REG                0xBD
#define TCS3707_GPIO_REG                0xBE
#define TCS3707_POFFSETL_REG            0xC7
#define TCS3707_POFFSETH_REG            0xC8
#define TCS3707_ASTEPL_REG              0xCA
#define TCS3707_ASTEPH_REG              0xCB
#define TCS3707_AGC_GAIN_MAX_REG        0xCF
#define TCS3707_PXAVGL_REG              0xD0
#define TCS3707_PXAVGH_REG              0xD1
#define TCS3707_PBSLNL_REG              0xD2
#define TCS3707_PBSLNH_REG              0xD3
#define TCS3707_AZ_CONFIG_REG           0xD6

#define TCS3707_FD_CFG0                 0xD7
#define TCS3707_FD_CFG1                 0xD8
#define TCS3707_FD_CFG3                 0xDA

#define TCS3707_CALIB_REG               0xEA
#define TCS3707_CALIBCFG0_REG           0xEB
#define TCS3707_CALIBCFG1_REG           0xEC
#define TCS3707_CALIBCFG2_REG           0xED
#define TCS3707_CALIBSTAT_REG           0xEE
#define TCS3707_INTENAB_REG             0xF9

#define TCS3707_CONTROL_REG             0xFA
#define TCS3707_FIFO_MAP                0xFC
#define TCS3707_FIFO_STATUS             0xFD
#define TCS3707_FDATAL                  0xFE
#define TCS3707_FDATAH                  0xFF

/* Register bits map */
//ENABLE @ 0x80
#define PON                             (0x01 << 0)
#define AEN                             (0x01 << 1)
#define PEN                             (0x01 << 2)
#define WEN                             (0x01 << 3)
#define FDEN                            (0x01 << 6)
#define POFF                             0x00
#define FD_MASK                         (0x01 << 6)


//AUXID @ 0x90
#define AUXID_MASK                      (0x0F << 0)

//REVID @ 0x91
#define REVID_MASK                      (0x07 << 0)

//ID_MASK @ 0x92
#define ID_MASK                         (0x3F << 2)

//STATUS @ 0x93
#define SINT                            (0x01 << 0)
#define CINT                            (0x01 << 1)
#define AINT                            (0x01 << 3)
#define PINT0                           (0x01 << 4)
#define PINT1                           (0x01 << 5)
#define PSAT                            (0x01 << 6)
#define ASAT                            (0x01 << 7)

//ASTATUS @0x94
#define AGAIN_STATUS_MASK               (0x0F << 0)
#define ASAT_STATUS                     (0x01 << 7)

//STATUS2 @0xA3
#define ASAT_ANALOG                     (0x01 << 3)
#define ASAT_DIGITAL                    (0x01 << 4)
#define PVALID                          (0x01 << 5)
#define AVALID                          (0x01 << 6)

//STATUS3 @0xA4
#define PSAT_AMBIENT                    (0x01 << 0)
#define PSAT_REFLECTIVE                 (0x01 << 1)
#define PSAT_ADC                        (0x01 << 2)
#define STATUS3_RVED                    (0x01 << 3)
#define AINT_AILT                       (0x01 << 4)
#define AINT_AIHT                       (0x01 << 5)

//STATUS4 @0xA5
#define PINT0_PILT                      (0x01 << 0)
#define PINT0_PIHT                      (0x01 << 1)
#define PINT1_PILT                      (0x01 << 2)
#define PINT1_PIHT                      (0x01 << 3)

//STATUS6 @0xA7
#define INIT_BUSY                       (0x01 << 0)
#define SAI_ACTIVE                      (0x01 << 1)
#define ALS_TRIGGER_ERROR               (0x01 << 2)
#define PROX_TRIGGER_ERROR              (0x01 << 3)
#define OVTEMP_DETECTED                 (0x01 << 5)

//CFG0 @0xA9
#define ALS_TRIGGER_LONG                (0x01 << 2)
#define PROX_TRIGGER_LONG               (0x01 << 3)
#define LOWPOWER_IDLE                   (0x01 << 5)

//CFG1 @0xAA
#define AGAIN_0_5X                      (0x00 << 0)
#define AGAIN_1X                        (0x01 << 0)
#define AGAIN_2X                        (0x02 << 0)
#define AGAIN_4X                        (0x03 << 0)
#define AGAIN_8X                        (0x04 << 0)
#define AGAIN_16X                       (0x05 << 0)
#define AGAIN_32X                       (0x06 << 0)
#define AGAIN_64X                       (0x07 << 0)
#define AGAIN_128X                      (0x08 << 0)
#define AGAIN_256X                      (0x09 << 0)
#define AGAIN_512X                      (0x0A << 0)
#define AGAIN_MASK                      (0x1F << 0)

//CFG3 @0xAC
#define CFG3_RVED                       (0x0C << 0)
#define SAI                             (0x01 << 4)
#define HXTALK_MODE1                    (0x01 << 5)

//CFG4 @0xAD
#define GPIO_PINMAP_DEFAULT             (0x00 << 0)
#define GPIO_PINMAP_RVED                (0x01 << 0)
#define GPIO_PINMAP_AINT                (0x02 << 0)
#define GPIO_PINMAP_PINT0               (0x03 << 0)
#define GPIO_PINMAP_PINT1               (0x04 << 0)
#define GPIO_PINMAP_MASK                (0x07 << 0)
#define INT_INVERT                      (0x01 << 3)
#define INT_PINMAP_NORMAL               (0x00 << 4)
#define INT_PINMAP_RVED                 (0x01 << 4)
#define INT_PINMAP_AINT                 (0x02 << 4)
#define INT_PINMAP_PINT0                (0x03 << 4)
#define INT_PINMAP_PINT1                (0x04 << 4)
#define INT_PINMAP_MASK                 (0x07 << 4)

//CFG8_REG @0xB1
#define SWAP_PROX_ALS5                  (0x01 << 0)
#define ALS_AGC_ENABLE                  (0x01 << 2)
#define CONCURRENT_PROX_AND_ALS         (0x01 << 4)

//CFG10_REG @0xB3
#define ALS_AGC_LOW_HYST_12_5           (0x00 << 4)
#define ALS_AGC_LOW_HYST_25             (0x01 << 4)
#define ALS_AGC_LOW_HYST_37_5           (0x02 << 4)
#define ALS_AGC_LOW_HYST_50             (0x03 << 4)
#define ALS_AGC_LOW_HYST_MASK           (0x03 << 4)
#define ALS_AGC_HIGH_HYST_50            (0x00 << 6)
#define ALS_AGC_HIGH_HYST_62_5          (0x01 << 6)
#define ALS_AGC_HIGH_HYST_75            (0x02 << 6)
#define ALS_AGC_HIGH_HYST_87_5          (0x03 << 6)
#define ALS_AGC_HIGH_HYST_MASK          (0x03 << 6)

//CFG11_REG @0xB4
#define PINT_DIRECT                     (0x01 << 6)
#define AINT_DIRECT                     (0x01 << 7)

//CFG12_REG @0xB5
#define ALS_TH_CHANNEL_0                (0x00 << 0)
#define ALS_TH_CHANNEL_1                (0x01 << 0)
#define ALS_TH_CHANNEL_2                (0x02 << 0)
#define ALS_TH_CHANNEL_3                (0x03 << 0)
#define ALS_TH_CHANNEL_4                (0x04 << 0)
#define ALS_TH_CHANNEL_MASK             (0x07 << 0)

//CFG14_REG @0xB7
#define PROX_OFFSET_COARSE_MASK         (0x1F << 0)
#define EN_PROX_OFFSET_RANGE            (0x01 << 5)
#define AUTO_CO_CAL_EN                  (0x01 << 6)

//PCFG1_REG @0xB8
#define PROX_FILTER_1                   (0x00 << 0)
#define PROX_FILTER_2                   (0x01 << 0)
#define PROX_FILTER_4                   (0x02 << 0)
#define PROX_FILTER_8                   (0x03 << 0)
#define PROX_FILTER_MASK                (0x03 << 0)
#define PROX_FILTER_DOWNSAMPLE          (0x01 << 2)
#define HXTALK_MODE2                    (0x01 << 7)

//PCFG2_REG @0xB9
#define PLDRIVE0_SHIFT                  0
#define PLDRIVE0_MASK                   (0x7F << PLDRIVE0_SHIFT)//2xPLDRIVE0 + 4mA


//PCFG4_REG @0xBB
#define PGAIN_1X                        (0x00 << 0)
#define PGAIN_2X                        (0x01 << 0)
#define PGAIN_4X                        (0x02 << 0)
#define PGAIN_8X                        (0x03 << 0)
#define PGAIN_MASK                      (0x03 << 0)

//PCFG5_REG @0xBC
#define PPULSE_SHIFT                    0
#define PPULSE_MASK                     (0x3F << PPULSE_SHIFT)
#define PPULSE_LEN_SHIFT                6
#define PPULSE_LEN_MASK                 (0x03 << PPULSE_LEN_SHIFT)
#define PPULSE_LEN_4US                  (0x00 << PPULSE_LEN_SHIFT)
#define PPULSE_LEN_8US                  (0x01 << PPULSE_LEN_SHIFT)
#define PPULSE_LEN_16US                 (0x02 << PPULSE_LEN_SHIFT)
#define PPULSE_LEN_32US                 (0x03 << PPULSE_LEN_SHIFT)


//PERS_REG @0xBD
#define APERS_SHIFT                     0
#define APERS_MASK                      (0x0F << APERS_SHIFT)
#define PPERS_SHIFT                     4
#define PPERS_MASK                      (0x0F << PPERS_SHIFT)


//GPIO_REG @0xBE
#define GPIO_IN                         (0x01 << 0)
#define GPIO_OUT                        (0x01 << 1)
#define GPIO_IN_EN                      (0x01 << 2)
#define GPIO_INVERT                     (0x01 << 3)

//GAIN_MAX_REG @0xCF
#define AGC_AGAIN_MAX_MASK              (0x0F << 0) //2^(AGC_AGAIN_MAX)

//CALIB_REG @0xEA
#define START_OFFSET_CALIB              (0x01 << 0)

//CALIBCFG0_REG @0xEB
#define DCAVG_ITERATIONS_MASK           (0x07 << 0)//0 is skip, 2^(ITERATIONS)
#define BINSRCH_SKIP                    (0x01 << 3)
#define DCAVG_AUTO_OFFSET_ADJUST        (0x01 << 6)
#define DCAVG_AUTO_BSLN                 (0x01 << 7)

//CALIBCFG1_REG @0xEC
#define PXAVG_ITERATIONS_MASK           (0x07 << 0)//0 is skip, 2^(ITERATIONS)
#define PXAVG_AUTO_BSLN                 (0x01 << 3)
#define PROX_AUTO_OFFSET_ADJUST         (0x01 << 6)

//CALIBCFG1_REG @0xED
#define BINSRCH_TARGET_3                (0x00 << 5)
#define BINSRCH_TARGET_7                (0x01 << 5)
#define BINSRCH_TARGET_15               (0x02 << 5)
#define BINSRCH_TARGET_31               (0x03 << 5)
#define BINSRCH_TARGET_63               (0x04 << 5)
#define BINSRCH_TARGET_127              (0x05 << 5)
#define BINSRCH_TARGET_255              (0x06 << 5)
#define BINSRCH_TARGET_511              (0x07 << 5)
#define BINSRCH_TARGET_MASK             (0x07 << 5)

//CALIBSTAT_REG @0xEE
#define CALIB_FINISHED                  (0x01 << 0)
#define OFFSET_ADJUSTED                 (0x01 << 1)
#define BASELINE_ADJUSTED               (0x01 << 2)

//INTENAB_REG @0xF9
#define SIEN                            (0x01 << 0)
#define CIEN                            (0x01 << 1)
#define AIEN                            (0x01 << 3)
#define PIEN0                           (0x01 << 4)
#define PIEN1                           (0x01 << 5)
#define PSIEN                           (0x01 << 6)
#define ASIEN                           (0x01 << 7)

//CONTROL_REG @0xFA
#define CLEAR_SAI_ACTIVE                (0x01 << 0)
#define ALS_MANUAL_AZ                   (0x01 << 2)


//TCS3707_FD_CFG3 @0xDA
#define FD_GAIN_0_5X                      (0x00 << 3)
#define FD_GAIN_1X                        (0x01 << 3)
#define FD_GAIN_2X                        (0x02 << 3)
#define FD_GAIN_4X                        (0x03 << 3)
#define FD_GAIN_8X                        (0x04 << 3)
#define FD_GAIN_16X                       (0x05 << 3)
#define FD_GAIN_32X                       (0x06 << 3)
#define FD_GAIN_64X                       (0x07 << 3)
#define FD_GAIN_128X                      (0x08 << 3)
#define FD_GAIN_256X                      (0x09 << 3)
#define FD_GAIN_512X                      (0x0A << 3)
#define FD_GAIN_MASK                      (0x1F << 3)
#define FD_TIME_MASK                      (0x07)

//TCS3707_CONTROL_REG @ 0xFA
#define FIFO_CLEAR                  0x02

#define tcs3707_DEV_NAME     "tcs3707"
#define TCS_TAG                  "<TCS3707> "
#define TCS_FUN(f)               pr_debug(TCS_TAG"%s\n", __func__)
#define TCS_ERR(fmt, args...)    pr_err(TCS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define TCS_LOG(fmt, args...)    pr_debug(TCS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define TCS_DBG(fmt, args...)    pr_debug(TCS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)


//Configration calculations
#define ASTEP_US_PER_100                278
#define ASTEP_US(us)                    (uint16_t)(((uint32_t)us*100 + (ASTEP_US_PER_100 >> 1)) / ASTEP_US_PER_100 - 1)
#define PTIME_MS_PER_100                278
#define PTIME_MS(ms)                    (uint8_t)(((uint32_t)ms*100 + (PTIME_MS_PER_100 >> 1)) / PTIME_MS_PER_100 - 1)
#define WTIME_MS_PER_100                278
#define WTIME_MS(ms)                    (uint8_t)(((uint32_t)ms*100 + (WTIME_MS_PER_100 >> 1)) / WTIME_MS_PER_100 - 1)
#define PLDRIVE_MA(ma)                  (uint8_t)(((ma-4) >> 1) << PLDRIVE0_SHIFT)
#define PPULSES(c)                      (uint8_t)((c - 1) << PPULSE_SHIFT)
#define ALS_PERSIST(p)                  (uint8_t)(((p) & 0x0F) << APERS_SHIFT)
#define PROX_PERSIST(p)                 (uint8_t)(((p) & 0x0F) << PPERS_SHIFT)
#define ATIME_MS_PER_100                278
#define ATIME_MS(ms)                    (uint16_t)(((uint32_t)ms*100 + (ASTEP_US_PER_100 >> 1)) / ASTEP_US_PER_100 - 1)



#endif
