/*
 * d2083 register declarations.
 *
 * Copyright(c) 2012 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

/*
 *  [ History ]
 *  1. 05 Apr 2012. Created and updated RegMap using D2083AA_TS_v0.3.pdf
 *
 */

#ifndef __LINUX_D2083_REG_H
#define __LINUX_D2083_REG_H

/* Status / Config */
#define D2083_PAGECON0_REG          0x00    /* 0 */
#define D2083_STATUSA_REG           0x01    /* 1 */
#define D2083_STATUSB_REG           0x02    /* 2 */
#define D2083_STATUSC_REG           0x03    /* 3 */
#define D2083_STATUSD_REG           0x04    /* 4 */
#define D2083_EVENTA_REG            0x05    /* 5 */
#define D2083_EVENTB_REG            0x06    /* 6 */
#define D2083_EVENTC_REG            0x07    /* 7 */
#define D2083_EVENTD_REG            0x08    /* 8 */
#define D2083_FAULTLOG_REG          0x09    /* 9 */
#define D2083_IRQMASKA_REG          0x0A    /* 10 */
#define D2083_IRQMASKB_REG          0x0B    /* 11 */
#define D2083_IRQMASKC_REG          0x0C    /* 12 */
#define D2083_IRQMASKD_REG          0x0D    /* 13 */
#define D2083_CONTROLA_REG          0x0E    /* 14 */
#define D2083_CONTROLB_REG          0x0F    /* 15 */
#define D2083_CONTROLC_REG          0x10    /* 16 */
#define D2083_CONTROLD_REG          0x11    /* 17 */
#define D2083_PDDIS_REG             0x12    /* 18 */
#define D2083_INTERFACE_REG         0x13    /* 19 */
#define D2083_RESET_REG             0x14    /* 20 */

/* GPIO */
#define D2083_RESERVED_0x15         0x15    /* 21 */
#define D2083_GPIO_TA_REG           0x16    /* 22 */
#define D2083_GPIO_NJIGON_REG       0x17    /* 23 */
#define D2083_RESERVED_0x18         0x18    /* 24 */
#define D2083_RESERVED_0x19         0x19    /* 25 */
#define D2083_RESERVED_0x1a         0x1A    /* 26 */
#define D2083_GPIO_REG              0x1B    /* 27 */
#define D2083_RESERVED_0x1c         0x1C    /* 28 */

/* Sequencer*/
#define D2083_ID01_REG              0x1D    /* 29 */
#define D2083_ID23_REG              0x1E    /* 30 */
#define D2083_ID45_REG              0x1F    /* 31 */
#define D2083_ID67_REG              0x20    /* 32 */
#define D2083_ID89_REG              0x21    /* 33 */
#define D2083_ID1011_REG            0x22    /* 34 */
#define D2083_ID1213_REG            0x23    /* 35 */
#define D2083_ID1415_REG            0x24    /* 36 */
#define D2083_ID1617_REG            0x25    /* 37 */
#define D2083_RESERVED_0x26         0x26    /* 38 */
#define D2083_RESERVED_0x27         0x27    /* 39 */
#define D2083_SEQSTATUS_REG         0x28    /* 40 */
#define D2083_SEQA_REG              0x29    /* 41 */
#define D2083_SEQB_REG              0x2A    /* 42 */
#define D2083_SEQTIMER_REG          0x2B    /* 43 */

/* Supplies */

/* Regulator Register set 1 */
#define D2083_BUCKA_REG             0x2C    /* 44 */
#define D2083_BUCKB_REG             0x2D    /* 45 */
    /* Sets of BUCK regulator */
#define D2083_BUCK1_REG             0x2E    /* 46 */
#define D2083_BUCK2_REG             0x2F    /* 47 */
#define D2083_BUCK3_REG             0x30    /* 48 */
#define D2083_BUCK4_REG             0x31    /* 49 */

    /* Sets of LDO regulator */
#define D2083_LDO1_REG              0x32    /* 50 */
#define D2083_LDO2_REG              0x33    /* 51 */
#define D2083_LDO3_REG              0x34    /* 52 */
#define D2083_LDO4_REG              0x35    /* 53 */
#define D2083_LDO5_REG              0x36    /* 54 */
#define D2083_LDO6_REG              0x37    /* 55 */
#define D2083_LDO7_REG              0x38    /* 56 */
#define D2083_LDO8_REG              0x39    /* 57 */
#define D2083_LDO9_REG              0x3A    /* 58 */
#define D2083_LDO10_REG             0x3B    /* 59 */
#define D2083_LDO11_REG             0x3C    /* 60 */
#define D2083_LDO12_REG             0x3D    /* 61 */
/* Regulator Register set 1 end */

#define D2083_PULLDOWN_REG_A        0x3E    /* 62 */
#define D2083_PULLDOWN_REG_B        0x3F    /* 63 */
#define D2083_PULLDOWN_REG_C        0x40    /* 64 */
#define D2083_PULLDOWN_REG_D        0x41    /* 65 */

/* Regulator Register set 2 */
#define D2083_LDO13_REG             0x42    /* 66 */
#define D2083_LDO14_REG             0x43    /* 67 */
#define D2083_LDO15_REG             0x44    /* 68 */
#define D2083_LDO16_REG             0x45    /* 69 */
#define D2083_LDO17_REG             0x46    /* 70 */
#define D2083_LDO_AUD_REG           0x47    /* 71 */
/* Regulator Register set 2 end */


#define D2083_SUPPLY_REG            0x48    /* 72 */


/* Regulator MCTL Register */
#define D2083_LDO1_MCTL_REG         0x49    /* 73 */
#define D2083_LDO2_MCTL_REG         0x4A    /* 74 */
#define D2083_LDO3_MCTL_REG         0x4B    /* 75 */
#define D2083_LDO4_MCTL_REG         0x4C    /* 76 */
#define D2083_LDO5_MCTL_REG         0x4D    /* 77 */
#define D2083_LDO6_MCTL_REG         0x4E    /* 78 */
#define D2083_LDO7_MCTL_REG         0x4F    /* 79 */
#define D2083_LDO8_MCTL_REG         0x50    /* 80 */
#define D2083_LDO9_MCTL_REG         0x51    /* 81 */
#define D2083_LDO10_MCTL_REG        0x52    /* 82 */
#define D2083_LDO11_MCTL_REG        0x53    /* 83 */
#define D2083_LDO12_MCTL_REG        0x54    /* 84 */
#define D2083_LDO13_MCTL_REG        0x55    /* 85 */
#define D2083_LDO14_MCTL_REG        0x56    /* 86 */
#define D2083_LDO15_MCTL_REG        0x57    /* 87 */
#define D2083_LDO16_MCTL_REG        0x58    /* 88 */
#define D2083_LDO17_MCTL_REG        0x59    /* 89 */
#define D2083_LDO_AUD_MCTL_REG      0x5A    /* 90 */

#define D2083_BUCK1_MCTL_REG        0x5B    /* 91 */
#define D2083_BUCK2_MCTL_REG        0x5C    /* 92 */
#define D2083_BUCK3_MCTL_REG        0x5D    /* 93 */
#define D2083_BUCK4_MCTL_REG        0x5E    /* 94 */

#define D2083_GPADC_MCTL_REG        0x5F    /* 95 */
#define D2083_MISC_MCTL_REG         0x60    /* 96 */

#define D2083_BUCK1_RETENTION_REG   0x61    /* 97 */
#define D2083_BUCK1_TURBO_REG       0x62    /* 98 */

/* Control */
#define D2083_WAITCONT_REG          0x63    /* 99 */
#define D2083_ONKEYCONT1_REG        0x64    /* 100 */
#define D2083_ONKEYCONT2_REG        0x65    /* 101 */
#define D2083_POWERCONT_REG         0x66    /* 102 */
#define D2083_VDDFAULT_REG          0x67    /* 103 */
#define D2083_BBATCONT_REG          0x68    /* 104 */

/* ADC */
#define D2083_ADC_MAN_REG           0x69    /* 105 */
#define D2083_ADC_CONT_REG          0x6A    /* 106 */
#define D2083_ADC_RES_L_REG         0x6B    /* 107 */
#define D2083_ADC_RES_H_REG         0x6C    /* 108 */
#define D2083_VBAT_RES_REG          0x6D    /* 109 */
#define D2083_TEMP1_RES_REG         0x6F    /* 111 */
#define D2083_TEMP1_HIGHP_REG       0x70    /* 112 */
#define D2083_TEMP1_HIGHN_REG       0x71    /* 113 */
#define D2083_TEMP1_LOW_REG         0x72    /* 114 */
#define D2083_T_OFFSET_REG          0x73    /* 115 */
#define D2083_VF_RES_REG            0x74    /* 116 */
#define D2083_AIN_RES_REG           0x77    /* 119 */
#define D2083_TEMP2_RES_REG         0x7A    /* 122 */
#define D2083_TEMP2_HIGHP_REG       0x7B    /* 123 */
#define D2083_TEMP2_HIGHN_REG       0x7C    /* 124 */
#define D2083_TEMP2_LOW_REG         0x7D    /* 125 */
#define D2083_TJUNC_RES_REG         0x7E    /* 126 */
#define D2083_ADC_RES_AUTO1_REG     0x7F    /* 127 */
#define D2083_ADC_RES_AUTO2_REG     0x80    /* 128 */
#define D2083_ADC_RES_AUTO3_REG     0x81    /* 129 */


/* RTC */
#define D2083_COUNTS_REG            0x82    /* 130 */
#define D2083_COUNTMI_REG           0x83    /* 131 */
#define D2083_COUNTH_REG            0x84    /* 132 */
#define D2083_COUNTD_REG            0x85    /* 133 */
#define D2083_COUNTMO_REG           0x86    /* 134 */
#define D2083_COUNTY_REG            0x87    /* 135 */
#define D2083_ALARMS_REG            0x88    /* 136 */
#define D2083_ALARMMI_REG           0x89    /* 137 */
#define D2083_ALARMH_REG            0x8A    /* 138 */
#define D2083_ALARMD_REG            0x8B    /* 139 */
#define D2083_ALARMMO_REG           0x8C    /* 140 */
#define D2083_ALARMY_REG            0x8D    /* 141 */


/* OTP Config */
#define D2083_PAGE_CONT_P1          0x8E    /* 142 */

#define D2083_CHIPID_REG            0x8F    /* 143 */
#define D2083_CONFIGID_REG          0x90    /* 144 */
#define D2083_OTPCONT_REG           0x91    /* 145 */
#define D2083_OSCTRIM_REG           0x92    /* 146 */
#define D2083_GPID0_REG             0x93    /* 147 */
#define D2083_GPID1_REG             0x94    /* 148 */
#define D2083_GPID2_REG             0x95    /* 149 */
#define D2083_GPID3_REG             0x96    /* 150 */
#define D2083_GPID4_REG             0x97    /* 151 */
#define D2083_GPID5_REG             0x98    /* 152 */
#define D2083_GPID6_REG             0x99    /* 153 */
#define D2083_GPID7_REG             0x9A    /* 154 */
#define D2083_GPID8_REG             0x9B    /* 155 */
#define D2083_GPID9_REG             0x9C    /* 156 */

/* Audio registers */
#define D2083_PREAMP_A_CTRL1_REG    0x9D    /* 157 */
#define D2083_PREAMP_A_CTRL2_REG    0x9E    /* 158 */
#define D2083_PREAMP_B_CTRL1_REG    0x9F    /* 159 */
#define D2083_PREAMP_B_CTRL2_REG    0xa0    /* 160 */

#define D2083_MXHPR_CTRL_REG        0xA1    /* 161 */
#define D2083_MXHPL_CTRL_REG        0xA2    /* 162 */
#define D2083_MXSP_CTRL_REG         0xA3    /* 163 */

#define D2083_SP_CTRL_REG           0xA4    /* 164 */
#define D2083_SP_CFG1_REG           0xA5    /* 165 */
#define D2083_SP_CFG2_REG           0xA6    /* 166 */
#define D2083_SP_ATB_SEL_REG        0xA7    /* 167 */
#define D2083_SP_STATUS_REG         0xA8    /* 168 */

#define D2083_HP_L_CTRL_REG         0xA9    /* 169 */
#define D2083_HP_L_GAIN_REG         0xAA    /* 170 */
#define D2083_HP_L_GAIN_STATUS_REG  0xAB    /* 171 */
#define D2083_HP_R_CTRL_REG         0xAC    /* 172 */
#define D2083_HP_R_GAIN_REG         0xAD    /* 173 */
#define D2083_HP_R_GAIN_STATUS_REG  0xAE    /* 174 */


#define D2083_HP_TEST_REG           0xAF    /* 175 */
#define D2083_CP_CTRL_REG           0xB0    /* 176 */
#define D2083_CP_DELAY_REG          0xB1    /* 177 */
#define D2083_CP_DETECTOR_REG       0xB2    /* 178 */
#define D2083_CP_VOL_THRESHOLD_REG  0xB3    /* 179 */
#define D2083_HP_NG1_REG            0xB4    /* 180 */
#define D2083_HP_NG2_REG            0xB5    /* 181 */
#define D2083_SP_NG1_REG            0xB6    /* 182 */
#define D2083_SP_NG2_REG            0xB7    /* 183 */
#define D2083_SP_NON_CLIP_ZC_REG    0xB8    /* 184 */
#define D2083_SP_NON_CLIP_REG       0xB9    /* 185 */
#define D2083_SP_PWR_REG            0xBA    /* 186 */
#define D2083_SV_CTRL_REG           0xBB    /* 187 */
#define D2083_BIAS_CTRL_REG         0xBC    /* 188 */

#define D2083_PAGE0_REG_START       D2083_STATUSA_REG
#define D2083_PAGE0_REG_END         D2083_ALARMY_REG

#define D2083_PAGE1_REG_START       D2083_CHIPID_REG
#define D2083_PAGE1_REG_END         D2083_BIAS_CTRL_REG

/************************PAGE CONFIGURATION ***************************/

/* PAGE CONFIGURATION 128 REGISTER */
#define D2083_PAGECONT_REGPAGE            (0x1<<7)

/************************SYSTEM REGISTER ***************************/

/* STATUS REGISTER A */
#define D2083_STATUSA_MCTL                  (0x3<<5)
#define D2083_STATUSA_MCTL_SHIFT            (5)
#define D2083_STATUSA_VDDMON                (0x1<<4)

/* STATUS REGISTER B */
#define D2083_STATUSB_SEQUENCING            (0x1<<6)
#define D2083_STATUSB_NONKEY                (0x1<<0)

/* STATUS REGISTER C */
#define D2083_STATUSC_NJIGON                (0x1<<4)
#define D2083_STATUSC_TA                    (0x1<<3)

/* STATUS REGISTER D */
#define D2083_STATUSD_GPI0                  (0x1<<5)

/* EVENT REGISTER A */
#define D2083_EVENTA_ETICK                  (0x1<<7)
#define D2083_EVENTA_ESEQRDY                (0x1<<6)
#define D2083_EVENTA_EALRAM                 (0x1<<5)
#define D2083_EVENTA_EVDDMON                (0x1<<4)
#define D2083_EVENTA_EVDDLOW                (0x1<<3)
#define D2083_EVENTA_ETBAT2                 (0x1<<2)
#define D2083_EVENTA_EVF                    (0x1<<0)

/* EVENT REGISTER B */
#define D2083_EVENTB_EADC_EOM               (0x1<<5)
#define D2083_EVENTB_ETBAT1                 (0x1<<4)
#define D2083_EVENTB_ENONKEY_HOLDOFF        (0x1<<3)
#define D2083_EVENTB_ENONKEY_HOLDON         (0x1<<2)
#define D2083_EVENTB_ENONKEY_HI             (0x1<<1)
#define D2083_EVENTB_ENONKEY_LO             (0x1<<0)

/* EVENT REGISTER C */
#define D2083_EVENTC_ENJIGON                (0x1<<4)
#define D2083_EVENTC_ETA                    (0x1<<3)

/* EVENT REGISTER D */
#define D2083_EVENTC_EGPI0                  (0x1<<5)

/* FAULT LOG REGISTER */
#define D2083_FAULTLOG_WAITSHUT             (0x1<<7)
#define D2083_FAULTLOG_KEYSHUT              (0x1<<5)
#define D2083_FAULTLOG_TEMPOVER             (0x1<<3)
#define D2083_FAULTLOG_VDDSTART             (0x1<<2)
#define D2083_FAULTLOG_VDDFAULT             (0x1<<1)
#define D2083_FAULTLOG_TWDERROR             (0x1<<0)

/* IRQ_MASK REGISTER A */
#define D2083_IRQMASKA_MTICK                (0x1<<7)
#define D2083_IRQMASKA_MSEQRDY              (0x1<<6)
#define D2083_IRQMASKA_MALRAM               (0x1<<5)
#define D2083_IRQMASKA_MVDDMON              (0x1<<4)
/* TODO: Check. There is no bit about MVDDLOW in IRQ_MASK_A register.*/
#define D2083_IRQMASKA_MVDDLOW              (0x1<<3)
#define D2083_IRQMASKA_MTBAT2               (0x1<<2)
#define D2083_IRQMASKA_MVF                  (0x1<<0)

/* IRQ_MASK REGISTER B */
#define D2083_IRQMASKB_MADC_EOM             (0x1<<5)
#define D2083_IRQMASKB_MTBAT1               (0x1<<4)

#define D2083_IRQMASKB_MNONKEY_HOLDOFF      (0x1<<3)
#define D2083_IRQMASKB_MNONKEY_HOLDON       (0x1<<2)
#define D2083_IRQMASKB_MNONKEY_HI           (0x1<<1)
#define D2083_IRQMASKB_MNONKEY_LO           (0x1<<0)

/* IRQ_MASK REGISTER C */
#define D2083_IRQMASKC_MNJIGON              (0x1<<4)
#define D2083_IRQMASKC_MTA                  (0x1<<3)

/* IRQ_MASK REGISTER D */
#define D2083_IRQMASKD_MGPI0                (0x1<<5)

/* CONTROL REGISTER A */
#define D2083_CONTROLA_GPIV                 (0x1<<7)
#define D2083_CONTROLA_PMIV                 (0x1<<4)
#define D2083_CONTROLA_PMIFV                (0x1<<3)
#define D2083_CONTROLA_PWR1EN               (0x1<<2)
#define D2083_CONTROLA_PWREN                (0x1<<1)
#define D2083_CONTROLA_SYSEN                (0x1<<0)

/* CONTROL REGISTER B */
#define D2083_CONTROLB_SHUTDOWN             (0x1<<7)
#define D2083_CONTROLB_DEEPSLEEP            (0x1<<6)
#define D2083_CONTROLB_WRITEMODE            (0x1<<5)
#define D2083_CONTROLB_I2C_SPEED            (0x1<<4)
#define D2083_CONTROLB_OTPREADEN            (0x1<<3)
#define D2083_CONTROLB_AUTOBOOT             (0x1<<2)

/* CONTROL REGISTER C */
#define D2083_CONTROLC_DEBOUNCING           (0x7<<2)
#define D2083_CONTROLC_DEBOUNCING_SHIFT     2
#define D2083_CONTROLC_PMFB1PIN             (0x1<<0)

/* CONTROL REGISTER D */
#define D2083_CONTROLD_WATCHDOG             (0x1<<7)
#define D2083_CONTROLD_ONKEYAUTOBOOTEN      (0x1<<5)
#define D2083_CONTROLD_ONKEYSD              (0x1<<4)
#define D2083_CONTROLD_KEEPACTEN            (0x1<<3)
#define D2083_CONTROLD_TWDSCALE             (0x7<<0)
#define D2083_CONTROLD_TWDSCALE_SHIFT       0

/* POWER DOWN DISABLE REGISTER */
#define D2083_PDDIS_PMCONTPD                (0x1<<7)
#define D2083_PDDIS_OUT32KPD                (0x1<<6)
#define D2083_PDDIS_CHGBBATPD               (0x1<<5)
#define D2083_PDDIS_HS2WIREPD               (0x1<<3)
#define D2083_PDDIS_PMIFPD                  (0x1<<2)
#define D2083_PDDIS_GPADCPD                 (0x1<<1)
#define D2083_PDDIS_GPIOPD                  (0x1<<0)

/* INTERFACE REGISTER */
#define D2083_INTERFACE_IFBASEADDR          (0x7<<5)
#define D2083_INTERFACE_IFBASEADDR_SHIFT    5

/* RESET REGISTER */
#define D2083_RESET_RESETEVENT              (0x3<<6)
#define D2083_RESET_RESETEVENT_SHIFT        6
#define D2083_RESET_RESETTIMER              (0x3F<<0)


/************************GPIO REGISTERS***************************/

/* TA control register */
#define D2083_GPIO_TAMODE                   (0x1<<7)
#define D2083_GPIO_TATYPE                   (0x1<<6)
#define D2083_GPIO_TAPIN                    (0x3<<4)
#define D2083_GPIO_TAPIN_SHIFT              4


/* NJIGON control register */
#define D2083_GPIO_NJIGONMODE               (0x1<<3)
#define D2083_GPIO_NJIGONTYPE               (0x1<<2)
#define D2083_GPIO_NJIGONPIN                (0x3<<0)
#define D2083_GPIO_NJIGONPIN_SHIFT          0


/* GPIO control register */
#define D2083_GPIO_MODE                     (0x1<<7)
#define D2083_GPIO_TYPE                     (0x1<<6)
#define D2083_GPIO_PIN                      (0x3<<4)
#define D2083_GPIO_IN_SHIFT                 4


/*****************POWER SEQUENCER REGISTER*********************/

/* SEQ control register for ID 0 and 1 */
#define D2083_ID01_LDO1STEP                 (0xF<<4)
#define D2083_ID01_WAITIDALWAYS             (0x1<<3)
#define D2083_ID01_SYSPRE                   (0x1<<2)
#define D2083_ID01_DEFSUPPLY                (0x1<<1)
#define D2083_ID01_NRESMODE                 (0x1<<0)

/* SEQ control register for ID 2 and 3 */
#define D2083_ID23_LDO3STEP                 (0xF<<4)
#define D2083_ID23_LDO2STEP                 (0xF<<0)

/* SEQ control register for ID 4 and 5 */
#define D2083_ID45_LDO5STEP                 (0xF<<4)
#define D2083_ID45_LDO4STEP                 (0xF<<0)

/* SEQ control register for ID 6 and 7 */
#define D2083_ID67_LDO7STEP                 (0xF<<4)
#define D2083_ID67_LDO6STEP                 (0xF<<0)

/* SEQ control register for ID 8 and 9 */
#define D2083_ID89_LDO9STEP                 (0xF<<4)
#define D2083_ID89_LDO8STEP                 (0xF<<0)

/* SEQ control register for ID 10 and 11 */
/*#define D2083_ID1011_PDDISSTEP              (0xF<<4)*/
#define D2083_ID1011_LDO11STEP              (0xF<<4)
#define D2083_ID1011_LDO10STEP              (0xF<<0)

/* SEQ control register for ID 12 and 13 */
/*#define D2083_ID1213_LDO13STEP              (0xF<<4)*/
#define D2083_ID1213_PDDISSTEP              (0xF<<4)
#define D2083_ID1213_LDO12STEP              (0xF<<0)

/* SEQ control register for ID 14 and 15 */
#define D2083_ID1415_BUCK2                  (0xF<<4)
#define D2083_ID1415_BUCK1                  (0xF<<0)

/* SEQ control register for ID 16 and 17 */
#define D2083_ID1617_BUCK4                  (0xF<<4)
#define D2083_ID1617_BUCK3                  (0xF<<0)

/* Power SEQ Status register */
#define D2083_SEQSTATUS_SEQPOINTER          (0xF<<4)
#define D2083_SEQSTATUS_WAITSTEP            (0xF<<0)

/* Power SEQ A register */
#define D2083_SEQA_POWEREND                 (0xF<<4)
#define D2083_SEQA_SYSTEMEND                (0xF<<0)

/* Power SEQ B register */
#define D2083_SEQB_PARTDOWN                 (0xF<<4)
#define D2083_SEQB_MAXCOUNT                 (0xF<<0)

/* Power SEQ TIMER register */
#define D2083_SEQTIMER_SEQDUMMY             (0xF<<4)
#define D2083_SEQTIMER_SEQTIME              (0xF<<0)


/***************** REGULATOR REGISTER*********************/

/* BUCK REGISTER A */
#define D2083_BUCKA_BUCK2ILIM               (0x3<<6)
#define D2083_BUCKA_BUCK2ILIM_SHIFT         6
#define D2083_BUCKA_BUCK2MODE               (0x3<<4)
#define D2083_BUCKA_BUCK2MODE_SHIFT         4
#define D2083_BUCKA_BUCK1ILIM               (0x3<<2)
#define D2083_BUCKA_BUCK1ILIM_SHIFT         2
#define D2083_BUCKA_BUCK1MODE               (0x3<<0)
#define D2083_BUCKA_BUCK1MODE_SHIFT         0

/* BUCK REGISTER B */
#define D2083_BUCKB_BUCK4ILIM               (0x3<<6)
#define D2083_BUCKB_BUCK4ILIM_SHIFT         6
#define D2083_BUCKB_BUCK4IMODE              (0x3<<4)
#define D2083_BUCKB_BUCK4IMODE_SHIFT        4
#define D2083_BUCKB_BUCK3ILIM               (0x3<<2)
#define D2083_BUCKB_BUCK3ILIM_SHIFT         2
#define D2083_BUCKB_BUCK3MODE               (0x3<<0)
#define D2083_BUCKB_BUCK3MODE_SHIFT         0


/* PULLDOWN REGISTER A */
#define D2083_PULLDOWN_A_LDO4PDDIS          (0x1<<7)
#define D2083_PULLDOWN_A_LDO3PDDIS          (0x1<<6)
#define D2083_PULLDOWN_A_LDO2PDDIS          (0x1<<5)
#define D2083_PULLDOWN_A_LDO1PDDIS          (0x1<<4)
#define D2083_PULLDOWN_A_BUCK4PDDIS         (0x1<<3)
#define D2083_PULLDOWN_A_BUCK3PDDIS         (0x1<<2)
#define D2083_PULLDOWN_A_BUCK2PDDIS         (0x1<<1)
#define D2083_PULLDOWN_A_BUCK1PDDIS         (0x1<<0)


/* PULLDOWN REGISTER B */
#define D2083_PULLDOWN_B_LDO12PDDIS         (0x1<<7)
#define D2083_PULLDOWN_B_LDO11PDDIS         (0x1<<6)
#define D2083_PULLDOWN_B_LDO10PDDIS         (0x1<<5)
#define D2083_PULLDOWN_B_LDO9PDDIS          (0x1<<4)
#define D2083_PULLDOWN_B_LDO8PDDIS          (0x1<<3)
#define D2083_PULLDOWN_B_LDO7PDDIS          (0x1<<2)
#define D2083_PULLDOWN_B_LDO6PDDIS          (0x1<<1)
#define D2083_PULLDOWN_B_LDO5PDDIS          (0x1<<0)


/* PULLDOWN REGISTER C */
#define D2083_PULLDOWN_C_LDOAUDPDDIS        (0x1<<5)
#define D2083_PULLDOWN_C_LDO17PDDIS         (0x1<<4)
#define D2083_PULLDOWN_C_LDO16PDDIS         (0x1<<3)
#define D2083_PULLDOWN_C_LDO15PDDIS         (0x1<<2)
#define D2083_PULLDOWN_C_LDO14PDDIS         (0x1<<1)
#define D2083_PULLDOWN_C_LDO13PDDIS         (0x1<<0)


/* PULLDOWN REGISTER D */


/* SUPPLY REGISTER */
#define D2083_SUPPLY_VLOCK                  (0x1<<7)
#define D2083_SUPPLY_BUCK4EN                (0x1<<6)
#define D2083_SUPPLY_BBCHGEN                (0x1<<4)
#define D2083_SUPPLY_VBUCK4GO               (0x1<<3)
#define D2083_SUPPLY_VBUCK3GO               (0x1<<2)
#define D2083_SUPPLY_VBUCK2GO               (0x1<<1)
#define D2083_SUPPLY_VBUCK1GO               (0x1<<0)

/* MISC MCTL */
#define D2083_MISC_MCTL3_DIGICLK            (0x1<<7)
#define D2083_MISC_MCTL2_DIGICLK            (0x1<<6)
#define D2083_MISC_MCTL1_DIGICLK            (0x1<<5)
#define D2083_MISC_MCTL0_DIGICLK            (0x1<<4)
#define D2083_MISC_MCTL3_BBAT               (0x1<<3)
#define D2083_MISC_MCTL2_BBAT               (0x1<<2)
#define D2083_MISC_MCTL1_BBAT               (0x1<<1)
#define D2083_MISC_MCTL0_BBAT               (0x1<<0)


/* BUCK 1 Retention register */
#define D2083_BUCK1_VRETENTION              (0x3F<<0)

/* Buck 1 Turbo register */
#define D2083_BUCK1_VTURBO                  (0x3F<<0)


/* WAIT CONTROL REGISTER */
#define D2083_WAITCONT_WAITDIR              (0x1<<7)
#define D2083_WAITCONT_RTCCLOCK             (0x1<<6)
#define D2083_WAITCONT_WAITMODE             (0x1<<5)
#define D2083_WAITCONT_EN32KOUT             (0x1<<4)
#define D2083_WAITCONT_DELAYTIME            (0xF<<0)

/* ONKEY CONTROL REGISTER */
#define D2083_ONKEYCONT_DEB                 (0xF<<4)
#define D2083_ONKEYCONT_PRESSTIME           (0xF<<0)

/* ONKEY CONTROL REGISTER */
#define D2083_ONKEYCONT_HOLDOFFDEB          (0x7<<4)
#define D2083_ONKEYCONT_HOLDONDEB           (0x7<<0)

/* POWER CONTROL REGISTER */
#define D2083_POWERCONT_NJIGMCTRLWAKEDIS    (0x1<<7)
#define D2083_POWERCONT_RTCAUTOEN           (0x1<<6)
#define D2083_POWERCONT_BBATILIMIGNORE      (0x1<<3)
#define D2083_POWERCONT_MCTRLEN             (0x1<<0)

/* VDD FAULT REGISTER */
#define D2083_VDD_FAULT_ADJ                 (0xF<<2)
#define D2083_VDD_HYST_ADJ                  (0x3<<0)


/***************** BBAT CHARGER REGISTER *********************/

/* BACKUP BATTERY CONTROL REGISTER */
#define D2083_BBATCONT_BCHARGERISET         (0xF<<4)
#define D2083_BBATCONT_BCHARGERVSET         (0xF<<0)


/***************** ADC REGISTER *********************/

/* ADC MAN REGISTER */
#define D2083_ADC_MAN_ISRC_50U              (0x1<<7)
#define D2083_ADC_MAN_CONV                  (0x1<<4)

#define D2083_ADCMAN_MUXSEL_VBAT	        (0x0<<0)
#define D2083_ADCMAN_MUXSEL_TEMP1           (0x2<<0)
#define D2083_ADCMAN_MUXSEL_VF              (0x4<<0)
#define D2083_ADCMAN_MUXSEL_TEMP2           (0x6<<0)
#define D2083_ADCMAN_MUXSEL_TJUNC           (0x8<<0)

/* ADC CONT REGISTER */
#define D2083_ADCCONT_ADC_AUTO_EN           (0x1<<7)
#define D2083_ADCCONT_ADC_MODE              (0x1<<6)
#define D2083_ADCCONT_TEMP1_ISRC_EN         (0x1<<5)
#define D2083_ADCCONT_VF_ISRC_EN            (0x1<<4)
#define D2083_ADCCONT_TEMP2_ISRC_EN         (0x1<<3)
#define D2083_ADCCONT_AUTO_AIN_EN           (0x1<<2)
#define D2083_ADCCONT_AUTO_VF_EN            (0x1<<1)
#define D2083_ADCCONT_AUTO_VBAT_EN          (0x1<<0)

#define D2083_ADC_RES_AUTO1_TEMP1_RES_LSB   (0xF<<4)
#define D2083_ADC_RES_AUTO1_VBAT_RES_LSB    (0xF<<0)

#define D2083_ADC_RES_AUTO2_AIN_RES_LSB     (0xF<<4)
#define D2083_ADC_RES_AUTO2_VF_RES_LSB      (0xF<<0)

#define D2083_ADC_RES_AUTO3_TJUNC_RES_LSB   (0xF<<4)
#define D2083_ADC_RES_AUTO3_TEMP2_RES_LSB   (0xF<<0)


/***************** RTC REGISTER *********************/

/* RTC TIMER SECONDS REGISTER */
#define D2083_COUNTS_COUNTSEC               (0x3F<<0)

/* RTC TIMER MINUTES REGISTER */
#define D2083_COUNTMI_COUNTMIN              (0x3F<<0)

/* RTC TIMER HOUR REGISTER */
#define D2083_COUNTH_COUNTHOUR              (0x1F<<0)

/* RTC TIMER DAYS REGISTER */
#define D2083_COUNTD_COUNTDAY               (0x1F<<0)

/* RTC TIMER MONTHS REGISTER */
#define D2083_COUNTMO_COUNTMONTH            (0xF<<0)

/* RTC TIMER YEARS REGISTER */
#define D2083_COUNTY_MONITOR                (0x1<<6)
#define D2083_COUNTY_COUNTYEAR              (0x3F<<0)

/* RTC ALARM SECONDS REGISTER */
#define D2083_ALARMMI_COUNTSEC              (0x3F<<0)

/* RTC ALARM MINUTES REGISTER */
#define D2083_ALARMMI_TICKTYPE              (0x1<<7)
#define D2083_ALARMMI_ALARMMIN              (0x3F<<0)

/* RTC ALARM HOURS REGISTER */
#define D2083_ALARMH_ALARMHOUR              (0x1F<<0)

/* RTC ALARM DAYS REGISTER */
#define D2083_ALARMD_ALARMDAY               (0x1F<<0)

/* RTC ALARM MONTHS REGISTER */
#define D2083_ALARMMO_ALARMMONTH            (0xF<<0)

/* RTC ALARM YEARS REGISTER */
#define D2083_ALARMY_TICKON                 (0x1<<7)
#define D2083_ALARMY_ALARMON                (0x1<<6)
#define D2083_ALARMY_ALARMYEAR              (0x3F<<0)


/*****************OTP REGISTER*********************/
#define D2083_PAGE_CONT_P1_BIT              (0x1<<7)

/* CHIP IDENTIFICATION REGISTER */
#define D2083_CHIPID_MRC                    (0xF<<4)
#define D2083_CHIPID_TRC                    (0xF<<0)

/* CONFIGURATION IDENTIFICATION REGISTER */
#define D2083_CONFIGID_CONFID               (0x7<<0)
#define D2083_CONFIGID_CONFID_SHIFT         0

/* OTP CONTROL REGISTER */
#define D2083_OTPCONT_GPWRITEDIS            (0x1<<7)
#define D2083_OTPCONT_OTPCONFLOCK           (0x1<<6)
#define D2083_OTPCONT_OTPGPLOCK             (0x1<<5)
#define D2083_OTPCONT_OTPCONFG              (0x1<<3)
#define D2083_OTPCONT_OTPGP                 (0x1<<2)
#define D2083_OTPCONT_OTPRP                 (0x1<<1)
#define D2083_OTPCONT_OTPTRANSFER           (0x1<<0)

/* RTC OSCILLATOR TRIM REGISTER */
#define D2083_OSCTRIM_TRIM32K               (0xFF<<0)

/* GP ID REGISTER 0 */
#define D2083_GPID0_GP0                     (0xFF<<0)

/* GP ID REGISTER 1 */
#define D2083_GPID1_GP1                     (0xFF<<0)

/* GP ID REGISTER 2 */
#define D2083_GPID2_GP2                     (0xFF<<0)

/* GP ID REGISTER 3 */
#define D2083_GPID3_GP3                     (0xFF<<0)

/* GP ID REGISTER 4 */
#define D2083_GPID4_GP4                     (0xFF<<0)

/* GP ID REGISTER 5 */
#define D2083_GPID5_GP5                     (0xFF<<0)

/* GP ID REGISTER 6 */
#define D2083_GPID6_GP6                     (0xFF<<0)

/* GP ID REGISTER 7 */
#define D2083_GPID7_GP7                     (0xFF<<0)

/* GP ID REGISTER 8 */
#define D2083_GPID8_GP8                     (0xFF<<0)

/* GP ID REGISTER 9 */
#define D2083_GPID9_GP9                     (0xFF<<0)

/***************** AUDIO CONF REGISTERS********************/

/* Preamp CTRL1 regs (A and B) */
#define D2083_PREAMP_ZC_EN                  (0x1<<7)
#define D2083_PREAMP_VOL                    (0x1F<<2)
#define D2083_PREAMP_VOL_SHIFT              2
#define D2083_PREAMP_MUTE                   (0x1<<1)
#define D2083_PREAMP_EN                     (0x1<<0)
/* Preamp CTRL2 regs (A and B) */
#define D2083_PREAMP_CFG                    (0x3<<0)

/* Mixer control registers (HPR, HPL and SP) */
#define D2083_MX_VOL                        (0x3<<5)
#define D2083_MX_VOL_SHIFT                  5
#define D2083_MX_SEL                        (0xF<<1)
#define D2083_MX_SEL_SHIFT                  1
#define D2083_MX_EN                         (0x1<<0)

/* Speaker control register */
#define D2083_SP_VOL                        (0x3F<<2)
#define D2083_SP_VOL_SHIFT                  2
#define D2083_SP_MUTE                       (0x1<<1)
#define D2083_SP_EN                         (0x1<<0)

/* Speaker CFG1 register */
#define D2083_SP_CFG1                       (0xFF<<0)
/* Speaker CFG2 register */
#define D2083_SP_CFG2                       (0xFF<<0)
/* Speaker ATB_SEL register */
#define D2083_SP_ATB_SEL                    (0xFF<<0)
/* Speaker STATUS register */
#define D2083_SP_STATUS                     (0xFF<<0)

/* Headphones control registers (Left and Right) */
#define D2083_HP_AMP_EN                     (0x1<<7)
#define D2083_HP_AMP_MUTE_EN                (0x1<<6)
#define D2083_HP_AMP_RAMP_EN                (0x1<<5)
#define D2083_HP_AMP_ZC_EN                  (0x1<<4)
#define D2083_HP_AMP_OE                     (0x1<<3)
#define D2083_HP_AMP_MIN_GAIN_EN            (0x1<<2)
#define D2042_HP_AMP_BIAS                   (0x3<<0)

/* Headphones amplifier gain register (Left and Right) */
#define D2042_HP_AMP_GAIN                   (0x3F<<0)

/* Headphones amplifier gain status register (Left and Right) */
#define D2042_HP_AMP_GAIN_STATUS            (0x3F<<0)


/* D2083_HP_TEST_REG */
#define D2083_HP_AMP_EMS_EN                 (0x1<<0)

/* D2083_CP_CTRL_REG */
#define D2083_CP_EN                         (0x1<<7)
#define D2083_CP_SMALL_SWITCH_FREQ_EN       (0x1<<6)
#define D2083_CP_MCHANGE                    (0x3<<4)
#define D2083_CP_MCHANGE_SHIFT              4
#define D2083_CP_MOD                        (0x3<<2)
#define D2083_CP_MOD_SHIFT                  2
#define D2083_CP_ANALOGUE_LVL               (0x3<<0)
#define D2083_CP_ANALOGUE_LVL_SHIFT         0

/* D2083_DELAY_REG */
#define D2083_CP_ONOFF                      (0x3<<6)
#define D2083_CP_ONOFF_SHIFT                6
#define D2083_CP_TAU_DELAY                  (0x7<<3)
#define D2083_CP_TAU_DELAY_SHIFT            3
#define D2083_CP_FCONTROL                   (0x7<<0)
#define D2083_CP_FCONTROL_SHIFT             0

/* D2083_CP_DETECTOR_REG */
#define D2083_CP_DET_DROP                   (0x3<<0)
#define D2083_CP_DET_DROP_SHIFT             0

/* D2083_CP_VOL_THRESHOLD_REG */
#define D2083_CP_THRESH_VDD2                (0x3F<<0)
#define D2083_CP_THRESH_VDD2_SHIFT          0

/* D2083_HP_NG1_REG */
#define D2083_HP_NG1_CFG                    (0x3<<6)
#define D2083_HP_NG1_CFG_SHIFT              6
#define D2083_HP_NG1_ATK                    (0x7<<3)
#define D2083_HP_NG1_ATK_SHIFT              3
#define D2083_HP_NG1_DEB                    (0x3<<1)
#define D2083_HP_NG1_DEB_SHIFT              1
#define D2083_HP_NG1_EN                     (0x1<<0)

/* D2083_HP_NG2_REG */
#define D2083_HP_NG2_RMS                    (0x3<<3)
#define D2083_HP_NG2_RMS_SHIFT              3
#define D2083_HP_NG2_REL                    (0x7<<0)
#define D2083_HP_NG2_REL_SHIFT              0

/* D2083_SP_NG1_REG */
#define D2083_SP_NG1_CFG                    (0x3<<6)
#define D2083_SP_NG1_CFG_SHIFT              6
#define D2083_SP_NG1_ATK                    (0x7<<3)
#define D2083_SP_NG1_ATK_SHIFT              3
#define D2083_SP_NG1_DEB                    (0x3<<1)
#define D2083_SP_NG1_DEB_SHIFT              1
#define D2083_SP_NG1_EN                     (0x1<<0)

/* D2083_SP_NG2_REG */
#define D2083_SP_NG2_RMS                    (0x3<<3)
#define D2083_SP_NG2_RMS_SHIFT              3
#define D2083_SP_NG2_REL                    (0x7<<0)
#define D2083_SP_NG2_REL_SHIFT              0

/* D2083_SP_NON_CLIP_ZC_REG */
#define D2083_SP_ZC_EN                      (0x1<<7)
#define D2083_SP_NON_CLIP_REL               (0x7<<4)
#define D2083_SP_NON_CLIP_REL_SHIFT         4
#define D2083_SP_NON_CLIP_ATK               (0x7<<1)
#define D2083_SP_NON_CLIP_ATK_SHIFT         1
#define D2083_SP_NON_CLIP_EN                (0x1<<0)

/* D2083_SP_NON_CLIP_REG */
#define D2083_SP_NON_CLIP_HLD               (0x3<<6)
#define D2083_SP_NON_CLIP_HLD_SHIFT         6
#define D2083_SP_NON_CLIP_THD               (0x3F<<0)
#define D2083_SP_NON_CLIP_THD_SHIFT         0

/* D2083_SP_PWR_REG */
#define D2083_SP_PWR                        (0x3F<<1)
#define D2083_SP_PWR_SHIFT                  1
#define D2083_SP_PWR_LIMIT_EN               (0x1<<0)

/* D2083_SV_CTRL_REG */

/* D2083_BIAS_CTRL_REG */
#define D2083_BIAS_CTRL                     (0xFF<<0)


#endif /* __LINUX_D2083_REG_H */
