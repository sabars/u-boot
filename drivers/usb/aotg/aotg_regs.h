/*
 * USB AOTG driver (Actions OTG controller)
 *
 * Copyright (c) 2017 Actions Semiconductor Co.Ltd.
 * lvjinang <lvjinang@actions-semi.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#ifndef __AOTG_REGS_H__
#define __AOTG_REGS_H__

/* Byte Counter */
#define OUT0BC                  0x00000000
#define HCIN0BC                 0x00000000
#define IN0BC                   0x00000001
#define HCOUT0BC                0x00000001
#define EP0CS                   0x00000002
#define HCEP0CS                 0x00000002
#define OUT1BCL                 0x00000008
#define HCIN1BCL                0x00000008
#define OUT1BCH                 0x00000009
#define HCIN1BCH                0x00000009
#define OUT1CON                 0x0000000A
#define HCIN1CON                0x0000000A
#define OUT1CS                  0x0000000B
#define HCIN1CS                 0x0000000B
#define IN1BCL                  0x0000000C
#define HCOUT1BCL               0x0000000C
#define IN1BCH                  0x0000000D
#define HCOUT1BCH               0x0000000D
#define IN1CON                  0x0000000E
#define HCOUT1CON               0x0000000E
#define IN1CS                   0x0000000F
#define HCOUT1CS                0x0000000F
#define OUT2BCL                 0x00000010
#define HCIN2BCL                0x00000010
#define OUT2BCH                 0x00000011
#define HCIN2BCH                0x00000011
#define OUT2CON                 0x00000012
#define HCIN2CON                0x00000012
#define OUT2CS                  0x00000013
#define HCIN2CS                 0x00000013
#define IN2BCL                  0x00000014
#define HCOUT2BCL               0x00000014
#define IN2BCH                  0x00000015
#define HCOUT2BCH               0x00000015
#define IN2CON                  0x00000016
#define HCOUT2CON               0x00000016
#define IN2CS                   0x00000017
#define HCOUT2CS                0x00000017
#define OUT3BCL                 0x00000018
#define HCIN3BCL                0x00000018
#define OUT3BCH                 0x00000019
#define HCIN3BCH                0x00000019
#define OUT3CON                 0x0000001A
#define HCIN3CON                0x0000001A
#define OUT3CS                  0x0000001B
#define HCIN3CS                 0x0000001B
#define IN3BCL                  0x0000001C
#define HCOUT3BCL               0x0000001C
#define IN3BCH                  0x0000001D
#define HCOUT3BCH               0x0000001D
#define IN3CON                  0x0000001E
#define HCOUT3CON               0x0000001E
#define IN3CS                   0x0000001F
#define HCOUT3CS                0x0000001F
#define OUT4BCL                 0x00000020
#define HCIN4BCL                0x00000020
#define OUT4BCH                 0x00000021
#define HCIN4BCH                0x00000021
#define OUT4CON                 0x00000022
#define HCIN4CON                0x00000022
#define OUT4CS                  0x00000023
#define HCIN4CS                 0x00000023
#define IN4BCL                  0x00000024
#define HCOUT4BCL               0x00000024
#define IN4BCH                  0x00000025
#define HCOUT4BCH               0x00000025
#define IN4CON                  0x00000026
#define HCOUT4CON               0x00000026
#define IN4CS                   0x00000027
#define HCOUT4CS                0x00000027
#define OUT5BCL                 0x00000028
#define HCIN5BCL                0x00000028
#define OUT5BCH                 0x00000029
#define HCIN5BCH                0x00000029
#define OUT5CON                 0x0000002A
#define HCIN5CON                0x0000002A
#define OUT5CS                  0x0000002B
#define HCIN5CS                 0x0000002B
#define IN5BCL                  0x0000002C
#define HCOUT5BCL               0x0000002C
#define IN5BCH                  0x0000002D
#define HCOUT5BCH               0x0000002D
#define IN5CON                  0x0000002E
#define HCOUT5CON               0x0000002E
#define IN5CS                   0x0000002F
#define HCOUT5CS                0x0000002F


/* FIFO Buffer */
#define FIFO1DATA               0x00000084
#define FIFO2DATA               0x00000088
#define FIFO3DATA               0x0000008C
#define FIFO4DATA               0x00000090
#define FIFO5DATA               0x00000094


#define HCEP0CTRL               0x000000C0
#define HCOUT0ERR               0x000000C1
#define HCIN0ERR                0x000000C3
#define HCOUT1CTRL              0x000000C4
#define HCOUT1ERR               0x000000C5
#define HCIN1CTRL               0x000000C6
#define HCIN1ERR                0x000000C7
#define HCOUT2CTRL              0x000000C8
#define HCOUT2ERR               0x000000C9
#define HCIN2CTRL               0x000000CA
#define HCIN2ERR                0x000000CB
#define HCOUT3CTRL              0x000000CC
#define HCOUT3ERR               0x000000CD
#define HCIN3CTRL               0x000000CE
#define HCIN3ERR                0x000000CF
#define HCOUT4CTRL              0x000000D0
#define HCOUT4ERR               0x000000D1
#define HCIN4CTRL               0x000000D2
#define HCIN4ERR                0x000000D3
#define HCOUT5CTRL              0x000000D4
#define HCOUT5ERR               0x000000D5
#define HCIN5CTRL               0x000000D6
#define HCIN5ERR                0x000000D7

#define EP0INDAT                0x00000100
#define HCEP0OUTDAT             0x00000100

#define EP0INDATA_W0            0x00000100
#define EP0INDATA_W1            0x00000104
#define EP0INDATA_W2            0x00000108
#define EP0INDATA_W3            0x0000010C
#define EP0INDATA_W4            0x00000110
#define EP0INDATA_W5            0x00000114
#define EP0INDATA_W6            0x00000118
#define EP0INDATA_W7            0x0000011C
#define EP0INDATA_W8            0x00000120
#define EP0INDATA_W9            0x00000124
#define EP0INDATA_W10           0x00000128
#define EP0INDATA_W11           0x0000012C
#define EP0INDATA_W12           0x00000130
#define EP0INDATA_W13           0x00000134
#define EP0INDATA_W14           0x00000138
#define EP0INDATA_W15           0x0000013C
#define EP0OUTDATA_W0           0x00000140
#define EP0OUTDATA_W1           0x00000144
#define EP0OUTDATA_W2           0x00000148
#define EP0OUTDATA_W3           0x0000014C
#define EP0OUTDATA_W4           0x00000150
#define EP0OUTDATA_W5           0x00000154
#define EP0OUTDATA_W6           0x00000158
#define EP0OUTDATA_W7           0x0000015C
#define EP0OUTDATA_W8           0x00000160
#define EP0OUTDATA_W9           0x00000164
#define EP0OUTDATA_W10          0x00000168
#define EP0OUTDATA_W11          0x0000016C
#define EP0OUTDATA_W12          0x00000170
#define EP0OUTDATA_W13          0x00000174
#define EP0OUTDATA_W14          0x00000178
#define EP0OUTDATA_W15          0x0000017C
#define SETUPDATA_W0            0x00000180
#define SETUPDATA_W1            0x00000184

#define IN07IRQ                 0x00000188
#define HCOUT07IRQ              0x00000188
#define OUT07IRQ                0x0000018A
#define HCIN07IRQ               0x0000018A
#define USBIRQ                  0x0000018C
#define OUT07PNGIRQ             0x0000018E
#define INTXKIRQ                0x00000190
#define OUTXTOKIRQ              0x00000190
#define OUT07EMPTIRQ            0x00000191
#define HCIN07EMPTIRQ           0x00000191
#define IN07IEN                 0x00000194
#define HCOUT07IEN              0x00000194
#define OUT07IEN                0x00000196
#define HCIN07IEN               0x00000196
#define USBIEN                  0x00000198
#define OUT07PNGIEN             0x0000019A
#define INTXKIEN                0x0000019C
#define OUTXTOKIEN              0x0000019D
#define IVECT                   0x000001A0
#define ENDPRST                 0x000001A2
#define HCENDPRST               0x000001A2
#define USBCS                   0x000001A3
#define FRMNRL                  0x000001A4
#define FRMNFH                  0x000001A5
#define FNADDR                  0x000001A6
#define CLKGATE                 0x000001A7
#define FIFOCTRL                0x000001A8
#define HCPORTCTRL              0x000001AB
#define HCFRMNRL                0x000001AC
#define HCFRMNRH                0x000001AD
#define HCFRMREMAINL            0x000001AE
#define HCFRMREMAINH            0x000001AF
#define HCIN07ERRIRQ            0x000001B4
#define HCOUT07ERRIRQ           0x000001B6
#define HCIN07ERRIEN            0x000001B8
#define HCOUT07ERRIEN           0x000001BA
#define OTGIRQ                  0x000001BC
#define OTGSTATE                0x000001BD
#define OTGCTRL                 0x000001BE
#define OTGSTATUS               0x000001BF
#define OTGIEN                  0x000001C0
#define TAAIDLBDIS              0x000001C1
#define TAWAITBCON              0x000001C2
#define TBVBUSPLS               0x000001C3
#define TBVBUSDISCHPLS          0x000001C7
#define HCIN0MAXPCK             0x000001E0
#define HCIN1MAXPCKL            0x000001E2
#define HCIN1MAXPCKH            0x000001E3
#define HCIN2MAXPCKL            0x000001E4
#define HCIN2MAXPCKH            0x000001E5
#define HCIN3MAXPCKL            0x000001E6
#define HCIN3MAXPCKH            0x000001E7
#define HCIN4MAXPCKL            0x000001E8
#define HCIN4MAXPCKH            0x000001E9
#define HCIN5MAXPCKL            0x000001EA
#define HCIN5MAXPCKH            0x000001EB


/* Start Address */
#define OUT1STARTADDRESS        0x00000304
#define OUT1STARTADDRESSL       0x00000304
#define OUT1STARTADDRESSH       0x00000305
#define OUT2STARTADDRESS        0x00000308
#define OUT2STARTADDRESSL       0x00000308
#define OUT2STARTADDRESSH       0x00000309

#define OUT3STARTADDRESS        0x0000030c
#define OUT4STARTADDRESS        0x00000310
#define OUT5STARTADDRESS        0x00000314

#define IN1STARTADDRESS         0x00000344
#define IN1STARTADDRESSL        0x00000344
#define IN1STARTADDRESSH        0x00000345
#define IN2STARTADDRESS         0x00000348
#define IN2STARTADDRESSL        0x00000348
#define IN2STARTADDRESSH        0x00000349

#define IN3STARTADDRESS         0x0000034C
#define IN4STARTADDRESS         0x00000350
#define IN5STARTADDRESS         0x00000354

#define HCIN1STARTADDR          0x00000304
#define HCIN2STARTADDR          0x00000308
#define HCIN3STARTADDR          0x0000030C
#define HCIN4STARTADDR          0x00000310
#define HCIN5STARTADDR          0x00000314

#define HCOUT1STARTADDR         0x00000344
#define HCOUT2STARTADDR         0x00000348
#define HCOUT3STARTADDR         0x0000034C
#define HCOUT4STARTADDR         0x00000350
#define HCOUT5STARTADDR         0x00000354


/* Max Packet */
#define HCOUT0MAXPCK            0x000003E0
#define HCOUT1MAXPCKL           0x000003E2
#define HCOUT1MAXPCKH           0x000003E3
#define HCOUT2MAXPCKL           0x000003E4
#define HCOUT2MAXPCKH           0x000003E5
#define HCOUT3MAXPCKL           0x000003E6
#define HCOUT3MAXPCKH           0x000003E7
#define HCOUT4MAXPCKL           0x000003E8
#define HCOUT4MAXPCKH           0x000003E9
#define HCOUT5MAXPCKL           0x000003EA
#define HCOUT5MAXPCKH           0x000003EB


#define USBERESET               0x00000400
#define TA_BCON_COUNT           0x00000401
#define VBUSDBCTIMERL           0x00000402
#define VBUSDBCTIMERH           0x00000403
#define VDCTRL                  0x00000404
#define VDSTATE                 0x00000405
#define BKDOOR                  0x00000406
#define DBGMODE                 0x00000407
#define SRPCTRL                 0x00000408
#define USBEIRQ                 0x0000040A
#define USBEIEN                 0x0000040C
#define UDMAIRQ                 0x0000040E
#define UDMAIEN                 0x0000040F
#define OUTXSHORTPCKIRQ         0x00000410
#define OUTXSHORTPCKIEN         0x00000412
#define OUTXNAKCTRL             0x00000414
#define HCINXNAKCTRL            0x00000414
#define HCINXSTART              0x00000416
#define HCINXENDIRQ             0x00000418
#define HCINXENDIEN             0x0000041A

#define HCIN1_COUNTL            0x00000420
#define HCIN1_COUNTH            0x00000421
#define HCIN2_COUNTL            0x00000424
#define HCIN2_COUNTH            0x00000425
#define HCIN3_COUNTL            0x00000428
#define HCIN3_COUNTH            0x00000429
#define HCIN4_COUNTL            0x0000042c
#define HCIN4_COUNTH            0x0000042d
#define HCIN5_COUNTL            0x00000430
#define HCIN5_COUNTH            0x00000431

#define INXBUFEMPTYIRQ          0x00000440
#define INXBUFEMPTYIEN          0x00000442
#define INXBUFEMPTYCTRL         0x00000444
#define UDMA1MEMADDR            0x00000450
#define UDMA1EPSEL              0x00000454
#define UDMA1COM                0x00000455
#define UDMA1CNTL               0x00000458
#define UDMA1CNTM               0x00000459
#define UDMA1CNTH               0x0000045A
#define UDMA1REML               0x0000045C
#define UDMA1REMM               0x0000045D
#define UDMA1REMH               0x0000045E
#define UDMA2MEMADDR            0x00000460
#define UDMA2EPSEL              0x00000464
#define UDMA2COM                0x00000465
#define UDMA2CNTL               0x00000468
#define UDMA2CNTM               0x00000469
#define UDMA2CNTH               0x0000046A
#define UDMA2REML               0x0000046C
#define UDMA2REMM               0x0000046D
#define UDMA2REMH               0x0000046E

#define USBEIRQ_USBIRQ          (0x1 << 7)
#define USBEIRQ_USBIEN          (0x1 << 7)

/*USBERES*/
#define USBERES_USBRESET        (1 << 0)

/*VDCTRL*/
#define VDCTRL_VLOAD            (1 << 4)
#define VDCTRL_VCONTROL(x)      ((x) & 0xf)

/*OTGIRQ*/
#define OTGIRQ_PERIPH           (1<<4)
#define OTGIRQ_VBUSEER          (1<<3)
#define OTGIRQ_LOCSOF           (1<<2)
#define OTGIRQ_SRPDET           (1<<1)
#define OTGIRQ_IDLE             (1<<0)

/* OTGSTATE: OTG FSM State */
#define A_IDLE                  (0x00)
#define A_WAIT_VRISE            (0x01)
#define A_WAIT_BCON             (0x02)
#define A_HOST                  (0x03)
#define A_SUSPEND               (0x04)
#define A_PHERIPHERAL           (0x05)
#define A_VBUS_ERR              (0x06)
#define A_WAIT_VFAL             (0x07)
#define B_IDLE                  (0x08)
#define B_PERIPHERAL            (0x09)
#define B_WAIT_ACON             (0x0a)
#define B_HOST                  (0x0b)
#define B_SRP_INIT1             (0x0c)
#define B_SRP_INIT2             (0x0d)
#define B_DISCHRG1              (0x0e)
#define B_DISCHRG2              (0x0f)

/* OTGCTRL */
#define OTGCTRL_FORCEBCONN      (1 << 7)
#define OTGCTRL_SRPDATDETEN     (1 << 5)
#define OTGCTRL_SRPVBUSDETEN    (1 << 4)
#define OTGCTRL_BHNPEN          (1 << 3)
#define OTGCTRL_ASETBHNPEN      (1 << 2)
#define OTGCTRL_ABUSDROP        (1 << 1)
#define OTGCTRL_BUSREQ          (1 << 0)

/* OTGSTATUS */
#define OTGSTATUS_ID            (1 << 6)
#define OTGSTATUS_AVBUSSVAL     (1 << 5)
#define OTGSTATUS_BSESSEND      (1 << 4)
#define OTGSTATUS_ASESSVAL      (1 << 3)
#define OTGSTATUS_BSESSVAL      (1 << 2)
#define OTGSTATUS_CONN          (1 << 1)
#define OTGSTATUS_BSE0SRP       (1 << 0)

/* OTGIEN */
#define OTGIEN_PERIPH           (1 << 4)
#define OTGIEN_VBUSEER          (1 << 3)
#define OTGIEN_LOCSOF           (1 << 2)
#define OTGIEN_SRPDET           (1 << 1)
#define OTGIEN_IDLE             (1 << 0)

/* HCEP0CTRL */
#define HCEP0CTRL_ENDPNR(x)     (((x) & 0xf) << 0)

/* HCOUTXCTRL */
#define HCOUTXCTRL(x)           (((x) & 0xf) << 0)

/* EP0CS */
#define EP0CS_HCSETTOOGLE       (1 << 6)
#define EP0CS_HCCLRTOOGLE       (1 << 5)
#define EP0CS_HCSET             (1 << 4)
#define EP0CS_HCINBSY           (1 << 3)
#define EP0CS_HCOUTBSY          (1 << 2)
#define EP0CS_OUTBSY            (1 << 3)
#define EP0CS_INBSY             (1 << 2)
#define EP0CS_HSNAK             (1 << 1)
#define EP0CS_STALL             (1 << 0)

/* EPxCS: Control and Status */
#define EPCS_AUTO_IN            (1 << 4)
#define EPCS_AUTO_OUT           (1 << 4)
#define EPCS_NPAK               (0x3 << 2)
#define EPCS_BUSY               (1 << 1)
#define EPCS_ERR                (1 << 0)

/* EPXCON host & device */
#define EPCON_VAL               (1 << 7)
#define EPCON_STALL             (1 << 6)
#define EPCON_TYPE              (0x3 << 2)
#define EPCON_TYPE_INT          (0x3 << 2)
#define EPCON_TYPE_BULK         (0x2 << 2)
#define EPCON_TYPE_ISO          (0x1 << 2)
#define EPCON_BUF               (0x03)
#define EPCON_BUF_QUAD          (0x03)
#define EPCON_BUF_TRIPLE        (0x02)
#define EPCON_BUF_DOUBLE        (0x01)
#define EPCON_BUF_SINGLE        (0x00)

/* OUTXIRQ */
#define OUTXIRQ(x)              (1 << (x))

/* INXIRQ */
#define INXIRQ(x)               (1 << (x))

/* USBIRQ */
#define USBIRQ_HS               (1 << 5)
#define USBIRQ_URES             (1 << 4)
#define USBIRQ_SUSP             (1 << 3)
#define USBIRQ_SUTOK            (1 << 2)
#define USBIRQ_SOF              (1 << 1)
#define USBIRQ_SUDAV            (1 << 0)

/* OUTXIEN */
#define OUTXIEN(x)              (1 << (x))

/* INXIEN */
#define INXIEN(x)               (1 << (x))

/* USBIEN */
#define USBIEN_HS               (1 << 5)
#define USBIEN_URES             (1 << 4)
#define USBIEN_SUSP             (1 << 3)
#define USBIEN_SUTOK            (1 << 2)
#define USBIEN_SOF              (1 << 1)
#define USBIEN_SUDAV            (1 << 0)

/* IVECT: */
#define UIV_SUDAV               (0x00)
#define UIV_SOF                 (0x04)
#define UIV_SUTOK               (0x08)
#define UIV_SUSPEND             (0x0c)
#define UIV_USBRESET            (0x10)
#define UIV_HSPEED              (0x14)
#define UIV_HCOUT0ERR           (0x16)
#define UIV_EP0IN               (0x18)
#define UIV_HCIN0ERR            (0x1A)
#define UIV_EP0OUT              (0x1c)
#define UIV_EP0PING             (0x20)
#define UIV_HCOUT1ERR           (0x22)
#define UIV_EP1IN               (0x24)
#define UIV_HCIN1ERR            (0x26)
#define UIV_EP1OUT              (0x28)
#define UIV_EP1PING             (0x2c)
#define UIV_HCOUT2ERR           (0x2E)
#define UIV_EP2IN               (0x30)
#define UIV_HCIN2ERR            (0x32)
#define UIV_EP2OUT              (0x34)
#define UIV_EP2PING             (0x38)
#define UIV_HCOUT3ERR           (0x3A)
#define UIV_EP3IN               (0x3C)
#define UIV_HCIN3ERR            (0x3E)
#define UIV_EP3OUT              (0x40)
#define UIV_EP3PING             (0x44)
#define UIV_HCOUT4ERR           (0x46)
#define UIV_EP4IN               (0x48)
#define UIV_HCIN4ERR            (0x4A)
#define UIV_EP4OUT              (0x4C)
#define UIV_EP4PING             (0x50)
#define UIV_HCOUT5ERR           (0x52)
#define UIV_EP5IN               (0x54)
#define UIV_HCIN5ERR            (0x56)
#define UIV_EP5OUT              (0x58)
#define UIV_EP5PING             (0x5C)
#define UIV_OTGIRQ              (0xd8)

/* ENDPRST: Endpoint Reset */
/* Endpoint Number Mask */
#define ENDPRST_EPNUM_MASK      0xf
/* Specific Endpoint Number */
#define ENDPRST_EPX(x)          ((x) & 0xf)
/* Direction: 1: IN/HCOUT; 0: OUT/HCIN */
#define ENDPRST_IO              (1 << 4)
/* Toggle Reset */
#define ENDPRST_TOGRST          (1 << 5)
/* FIFO Reset */
#define ENDPRST_FIFORST         (1 << 6)

/* USBCS */
#define USBCS_DISCONN           (1 << 6)
#define USBCS_SIGRSUME          (1 << 5)
#define USBCS_HFMODE            (1 << 1)
#define USBCS_LSMODE            (1 << 0)

/* FIFOCTRL: */
#define FIFOCTRL_EPX(x)         ((x) & 0xf)
#define FIFOCTRL_FIFOAUTO       (1 << 5)
#define FIFOCTRL_IO             (1 << 4)

/* OTGSTATE: OTG FSM State */
#define AOTG_STATE_UNDEFINED    17
/* single-role peripheral, and dual-role default-b */
#define AOTG_STATE_B_IDLE       8
#define AOTG_STATE_B_PERIPHERAL 9
#define AOTG_STATE_B_WAIT_ACON  10
#define AOTG_STATE_B_HOST       11
#define AOTG_STATE_B_SRP_INIT   12
/* extra dual-role default-b states */
/* dual-role default-a */
#define AOTG_STATE_A_IDLE       0
#define AOTG_STATE_A_WAIT_VRISE 1
#define AOTG_STATE_A_WAIT_BCON  2
#define AOTG_STATE_A_HOST       3
#define AOTG_STATE_A_SUSPEND    4
#define AOTG_STATE_A_PERIPHERAL 5
#define AOTG_STATE_A_VBUS_ERR   6
#define AOTG_STATE_A_WAIT_VFALL 7

#endif  /* __AOTG_REGS_H__ */
