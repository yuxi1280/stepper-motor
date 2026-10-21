#ifndef __CAN_H__
#define __CAN_H__

#include "hc32_ll.h"


/*
 * CAN1
 * ■ 完全支持 CAN2.0A/CAN2.0B 协议。
 * ■ CAN2.0B 向上兼容 CAN FD 协议。
 * ■ CAN2.0B 支持最高通信波特率 1Mbit/s
 *
 * CAN2
 * ■ 完全支持 CAN2.0A/CAN2.0B/CAN FD 协议。
 * ■ CAN FD 支持最高通信波特率 8Mbit/s
 *
 *
 * ■ 支持 1~1/256 的波特率预分频，灵活配置波特率。
 *
 * ■ 8 个接收缓冲器
 * 	- FIFO 方式
 * 	- 错误或者不被接收的数据不会覆盖存储的消息
 *
 * ■ 1 个高优先主发送缓冲器 PTB
 * ■ 3 个副发送缓冲器 STB
 * 	- FIFO 方式
 * 	- 优先级仲裁方式
 *
 * ■ 16 组独立的筛选器
 * 	- 支持 11 位标准 ID 和 29 位扩展 ID
 * 	- 可编程 ID CODE 位以及 MASK 位
 *
 * ■ PTB/STB 均支持单次发送模式
 * ■ 支持静默模式
 * ■ 支持回环模式
 * ■ 支持捕捉传输的错误种类以及定位仲裁失败位置
 * ■ 可编程的错误警告值
 *
 * ■ 支持 ISO11898-4 规定时间触发 CAN 以及接收时间戳
 */


// CAN_RBUF
// CAN Receive Buffer Registers
#define CAN_RBUF_CONTROL_IDE_Pos			7
#define CAN_RBUF_CONTROL_RTR_Pos			6
#define CAN_FD_RBUF_CONTROL_FDF_Pos			5
#define CAN_FD_RBUF_CONTROL_BRS_Pos			4
#define CAN_RBUF_CONTROL_DLC_Pos			0
#define CAN_RBUF_CONTROL_DLC_Msk			(0xF<<CAN_RBUF_CONTROL_DLC_Pos)

#define CAN_FD_RBUF_CONTROL_DLC_12			(0x9<<CAN_RBUF_CONTROL_DLC_Pos)
#define CAN_FD_RBUF_CONTROL_DLC_16			(0xA<<CAN_RBUF_CONTROL_DLC_Pos)
#define CAN_FD_RBUF_CONTROL_DLC_20			(0xB<<CAN_RBUF_CONTROL_DLC_Pos)
#define CAN_FD_RBUF_CONTROL_DLC_24			(0xC<<CAN_RBUF_CONTROL_DLC_Pos)
#define CAN_FD_RBUF_CONTROL_DLC_32			(0xD<<CAN_RBUF_CONTROL_DLC_Pos)
#define CAN_FD_RBUF_CONTROL_DLC_48			(0xE<<CAN_RBUF_CONTROL_DLC_Pos)
#define CAN_FD_RBUF_CONTROL_DLC_64			(0xF<<CAN_RBUF_CONTROL_DLC_Pos)


#define CAN_RBUF_STATUS_KOER_Pos			5
#define CAN_RBUF_STATUS_KOER_Msk			(0x7<<CAN_RBUF_STATUS_KOER_Pos)
#define CAN_RBUF_STATUS_TX_Pos				4

typedef struct __can_frame_t {
	union {
		uint32_t buf32_0;
		uint32_t std_id;
		uint32_t ext_id;
	};
	union {
		uint32_t buf32_1;
		struct {
			uint8_t control;
			uint8_t status;
			// CYCLE_TIME 位仅在 TTCAN 模式时有效，表示 SOF 开始时的 cycle time。
			uint16_t cycle_time;
		};
	};
	union {
		struct {
			uint32_t buf32_2;
			uint32_t buf32_3;
		};
		uint8_t data[8];
	};
} can_frame_t;


typedef struct __can_fd_frame_t {
	union {
		uint32_t buf32_0;
		uint32_t std_id;
		uint32_t ext_id;
	};
	union {
		uint32_t buf32_1;
		struct {
			uint8_t control;
			uint8_t status;
			uint16_t cycle_time;
		};
	};
	union {
		uint32_t data32[16];
		uint8_t data[64];
	};
} can_fd_frame_t;




// CAN_TBUF
// CAN Transmit Buffer Registers
#define CAN_TBUF_CONTROL_DLC_Pos			0
#define CAN_TBUF_CONTROL_DLC_Msk			(0xF<<CAN_TBUF_CONTROL_DLC_Pos)
#define CAN_TBUF_CONTROL_RTR_Pos			6
#define CAN_TBUF_CONTROL_IDE_Pos			7


// CAN_CFG_STAT
// CAN Configuration and Status Register
#define CAN_CFG_STAT_RESET_Pos				7
#define CAN_CFG_STAT_LBME_Pos				6
#define CAN_CFG_STAT_LBMI_Pos				5
#define CAN_CFG_STAT_TPSS_Pos				4
#define CAN_CFG_STAT_TSSS_Pos				3
#define CAN_CFG_STAT_RACTIVE_Pos			2
#define CAN_CFG_STAT_TACTIVE_Pos			1
#define CAN_CFG_STAT_BUSOFF_Pos				0

#define can_enable_reset(can)\
	((can)->CFG_STAT|=(1<<CAN_CFG_STAT_RESET_Pos))

#define can_enable_loopback_mode_external(can)\
	((can)->CFG_STAT|=(1<<CAN_CFG_STAT_LBME_Pos))

#define can_enable_loopback_mode_internal(can)\
	((can)->CFG_STAT|=(1<<CAN_CFG_STAT_LBMI_Pos))

#define can_enable_transmit_ptb_single_mode(can)\
	((can)->CFG_STAT|=(1<<CAN_CFG_STAT_TPSS_Pos))
#define can_enable_transmit_stb_single_mode(can)\
	((can)->CFG_STAT|=(1<<CAN_CFG_STAT_TSSS_Pos))


#define can_get_r_active(can)\
	((can)->CFG_STAT&(1<<CAN_CFG_STAT_RACTIVE_Pos))
#define can_get_t_active(can)\
	((can)->CFG_STAT&(1<<CAN_CFG_STAT_TACTIVE_Pos))

#define can_get_bus_off(can)\
	((can)->CFG_STAT&(1<<CAN_CFG_STAT_BUSOFF_Pos))

// 注意：写1可以清零TECNT和RECNT寄存器，但仅限用于调试用途。
#define can_clear_bus_off(can)\
	((can)->CFG_STAT|=(1<<CAN_CFG_STAT_BUSOFF_Pos))



// CAN_TCMD
// CAN Command Register
#define CAN_TCMD_TBSEL_Pos					7
#define CAN_TCMD_LOM_Pos					6
#define CAN_TCMD_TPE_Pos					4
#define CAN_TCMD_TPA_Pos					3
#define CAN_TCMD_TSONE_Pos					2
#define CAN_TCMD_TSALL_Pos					1
#define CAN_TCMD_TSA_Pos					0

#define CAN_TCMD_TBSEL_PTB					(0<<CAN_TCMD_TBSEL_Pos)
#define CAN_TCMD_TBSEL_STB					(1<<CAN_TCMD_TBSEL_Pos)

#define can_select_transmit_buffer(can, tb)\
	((can)->TCMD = ((can)->TCMD&~(1<<CAN_TCMD_TBSEL_Pos)) | (tb))

#define can_enable_listen_only_mode(can)\
	((can)->TCMD|=(1<<CAN_TCMD_LOM_Pos))
#define can_disable_listen_only_mode(can)\
	((can)->TCMD&=~(1<<CAN_TCMD_LOM_Pos))

#define can_enable_transmit_ptb(can)\
	((can)->TCMD|=(1<<CAN_TCMD_TPE_Pos))

#define can_get_tpe(can)\
	((can)->TCMD&(1<<CAN_TCMD_TPE_Pos))

#define can_abort_transmit_ptb(can)\
	((can)->TCMD|=(1<<CAN_TCMD_TPA_Pos))

#define can_enable_transmit_one_stb(can)\
	((can)->TCMD|=(1<<CAN_TCMD_TSONE_Pos))
#define can_enable_transmit_all_stb(can)\
	((can)->TCMD|=(1<<CAN_TCMD_TSALL_Pos))

#define can_get_tsone(can)\
	((can)->TCMD&(1<<CAN_TCMD_TSONE_Pos))
#define can_get_tsall(can)\
	((can)->TCMD&(1<<CAN_TCMD_TSALL_Pos))

#define can_abort_transmit_stb(can)\
	((can)->TCMD|=(1<<CAN_TCMD_TSA_Pos))


// CAN_TCTRL
// CAN Transmit Control Register
// TTCAN模式时此位固定为0。
#define CAN_FD_TCTRL_FD_ISO_Pos				7

#define CAN_TCTRL_TSNEXT_Pos				6
#define CAN_TCTRL_TSMODE_Pos				5
// TTEN=0时TTTBM被忽略。
#define CAN_TCTRL_TTTBM_Pos					4
#define CAN_TCTRL_TSSTAT_Pos				0
#define CAN_TCTRL_TSSTAT_Msk				(0x3<<CAN_TCTRL_TSSTAT_Pos)

// 复位模式
#define CAN_FD_TCTRL_BOSCH_CAN_FD			(0<<CAN_FD_TCTRL_FD_ISO_Pos)
#define CAN_FD_TCTRL_ISO_11898_1_CAN_FD		(1<<CAN_FD_TCTRL_FD_ISO_Pos)

#define can_transmit_stb_next(can)\
	((can)->TCTRL|=(1<<CAN_TCTRL_TSNEXT_Pos))

#define can_get_tsnext(can)\
	((can)->TCTRL&(1<<CAN_TCTRL_TSNEXT_Pos))

#define CAN_TCTRL_TSMODE_FIFO				(0<<CAN_TCTRL_TSMODE_Pos)
#define CAN_TCTRL_TSMODE_ID_PRIORITY		(1<<CAN_TCTRL_TSMODE_Pos)

#define CAN_TCTRL_TTTBM_TSMODE				(0<<CAN_TCTRL_TTTBM_Pos)
#define CAN_TCTRL_TTTBM_PTR					(1<<CAN_TCTRL_TTTBM_Pos)



// TTEN=0 || (TTEN=1 && TTTBM=0)
#define CAN_TCTRL_TSSTAT_STB_EMPTY			(0x0<<CAN_TCTRL_TSSTAT_Pos)
#define CAN_TCTRL_TSSTAT_STB_LE_HALF_FULL	(0x1<<CAN_TCTRL_TSSTAT_Pos)
#define CAN_TCTRL_TSSTAT_STB_GT_HALF_FULL	(0x2<<CAN_TCTRL_TSSTAT_Pos)
#define CAN_TCTRL_TSSTAT_STB_FULL			(0x3<<CAN_TCTRL_TSSTAT_Pos)

// TTCAN模式
// TTEN=1 && TTTBM=1
#define CAN_TCTRL_TSSTAT_PTB_STB_EMPTY		(0x0<<CAN_TCTRL_TSSTAT_Pos)
#define CAN_TCTRL_TSSTAT_PTB_STB_NOT_FULL	(0x1<<CAN_TCTRL_TSSTAT_Pos)
#define CAN_TCTRL_TSSTAT_PTB_STB_FULL		(0x3<<CAN_TCTRL_TSSTAT_Pos)

#define can_get_stb_status(can)\
	((can)->TCTRL&CAN_TCTRL_TSSTAT_Msk)

// CAN_RCTRL
// CAN Receive Control Register
#define CAN_RCTRL_SACK_Pos					7		// 外部回环
#define CAN_RCTRL_ROM_Pos					6
#define CAN_RCTRL_ROV_Pos					5
#define CAN_RCTRL_RREL_Pos					4
#define CAN_RCTRL_RBALL_Pos					3
#define CAN_RCTRL_RSTAT_Pos					0
#define CAN_RCTRL_RSTAT_Msk					(0x3<<CAN_RCTRL_RSTAT_Pos)

#define can_enable_self_acknowledge(can)\
	((can)->RCTRL|=(1<<CAN_RCTRL_SACK_Pos))
#define can_disable_self_acknowledge(can)\
	((can)->RCTRL&=~(1<<CAN_RCTRL_SACK_Pos))

#define CAN_RCTRL_ROM_FI_OVER				(0<<CAN_RCTRL_ROM_Pos)
#define CAN_RCTRL_ROM_LI_DROP				(1<<CAN_RCTRL_ROM_Pos)

#define can_release_rbuf(can)\
	((can)->RCTRL|=(1<<CAN_RCTRL_RREL_Pos))

#define can_get_rbuf_overflow(can)\
	((can)->RCTRL&(1<<CAN_RCTRL_ROV_Pos))


#define CAN_RCTRL_RSTAT_RBUF_EMPTY					0x0
#define CAN_RCTRL_RSTAT_RBUF_NOT_EMPTY_LT_AFWL		0x1
#define CAN_RCTRL_RSTAT_RBUF_NOT_FULL_GE_AFWL		0x2
#define CAN_RCTRL_RSTAT_RBUF_FULL					0x3

#define can_get_rstat(can)\
	((can)->RCTRL&CAN_RCTRL_RSTAT_Msk)


// CAN_RTIE
// CAN Receive and Transmit Interrupt Enable Register
#define CAN_RTIE_RIE_Pos					7	// 接收
#define CAN_RTIE_ROIE_Pos					6	// 接收缓冲区溢出
#define CAN_RTIE_RFIE_Pos					5	// 接收缓冲区满
#define CAN_RTIE_RAFIE_Pos					4	// 接收缓冲区将满

#define CAN_RTIE_TPIE_Pos					3	// PTB发送
#define CAN_RTIE_TSIE_Pos					2	// STB发送
#define CAN_RTIE_EIE_Pos					1	// 错误
#define CAN_RTIE_TSFF_Pos					0	// 发送缓冲区标志

#define can_enable_receive_interrupt(can)\
	((can)->RTIE|=(1<<CAN_RTIE_RIE_Pos))
#define can_enable_receive_overrun_interrupt(can)\
	((can)->RTIE|=(1<<CAN_RTIE_ROIE_Pos))
#define can_enable_receive_buffer_full_interrupt(can)\
	((can)->RTIE|=(1<<CAN_RTIE_RFIE_Pos))
#define can_enable_receive_buffer_almost_full_interrupt(can)\
	((can)->RTIE|=(1<<CAN_RTIE_RAFIE_Pos))

#define can_disable_receive_interrupt(can)\
	((can)->RTIE&=~(1<<CAN_RTIE_RIE_Pos))
#define can_disable_receive_overrun_interrupt(can)\
	((can)->RTIE&=~(1<<CAN_RTIE_ROIE_Pos))
#define can_disable_receive_buffer_full_interrupt(can)\
	((can)->RTIE&=~(1<<CAN_RTIE_RFIE_Pos))
#define can_disable_receive_buffer_almost_full_interrupt(can)\
	((can)->RTIE&=~(1<<CAN_RTIE_RAFIE_Pos))


#define can_enable_transmit_ptb_interrupt(can)\
	((can)->RTIE|=(1<<CAN_RTIE_TPIE_Pos))
#define can_enable_transmit_stb_interrupt(can)\
	((can)->RTIE|=(1<<CAN_RTIE_TSIE_Pos))

#define can_disable_transmit_ptb_interrupt(can)\
	((can)->RTIE&=~(1<<CAN_RTIE_TPIE_Pos))
#define can_disable_transmit_stb_interrupt(can)\
	((can)->RTIE&=~(1<<CAN_RTIE_TSIE_Pos))


#define can_enable_error_interrupt(can)\
	((can)->RTIE|=(1<<CAN_RTIE_EIE_Pos))

#define can_disable_error_interrupt(can)\
	((can)->RTIE&=~(1<<CAN_RTIE_EIE_Pos))

// TTEN=0 || TTTBM=0
// Secondary Transmit Buffer Full Flag
// TTEN=1 && TTTBM=1
// Transmit Buffer Full Flag
#define can_get_tsff(can)\
	((can)->RTIE&(1<<CAN_RTIE_TSFF_Pos))

// CAN_RTIF
// CAN Receive and Transmit Interrupt Status Register
#define CAN_RTIF_RIF_Pos					7
#define CAN_RTIF_ROIF_Pos					6
#define CAN_RTIF_RFIF_Pos					5
#define CAN_RTIF_RAFIF_Pos					4

#define CAN_RTIF_TPIF_Pos					3	// PTB发送完成
// TTCAN模式时，TPIF无效，仅使用TSIF标志。
#define CAN_RTIF_TSIF_Pos					2	// STB发送完成
#define CAN_RTIF_EIF_Pos					1
#define CAN_RTIF_AIF_Pos					0	// 发送被成功取消

#define can_get_rif(can)\
	((can)->RTIF&(1<<CAN_RTIF_RIF_Pos))
#define can_get_roif(can)\
	((can)->RTIF&(1<<CAN_RTIF_ROIF_Pos))
#define can_get_rfif(can)\
	((can)->RTIF&(1<<CAN_RTIF_RFIF_Pos))
#define can_get_rafif(can)\
	((can)->RTIF&(1<<CAN_RTIF_RAFIF_Pos))

#define can_get_tpif(can)\
	((can)->RTIF&(1<<CAN_RTIF_TPIF_Pos))
#define can_get_tsif(can)\
	((can)->RTIF&(1<<CAN_RTIF_TSIF_Pos))
#define can_get_eif(can)\
	((can)->RTIF&(1<<CAN_RTIF_EIF_Pos))
#define can_get_aif(can)\
	((can)->RTIF&(1<<CAN_RTIF_AIF_Pos))

#define can_clear_rif(can)\
	((can)->RTIF|=(1<<CAN_RTIF_RIF_Pos))
#define can_clear_roif(can)\
	((can)->RTIF|=(1<<CAN_RTIF_ROIF_Pos))
#define can_clear_rfif(can)\
	((can)->RTIF|=(1<<CAN_RTIF_RFIF_Pos))
#define can_clear_rafif(can)\
	((can)->RTIF|=(1<<CAN_RTIF_RAFIF_Pos))

#define can_clear_tpif(can)\
	((can)->RTIF|=(1<<CAN_RTIF_TPIF_Pos))
#define can_clear_tsif(can)\
	((can)->RTIF|=(1<<CAN_RTIF_TSIF_Pos))

#define can_clear_eif(can)\
	((can)->RTIF|=(1<<CAN_RTIF_EIF_Pos))
#define can_clear_aif(can)\
	((can)->RTIF|=(1<<CAN_RTIF_AIF_Pos))



/*
 * CAN 节点有以下三种错误状态：
 * ■ 错误主动：节点检测到错误时自动发送主动错误标志。
 * ■ 错误被动：节点检测到错误时自动发送被动错误标志。
 * ■ 节点关闭：关闭状态下此节点不再影响整个 CAN 网络。
 */
/*
 * ISO 11898-1 (125Kbps~1Mbps)
 * <---- 5V
 *
 *
 *               SOF|<--------------------ID------------------->|RTR|
 *              |---|---|---|---|---|---|---|   |---|   |---|---|---|---
 * CAN HI       |                           |   |   |   |
 * -|---|---|---|<--2.5V                    |---|   |---|
 *    1   1   1   0   0   0   0   0   0   0   1   0   1   0   0   0
 * -|---|---|---|<--2.5V                    |---|   |---|
 * CAN LO       |                           |   |   |   |
 *              |---|---|---|---|---|---|---|   |---|   |---|---|---|---
 *                    ↑                       ↑
 *                 dominant               recessive
 *                   显性                     隐性
 * <---- 0V
 */
/*
 *
 *    reset
 *     |
 *     ↓           when TECNT or RECNT >= EWL
 * ERROR ACTIVE --------------------------------> ERROR PASSIVE
 *              <--------------------------------
 *                 when TECNT and RECNT < EWL
 *         ↖                                        /
 *          \                                      /
 *           \                                    /
 *            \                                  /
 *  when       \                                /
 *  128*11      \                              /
 *  recessive    \                            /  when TECNT > 255
 *  bits          \                          /
 *  occur          \                        /
 *                  \                      /
 *                   \                    /
 *                    \                  /
 *                     \                ↙
 *                           BUS OFF
 */
// 125kbps
// 128*11*125ns=176us=0.176ms

// CAN_ERRINT
// CAN ERRor INTerrupt Enable and Flag Register
#define CAN_ERRINT_EWARN_Pos				7
#define CAN_ERRINT_EPASS_Pos				6
#define CAN_ERRINT_EPIE_Pos					5
#define CAN_ERRINT_EPIF_Pos					4
#define CAN_ERRINT_ALIE_Pos					3
#define CAN_ERRINT_ALIF_Pos					2
#define CAN_ERRINT_BEIE_Pos					1
#define CAN_ERRINT_BEIF_Pos					0

#define can_get_error_warning(can)\
	((can)->ERRINT&(1<<CAN_ERRINT_EWARN_Pos))

#define can_get_error_passive(can)\
	((can)->ERRINT&(1<<CAN_ERRINT_EPASS_Pos))


#define can_enable_error_passive_interrupt(can)\
	((can)->ERRINT|=(1<<CAN_ERRINT_EPIE_Pos))
#define can_disable_error_passive_interrupt(can)\
	((can)->ERRINT&=~(1<<CAN_ERRINT_EPIE_Pos))

// 发生错误主动到错误被动或者错误被动到错误主动的变化
#define can_get_epif(can)\
	((can)->ERRINT&(1<<CAN_ERRINT_EPIF_Pos))
#define can_clear_epif(can)\
	((can)->ERRINT|=(1<<CAN_ERRINT_EPIF_Pos))

#define can_enable_arbitration_lost_interrupt(can)\
	((can)->ERRINT|=(1<<CAN_ERRINT_ALIE_Pos))
#define can_disable_arbitration_lost_interrupt(can)\
	((can)->ERRINT&=~(1<<CAN_ERRINT_ALIE_Pos))

#define can_get_alif(can)\
	((can)->ERRINT&(1<<CAN_ERRINT_ALIF_Pos))
#define can_clear_alif(can)\
	((can)->ERRINT|=(1<<CAN_ERRINT_ALIF_Pos))

#define can_enable_bus_error_interrupt(can)\
	((can)->ERRINT|=(1<<CAN_ERRINT_BEIE_Pos))
#define can_disable_bus_error_interrupt(can)\
	((can)->ERRINT&=~(1<<CAN_ERRINT_BEIE_Pos))

#define can_get_beif(can)\
	((can)->ERRINT&(1<<CAN_ERRINT_BEIF_Pos))
#define can_clear_beif(can)\
	((can)->ERRINT|=(1<<CAN_ERRINT_BEIF_Pos))





// CAN_SBT
// CAN Slow Bit Timing Register
// PRESC
// SEG_1:0~63
// SEG_2:0~7
// SJW:0~15
// SEG_1>=SEG2+1,SEG_2>=SJW
#define CAN_SBT_PRESC_Pos					24
#define CAN_SBT_PRESC_Msk					(0xFF<<CAN_SBT_PRESC_Pos)
#define CAN_SBT_SJW_Pos						16
#define CAN_SBT_SJW_Msk						(0x7F<<CAN_SBT_SJW_Pos)
#define CAN_SBT_SEG_2_Pos					8
#define CAN_SBT_SEG_2_Msk					(0x7F<<CAN_SBT_SEG_2_Pos)
#define CAN_SBT_SEG_1_Pos					0
#define CAN_SBT_SEG_1_Msk					(0xFF<<CAN_SBT_SEG_1_Pos)

// SBT 寄存器用于 CAN2.0 和 CAN FD 的仲裁段
//
// f{can_clk}=240,000,000/6=40,000,000
// 1tq=(PRESC+1)/f{can_clk}
// baudrate
// =1/(((SJW_1+2)+(SJW_2+1))*((PRESC+1)/f{can_clk}))
// =1/(((22   +2)+(7    +1))*((9    +1)/40000000))
// =125000bps
#define can_set_slow_bit_timing(can, presc, seg_1, seg_2, sjw)\
	(can)->SBT = (((presc)-1)<<CAN_SBT_PRESC_Pos) | (((sjw)-1)<<CAN_SBT_SJW_Pos) | (((seg_2)-1)<<CAN_SBT_SEG_2_Pos) | (((seg_1)-2)<<CAN_SBT_SEG_1_Pos)


// CAN_FBT
// CAN Fast Bit Timing Register
#define CAN_FD_FBT_PRESC_Pos					24
#define CAN_FD_FBT_PRESC_Msk					(0xFF<<CAN_FD_FBT_PRESC_Pos)
#define CAN_FD_FBT_SJW_Pos						16
#define CAN_FD_FBT_SJW_Msk						(0xF<<CAN_FD_FBT_PRESC_Pos)
#define CAN_FD_FBT_SEG_2_Pos					8
#define CAN_FD_FBT_SEG_2_Msk					(0xF<<CAN_FD_FBT_PRESC_Pos)
#define CAN_FD_FBT_SEG_1_Pos					0
#define CAN_FD_FBT_SEG_1_Msk					(0x1F<<CAN_FD_FBT_PRESC_Pos)

// FBT 寄存器用于 CAN FD 数据段
#define can_set_fast_bit_timing(can, presc, seg_1, seg_2, sjw)\
	(can)->SBT = (((presc)-1)<<CAN_FD_FBT_PRESC_Pos) | (((sjw)-1)<<CAN_FD_FBT_SJW_Pos) | (((seg_2)-1)<<CAN_FD_FBT_SEG_2_Pos) | (((seg_1)-2)<<CAN_FD_FBT_SEG_1_Pos)


// CAN_TDC
// CAN Transmitter Delay Compensatin Register
#define CAN_FD_TDC_TDCEN_Pos					7
#define CAN_FD_TDC_SSPOFF_Pos					0
#define CAN_FD_TDC_SSPOFF_Msk					(0x7F<<CAN_FD_TDC_SSPOFF_Pos)


// CAN_EALCAP
// CAN Error and Arbitration Lost Capture Register
//#define CAN_RBUF_STATUS_KOER_Pos			5
#define CAN_EALCAP_KOER_Pos					5
#define CAN_EALCAP_KOER_Msk					(0x7<<CAN_EALCAP_KOER_Pos)

#define CAN_EALCAP_ALC_Pos					0
#define CAN_EALCAP_ALC_Msk					(0x1F<<CAN_EALCAP_ALC_Pos)

#define CAN_EOER_NO_ERROR					0x00//(0x0<<CAN_EALCAP_KOER_Pos)
// 位错误
#define CAN_EOER_BIT_ERROR					0x20//(0x1<<CAN_EALCAP_KOER_Pos)
// 形式错误
#define CAN_EOER_FORM_ERROR					0x40//(0x2<<CAN_EALCAP_KOER_Pos)
// 填充错误
#define CAN_EOER_STUFF_ERROR				0x60//(0x3<<CAN_EALCAP_KOER_Pos)
// 应答错误
#define CAN_EOER_ACK_ERROR					0x80//(0x4<<CAN_EALCAP_KOER_Pos)
// CRC错误
#define CAN_EOER_CRC_ERROR					0xA0//(0x5<<CAN_EALCAP_KOER_Pos)
#define CAN_EOER_OTHER_ERROR				0xC0//(0x6<<CAN_EALCAP_KOER_Pos)

// 错误种类
#define can_get_kind_of_error(can)\
	((can)->EALCAP&CAN_EALCAP_KOER_Msk)
extern const char *can_get_kind_of_error_string(CM_CAN_TypeDef *can);

// 仲裁失败位位置
// 0~31
#define can_get_arbitration_lost_capture(can)\
	((can)->EALCAP&CAN_EALCAP_ALC_Msk)



// CAN_LIMIT
// CAN Warning Limits Register
// 1~8
#define CAN_LIMIT_AFWL_Pos					4
#define CAN_LIMIT_AFWL_Msk					(0xF<<CAN_LIMIT_AFWL_Pos)

// 128
#define CAN_LIMIT_EWL_Pos					0
#define CAN_LIMIT_EWL_Msk					(0xF<<CAN_LIMIT_EWL_Pos)

// 主动错误状态
// tec:0~127 && rec:0~127
// 被动错误状态
// tec:128~255 || rec:128~255
// 总线关闭状态-->(在总线上检测到128次连续的11个位的隐性位)-->主动错误状态
// tec:256~,rec:-

// CAN_RECNT
// CAN Receive Error CouNT Register
#define can_get_receive_error_count(can)\
	((can)->RECNT)
// CAN_TECNT
// CAN Transmit Error CouNT Register
#define can_get_transmit_error_count(can)\
	((can)->TECNT)

// CAN_ACFCTRL
// CAN Acceptance Filter Control Register
#define CAN_ACFCTRL_SELMASK_Pos				5
#define CAN_ACFCTRL_ACFADR_Pos				0
#define CAN_ACFCTRL_ACFADR_Msk				(0xF<<CAN_ACFCTRL_ACFADR_Pos)

#define CAN_ACFCTRL_ACFADR_ACF0				0x0
#define CAN_ACFCTRL_ACFADR_ACF1				0x1
#define CAN_ACFCTRL_ACFADR_ACF2				0x2
#define CAN_ACFCTRL_ACFADR_ACF3				0x3
#define CAN_ACFCTRL_ACFADR_ACF4				0x4
#define CAN_ACFCTRL_ACFADR_ACF5				0x5
#define CAN_ACFCTRL_ACFADR_ACF6				0x6
#define CAN_ACFCTRL_ACFADR_ACF7				0x7
#define CAN_ACFCTRL_ACFADR_ACF8				0x8
#define CAN_ACFCTRL_ACFADR_ACF9				0x9
#define CAN_ACFCTRL_ACFADR_ACF10			0xA
#define CAN_ACFCTRL_ACFADR_ACF11			0xB
#define CAN_ACFCTRL_ACFADR_ACF12			0xC
#define CAN_ACFCTRL_ACFADR_ACF13			0xD
#define CAN_ACFCTRL_ACFADR_ACF14			0xE
#define CAN_ACFCTRL_ACFADR_ACF15			0xF



// CAN_ACFEN
// CAN Acceptance Filter Enable Register
#define can_enable_acf(can, i)\
	((can)->ACFEN|=(1<<(i)))
#define can_disable_acf(can, i)\
	((can)->ACFEN&=~(1<<(i)))

// CAN_ACF
// CAN Acceptance Filter code and mask Register
#define CAN_ACF_AIDEE_Pos					30
#define CAN_ACF_AIDE_Pos					29

#define CAN_ACF_AIDE_Msk					(0x3<<CAN_ACF_AIDE_Pos)

#define CAN_ACF_FT_BOTH						0x0
#define CAN_ACF_FT_STD						0x2
#define CAN_ACF_FT_EXT						0x3


#define CAN_ACF_ACODE_Pos					0
#define CAN_ACF_ACODE_Msk					(0x1FFFFFFF)
#define CAN_ACF_AMASK_Pos					0
#define CAN_ACF_AMASK_Msk					(0x1FFFFFFF)








// Time Trigger Controller Area Network
// CAN-CTRL 为 ISO11898-4 规定的时间触发通信方式提供部分(lever 1)硬件支持。
//
// CAN_TBSLOT
// TTCAN TB Slot Pointer Register
#define CAN_TBSLOT_TBE_Pos					7
#define CAN_TBSLOT_TBF_Pos					6
#define CAN_TBSLOT_TBPTR_Pos				0
#define CAN_TBSLOT_TBPTR_Msk				(0x7<<CAN_TBSLOT_TBPTR_Pos)

#define CAN_TBSLOT_TBPTR_PTB				(0x0<<CAN_TBSLOT_TBPTR_Pos)
#define CAN_TBSLOT_TBPTR_STB_SLOT1			(0x1<<CAN_TBSLOT_TBPTR_Pos)
#define CAN_TBSLOT_TBPTR_STB_SLOT2			(0x2<<CAN_TBSLOT_TBPTR_Pos)
#define CAN_TBSLOT_TBPTR_STB_SLOT3			(0x3<<CAN_TBSLOT_TBPTR_Pos)

#define can_set_transmit_buffer_slot_empty(can)\
	((can)->TBSLOT|=(1<<CAN_TBSLOT_TBE_Pos))

#define can_set_transmit_buffer_slot_filled(can)\
	((can)->TBSLOT|=(1<<CAN_TBSLOT_TBF_Pos))

#define can_set_transmit_buffer_slot_pointer(can, p)\
	((can)->TBSLOT=((can)->TBSLOT&~(CAN_TBSLOT_TBPTR_Msk)) | (p))


// CAN_TTCFG
// TTCAN TB Slot Pointer Register
#define CAN_TTCFG_WTIE_Pos					7
#define CAN_TTCFG_WTIF_Pos					6
#define CAN_TTCFG_TEIF_Pos					5
#define CAN_TTCFG_TTIE_Pos					4
#define CAN_TTCFG_TTIF_Pos					3
#define CAN_TTCFG_T_PRESC_Pos				1
#define CAN_TTCFG_T_PRESC_Msk				(0x3<<CAN_TTCFG_T_PRESC_Pos)
#define CAN_TTCFG_TTEN_Pos					0


// CAN_REF_MSG
// TTCAN Reference Message Register
#define CAN_REF_MSG_REF_IDE_Pos				31
#define CAN_REF_MSG_REF_ID_Pos				0
#define CAN_REF_MSG_REF_ID_Msk				(0x1FFFFFFF)


// CAN_TRG_CFG
// TTCAN Reference Message Register
#define CAN_TRG_CFG_TEW_Pos					12
#define CAN_TRG_TTYPE_Pos					8
#define CAN_TRG_TTPTR_Pos					0


// 需先写低字节再写高字节
// CAN_TT_TRIG
// TTCAN Reference Message Register
#define can_set_tt_trig(can, t)\
	((can)->TT_TRIG = (t))

// 需先写低字节再写高字节
// CAN_TT_WTRIG
// TTCAN Watch Trigger Time Register
#define can_set_tt_wtrig(can, t)\
	((can)->TT_WTRIG = (t))





extern void can_disable_reset(CM_CAN_TypeDef *can);
extern void can_set_acf(CM_CAN_TypeDef *can, uint32_t acf, uint32_t ft, uint32_t code, uint32_t mask);
extern void can_push_tx_frame(CM_CAN_TypeDef *can, uint32_t tb, can_frame_t *f);
extern void can_get_rx_frame(CM_CAN_TypeDef *can, can_frame_t *f);

extern void can_fd_push_tx_frame(CM_CAN_TypeDef *can, uint32_t tb, can_fd_frame_t *f);
extern void can_fd_get_rx_frame(CM_CAN_TypeDef *can, can_fd_frame_t *f);


#endif	// __CAN_H__
