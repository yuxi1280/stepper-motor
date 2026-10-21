#include "can_context.h"
#include "board.h"


void can_open(can_context_t *ctx)
{
	log_printf("[%d]can_open\n", can_get_channel(ctx));
	CM_CAN_TypeDef *can = ctx->_can;
	can_frame_buffer_init(ctx->_tx_fb, CAN_TX_RING_BUFFER_SIZE);
	can_frame_buffer_init(ctx->_rx_fb, CAN_RX_RING_BUFFER_SIZE);

	LL_PERIPH_WE(LL_PERIPH_FCG);

	if (can == CM_CAN1) {
		state3_led_on();
		FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_CAN1, ENABLE);
	} else if (can == CM_CAN2) {
		state4_led_on();
		FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_CAN2, ENABLE);
	} else {

	}

	LL_PERIPH_WP(LL_PERIPH_FCG);

	ctx->_is_opened = true;
	ctx->_error_count = 0;
}

void can_close(can_context_t *ctx)
{
	log_printf("[%d]can_close\n", can_get_channel(ctx));
	CM_CAN_TypeDef *can = ctx->_can;

	LL_PERIPH_WE(LL_PERIPH_FCG);

	// 禁止所有中断
	can->RTIE = 0x00;
	can->RTIF = 0x00;
	can->ERRINT = 0x00;

	if (can == CM_CAN1) {
		state3_led_off();
		NVIC_DisableIRQ(CAN1_IRQn);
		FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_CAN1, DISABLE);
	} else if (can == CM_CAN2) {
		state4_led_off();
		NVIC_DisableIRQ(CAN2_IRQn);
		FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_CAN2, DISABLE);
	} else {
		return;
	}

	LL_PERIPH_WP(LL_PERIPH_FCG);

	ctx->_is_opened = false;
}

void can_config(can_context_t *ctx, can_config_t *cfg)
{
	log_printf("[%d]can_config:%p\n", can_get_channel(ctx), cfg);

	CM_CAN_TypeDef *can = ctx->_can;


	// 复位模式
	can_enable_reset(can);

	// 设置CAN FD模式
	//can->TCTRL = (can->TCTRL&~(1<<CAN_FD_TCTRL_FD_ISO_Pos)) | CAN_FD_TCTRL_ISO_11898_1_CAN_FD;

	// 设置波特率
	can_set_slow_bit_timing(can, cfg->_presc, cfg->_seg_1, cfg->_seg_2, cfg->_sjw);

	// 设置筛选器
	can_filter_t *filter;
	for (uint32_t i = 0; i < cfg->_filter_size; ++i) {
		filter = cfg->_filters+i;
//		log_printf("acf:%d,ft:%02x,code:%08lx,mask:%08lx\n",
//				filter->_acf, filter->_ft, filter->_code, filter->_mask);

		can_set_acf(can, filter->_acf, filter->_ft, filter->_code, filter->_mask);
		can_enable_acf(can, filter->_acf);
	}

	can_disable_reset(can);


	// 动作模式
	can->CFG_STAT =
		  (0<<CAN_CFG_STAT_RESET_Pos)
		| (0<<CAN_CFG_STAT_LBME_Pos)		// 外部回环模式
		| (0<<CAN_CFG_STAT_LBMI_Pos)		// 内部回环模式
		| (0<<CAN_CFG_STAT_TPSS_Pos)		// PTB单次传输模式
		| (0<<CAN_CFG_STAT_TSSS_Pos)		// STB单次传输模式
		;

	can->TCMD =
		  CAN_TCMD_TBSEL_PTB
		| (0<<CAN_TCMD_LOM_Pos)
		| (0<<CAN_TCMD_TPE_Pos)
		| (0<<CAN_TCMD_TPA_Pos)
		| (0<<CAN_TCMD_TSONE_Pos)
		| (0<<CAN_TCMD_TSALL_Pos)
		| (0<<CAN_TCMD_TSA_Pos)
		;

	can->TCTRL =
		 CAN_FD_TCTRL_ISO_11898_1_CAN_FD
		| (0<<CAN_TCTRL_TSNEXT_Pos)
		| CAN_TCTRL_TSMODE_FIFO
		| CAN_TCTRL_TTTBM_TSMODE
		;

	can->RCTRL =
		  (0<<CAN_RCTRL_SACK_Pos)
		| CAN_RCTRL_ROM_FI_OVER
		| (0<<CAN_RCTRL_RREL_Pos)
		| (0<<CAN_RCTRL_RBALL_Pos)
		| (1<<2)
		;

	can->LIMIT =
		  (cfg->_almost_full_warning_limit<<CAN_LIMIT_AFWL_Pos)
		| ((((cfg->_error_warning_limit)>>3)-1)<<CAN_LIMIT_EWL_Pos)
		;

	can->RTIE = 0x00;
	can->ERRINT = 0x00;

	can_enable_receive_interrupt(can);
//	can_enable_receive_overrun_interrput(can);
//	can_enable_receive_buffer_full_interrupt(can);
//	can_enable_receive_buffer_almost_full_interrupt(can);

	can_enable_transmit_ptb_interrupt(can);
//	can_enable_transmit_stb_interrupt(can);

	can_enable_error_interrupt(can);

	can_enable_error_passive_interrupt(can);
	can_enable_arbitration_lost_interrupt(can);
	can_enable_bus_error_interrupt(can);


	if (can == CM_CAN1) {
		NVIC_EnableIRQ(CAN1_IRQn);
	} else if (can == CM_CAN2) {
		NVIC_EnableIRQ(CAN2_IRQn);
	}
}



void can_poll(can_context_t *ctx)
{
	can_frame_t f;

	if (!ctx->_is_opened) {
		return;
	}

	if (can_frame_buffer_not_empty(ctx->_rx_fb)) {
		while (can_frame_buffer_not_empty(ctx->_rx_fb)) {
			can_frame_buffer_get(ctx->_rx_fb, &f);

			can_on_recv(ctx, &f);
		}
	}
}

static bool can_is_invalid_id(uint8_t ide, uint32_t id)
{
	if (ide) {
		if ((id&0xE0000000) != 0 || (id&0x1FC00000) == 0x1FC00000) {
			return true;
		}
	} else {
		if ((id&0xFFFFF800) != 0 || (id&0x7F0) == 0x7F0) {
			return true;
		}
	}
	return false;
}

bool can_tx_remote_frame(can_context_t *ctx, uint8_t ide, uint32_t id)
{
	if (can_is_invalid_id(ide, id)) {
		return false;
	}

	can_frame_t tx_frame;
	memset(&tx_frame, 0, sizeof(tx_frame));
	if (ide) {
		tx_frame.ext_id = id;
	} else {
		tx_frame.std_id = id;
	}

	tx_frame.control =
		  (ide<<CAN_TBUF_CONTROL_IDE_Pos)
		| (1<<CAN_TBUF_CONTROL_RTR_Pos)
		| (0<<CAN_TBUF_CONTROL_DLC_Pos);

	// 无锁版本
	if (can_frame_buffer_not_full(ctx->_tx_fb)) {
		can_frame_buffer_put(ctx->_tx_fb, &tx_frame);

		if (ctx->_transmit_ptb_ready) {
			ctx->_transmit_ptb_ready = false;

			can_push_tx_frame(ctx->_can, CAN_TCMD_TBSEL_PTB, &tx_frame);
			can_enable_transmit_ptb(ctx->_can);
		}

		return true;
	} else {
		return false;
	}
}

#if	1
void can_tx_frame(can_context_t *ctx, can_frame_t *f)
{
	can_frame_buffer_put(ctx->_tx_fb, f);

	if (ctx->_transmit_ptb_ready) {
		ctx->_transmit_ptb_ready = false;

		can_push_tx_frame(ctx->_can, CAN_TCMD_TBSEL_PTB, f);
		can_enable_transmit_ptb(ctx->_can);
	}
}
#endif


bool can_tx_data_frames(can_context_t *ctx, uint8_t ide, uint32_t id, const uint8_t *p, int n)
{
#if	0
	log_printf("[%d][%u,%08lx]", can_get_channel(ctx), ide, id);
	for (int i = 0; i < n; ++i) {
		log_printf("%02x ", p[i]);
	}
	log_printf("->\n");
#endif
	if (can_is_invalid_id(ide, id)) {
		log_printf("[%d]can_is_invalid_id\n", can_get_channel(ctx));
		return false;
	}
	//log_printf("%d,%d\n", ctx->_tx_fb.head, ctx->_tx_fb.tail);

	// 缓冲区不够此次发送
	if (n > can_frame_get_free_size(&ctx->_tx_fb)*8) {
		log_printf("[%d]can_frame_get_free_size failed\n", can_get_channel(ctx));
		return false;
	}


	can_frame_t tx_frame;
	memset(&tx_frame, 0, sizeof(tx_frame));
	if (ide) {
		tx_frame.ext_id = id;
	} else {
		tx_frame.std_id = id;
	}

	uint8_t delta = 0;

	for (int i = 0; i < n; i += 8) {
		delta = n-i > 8 ? 8 : n-i;
		tx_frame.control =
			  (ide<<CAN_TBUF_CONTROL_IDE_Pos)
			| (0<<CAN_TBUF_CONTROL_RTR_Pos)
			| (delta<<CAN_TBUF_CONTROL_DLC_Pos);

		for (int j = 0; j < delta; ++j) {
			tx_frame.data[j] = p[i+j];
		}

		// 无锁版本
		if (can_frame_buffer_not_full(ctx->_tx_fb)) {
			can_frame_buffer_put(ctx->_tx_fb, &tx_frame);

			if (ctx->_transmit_ptb_ready) {
				ctx->_transmit_ptb_ready = false;

				can_push_tx_frame(ctx->_can, CAN_TCMD_TBSEL_PTB, &tx_frame);
				can_enable_transmit_ptb(ctx->_can);
			}
		} else {
			log_printf("tx frame buffer full\n");
		}
	}

	return true;
}

// 注意：120Ω电阻必须要接上！
void can_irq_callback(can_context_t *ctx)
{
	can_frame_t f;

	CM_CAN_TypeDef *can = ctx->_can;

	// 接收到有效的数据帧或远程帧
	if (can_get_rif(can)) {
		can_clear_rif(can);

		while (can_get_rstat(can) != CAN_RCTRL_RSTAT_RBUF_EMPTY) {
			can_get_rx_frame(can, &f);

			// 放入队列
			if (can_frame_buffer_not_full(ctx->_rx_fb)) {
				can_frame_buffer_put(ctx->_rx_fb, &f);
			}
		}
	}
	// 接收上溢
	if (can_get_roif(can)) {
		can_clear_roif(can);
//		log_printf("[%d]roif\n", can_get_channel(ctx));

		can_on_error(ctx, CAN_ERROR_RO);
	}

#if	0
	// 接收缓冲区满
	if (can_get_rfif(can)) {
		can_clear_rfif(can);

	}
	// 接收缓冲区被填充大于等于_almost_full_warning_limit
	if (can_get_rafif(can)) {
		can_clear_rafif(can);

	}
#endif


	// PTB发送完成
	if (can_get_tpif(can)) {
		can_clear_tpif(can);

#if	0
		log_printf("tpif:beif:%u,alif:%u,bus_off:%u,error_passive:%u,koe:%02x,tecnt:%u,recnt:%u\n",
				can_get_beif(can),
				can_get_alif(can),
				can_get_bus_off(can),
				can_get_error_passive(can),
				can_get_kind_of_error(can),
				can_get_transmit_error_count(can), can_get_receive_error_count(can));
#endif

		// 无锁版本
		// 去除已经发送完成的包
		can_frame_buffer_remove_head(ctx->_tx_fb);

		if (can_frame_buffer_not_empty(ctx->_tx_fb)) {
			//log_printf(".\n");
			//ctx->_transmit_ptb_ready = false;

			can_frame_t f;
			can_frame_buffer_head(ctx->_tx_fb, &f);

			can_push_tx_frame(can, CAN_TCMD_TBSEL_PTB, &f);
			can_enable_transmit_ptb(can);
		} else {
			//log_printf("$\n");
			ctx->_transmit_ptb_ready = true;
		}
	}

#if	0
	// STB发送完成
	if (can_get_tsif(can)) {
		can_clear_tsif(can);


	}
#endif


	/*
	 * BUSOFF位发生变化，
	 * 或者错误计数器的值与ERROR warning limit设定值的相对关系发生变化。
	 * 比如错误计数器的值从小于设定值变为大于设定值，
	 * 或者从大于设定值变为小于设定值。
	 */
	if (can_get_eif(can)) {
		can_clear_eif(can);
//		log_printf("[%d]eif:bus_off:%u\n", can_get_channel(ctx), can_get_bus_off(can));

		can_on_error(ctx, CAN_ERROR_E);
	}

	// 发生错误主动到错误被动或者错误被动到错误主动的变化
	if (can_get_epif(can)) {
		can_clear_epif(can);
#if	0
		log_printf("[%d]epif:error_passive:%u,tecnt:%u,recnt:%u\n", can_get_channel(ctx), can_get_error_passive(can),
				can_get_transmit_error_count(can), can_get_receive_error_count(can));
#endif

		can_on_error(ctx, CAN_ERROR_EP);
	}

	// 仲裁失败
	if (can_get_alif(can)) {
		can_clear_alif(can);
//		log_printf("[%d]alc:%u\n", can_get_channel(ctx), can_get_arbitration_lost_capture(can));

		can_on_error(ctx, CAN_ERROR_AL);
	}

	// 总线错误
	if (can_get_beif(can)) {
		can_clear_beif(can);
#if	0
		log_printf("[%d]beif:bus_off:%u,error_passive:%u,koe:%s[%02x],tecnt:%u,recnt:%u\n",
				can_get_channel(ctx),
				can_get_bus_off(can),
				can_get_error_passive(can),
				can_get_kind_of_error_string(can),
				can_get_kind_of_error(can),
				can_get_transmit_error_count(can), can_get_receive_error_count(can));
#endif

		can_on_error(ctx, CAN_ERROR_BE);
	}
}
