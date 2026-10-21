#include "can.h"
#include "board.h"


void can_push_tx_frame(CM_CAN_TypeDef *can, uint32_t tb, can_frame_t *f)
{
	if (can == NULL || f == NULL) {
		log_printf("can_push_tx_frame failed:%d\n", __LINE__);
		return;
	}
	if (tb == CAN_TCMD_TBSEL_STB
			&& can_get_stb_status(can) == CAN_TCTRL_TSSTAT_STB_FULL) {
		log_printf("can_push_tx_frame failed:%d\n", __LINE__);
		return;
	}

	// 选择PTB或者STB
	can_select_transmit_buffer(can, tb);

	uint32_t a = (uint32_t)&can->TBUF;
	*(volatile uint32_t *)(a+ 0) = f->buf32_0;
	*(volatile uint32_t *)(a+ 4) = f->buf32_1;
	*(volatile uint32_t *)(a+ 8) = f->buf32_2;
	*(volatile uint32_t *)(a+12) = f->buf32_3;

	if (tb == CAN_TCMD_TBSEL_STB) {
		can_transmit_stb_next(can);
	} else {
		//log_printf("can_push_tx_frame failed:%d\n", __LINE__);
	}
}

void can_get_rx_frame(CM_CAN_TypeDef *can, can_frame_t *f)
{
	if (can == NULL || f == NULL) {
		return;
	}

	uint32_t a = (uint32_t)&can->RBUF;

	f->buf32_0 = *(volatile uint32_t *)(a+ 0);
	f->buf32_1 = *(volatile uint32_t *)(a+ 4);
	f->buf32_2 = *(volatile uint32_t *)(a+ 8);
	f->buf32_3 = *(volatile uint32_t *)(a+12);

	can_release_rbuf(can);
}


void can_fd_push_tx_frame(CM_CAN_TypeDef *can, uint32_t tb, can_fd_frame_t *f)
{
	if (can == NULL || f == NULL) {
		return;
	}
	if (tb == CAN_TCMD_TBSEL_STB
			&& can_get_stb_status(can) == CAN_TCTRL_TSSTAT_STB_FULL) {
		return;
	}

	// 选择PTB或者STB
	can_select_transmit_buffer(can, tb);

	uint32_t a = (uint32_t)&can->TBUF;
	*(volatile uint32_t *)(a+ 0) = f->buf32_0;
	*(volatile uint32_t *)(a+ 4) = f->buf32_1;

	for (int i = 0; i < 16; ++i) {
		*(volatile uint32_t *)(a+8+i*4) = f->data32[i];
	}

	if (tb == CAN_TCMD_TBSEL_STB) {
		can_transmit_stb_next(can);
	}
}

void can_fd_get_rx_frame(CM_CAN_TypeDef *can, can_fd_frame_t *f)
{
	if (can == NULL || f == NULL) {
		return;
	}

	uint32_t a = (uint32_t)&can->RBUF;

	f->buf32_0 = *(volatile uint32_t *)(a+ 0);
	f->buf32_1 = *(volatile uint32_t *)(a+ 4);

	for (int i = 0; i < 16; ++i) {
		f->data32[i] = *(volatile uint32_t *)(a+8+i*4);
	}

	can_release_rbuf(can);
}


void can_disable_reset(CM_CAN_TypeDef *can)
{
	do {
		can->CFG_STAT &= ~(1<<CAN_CFG_STAT_RESET_Pos);
	} while (can->CFG_STAT&(1<<CAN_CFG_STAT_RESET_Pos));
}


/*
 * 每组筛选器有一个 32 位 ID CODE 寄存器和一个 32 位 ID MASK 寄存器，
 * ID CODE 寄存器用于比较接收到 CAN ID，而 ID MASK 寄存器用于选择比较的CAN ID 位。
 * 对应的 ID MASK 位为 1 时，不比较该位的 ID CODE。
 * 接收到的数据只要通过 16 组筛选器的任意一组，则被接收，
 * 接收到的数据存储在 RB 中，否则数据不被接收，也不被存储。
*/
void can_set_acf(CM_CAN_TypeDef *can, uint32_t acf, uint32_t ft, uint32_t code, uint32_t mask)
{
	can->ACFCTRL = (0<<CAN_ACFCTRL_SELMASK_Pos) | (acf);
	can->ACF = ((ft)<<CAN_ACF_AIDE_Pos) | ((code)&CAN_ACF_ACODE_Msk);

	can->ACFCTRL = (1<<CAN_ACFCTRL_SELMASK_Pos) | (acf);
	can->ACF = ((ft)<<CAN_ACF_AIDE_Pos) | ((mask)&CAN_ACF_AMASK_Msk);
}



static const char *s_koe_string[] = {
	"no error",

	"bit error",
	"form error",
	"staff error",
	"ack error",
	"crc error",

	"other error",
	"reserved",
};


const char *can_get_kind_of_error_string(CM_CAN_TypeDef *can)
{
	uint32_t error_index = can_get_kind_of_error(can)>>5;
	//log_printf("error_index:%lu\n", error_index);
	if (error_index < sizeof(s_koe_string)/sizeof(s_koe_string[0])) {
		return s_koe_string[error_index];
	} else {
		return "";
	}
}
