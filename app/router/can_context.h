#ifndef CAN_CONTEXT_H
#define CAN_CONTEXT_H

#include "can.h"
#include "ring_buffer.h"
#include <stdbool.h>

typedef struct __can_context_t {
	CM_CAN_TypeDef *_can;
	can_rx_frame_buffer_t _rx_fb;
	can_tx_frame_buffer_t _tx_fb;
	bool _transmit_ptb_ready;
	bool _is_opened;
	uint32_t _error_count;
} can_context_t;

typedef struct __attribute__((packed, aligned(1))) __can_filter_t {
	uint8_t _acf;
	uint8_t _ft;
	uint32_t _code;
	uint32_t _mask;
} can_filter_t;

typedef struct __attribute__((packed, aligned(1))) __can_config_t {
	uint32_t _presc;
	uint32_t _seg_1;
	uint32_t _seg_2;
	uint32_t _sjw;

	uint32_t _filter_size;
	can_filter_t _filters[16];

	uint8_t _almost_full_warning_limit;
	uint8_t _error_warning_limit;
} can_config_t;

extern void can_open(can_context_t *can);
extern void can_close(can_context_t *can);
extern void can_config(can_context_t *can, can_config_t *cfg);

extern bool can_tx_data_frames(can_context_t *ctx, uint8_t ide, uint32_t id, const uint8_t *p, int n);
extern bool can_tx_remote_frame(can_context_t *ctx, uint8_t ide, uint32_t id);
extern void can_tx_frame(can_context_t *ctx, can_frame_t *f);

extern void can_irq_callback(can_context_t *ctx);
extern void can_poll(can_context_t *ctx);

extern void can_on_recv(can_context_t *ctx, can_frame_t *f);


#define CAN_ERROR_OK				0x00
#define CAN_ERROR_AL				0x01	// Arbitration Lost
#define CAN_ERROR_RO				0x02	// Recieve Overflow
#define CAN_ERROR_E					0x03	// Error
#define CAN_ERROR_EP				0x04	// Error Passive
#define CAN_ERROR_BE				0x05	// Bus Error

extern void can_on_error(can_context_t *ctx, uint8_t error);

#define can_get_channel(ctx)\
	((ctx)->_can == CM_CAN1 ? 1 : 2)

#endif	// CAN_CONTEXT_H
