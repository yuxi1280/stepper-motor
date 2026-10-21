#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include <stdint.h>
#include <string.h>
#include "can.h"

#define CAN_TX_RING_BUFFER_SIZE		4096
#define CAN_RX_RING_BUFFER_SIZE		512

typedef struct __can_tx_frame_buffer_t {
	uint16_t head, tail;
	uint32_t size;
	can_frame_t frames[CAN_TX_RING_BUFFER_SIZE];
} can_tx_frame_buffer_t;


typedef struct __can_rx_frame_buffer_t {
	uint16_t head, tail;
	uint32_t size;
	can_frame_t frames[CAN_RX_RING_BUFFER_SIZE];
} can_rx_frame_buffer_t;


#define can_frame_buffer_init(fb, n)\
	{\
		(fb).head = 0;\
		(fb).tail = 0;\
		(fb).size = (n);\
	}

extern int can_frame_get_free_size(can_tx_frame_buffer_t *rb);

#define can_frame_buffer_empty(fb)\
	((fb).head == (fb).tail)
#define can_frame_buffer_not_empty(fb)\
	((fb).head != (fb).tail)

#define can_frame_buffer_not_full(fb)\
	(((fb).tail+1)%((fb).size) != (fb).head)

#define can_frame_buffer_put(fb, f)\
	{\
		memcpy(&(fb).frames[(fb).tail], (f), sizeof(can_frame_t));\
		(fb).tail = ((fb).tail+1)%((fb).size);\
	}

#define can_frame_buffer_get(fb, f)\
	memcpy((f), &(fb).frames[(fb).head], sizeof(can_frame_t));\
	(fb).head = ((fb).head+1)%((fb).size);

#define can_frame_buffer_head(fb, f)\
	memcpy((f), &(fb).frames[(fb).head], sizeof(can_frame_t));
#define can_frame_buffer_remove_head(fb)\
	(fb).head = ((fb).head+1)%((fb).size);

#define can_frame_buffer_clear(fb)\
	(fb).head = (fb).tail = 0



#define BYTE_BUFFER_SIZE		4096

typedef struct __byte_buffer_t {
	uint16_t head, tail;
	uint8_t *buffer;
	int size;
} byte_buffer_t;

#define byte_buffer_init(fb, n)\
	{\
		(fb).head = 0;\
		(fb).tail = 0;\
		(fb).size = n;\
		(fb).buffer = new_mp_malloc((n));\
	}

#define byte_buffer_not_empty(fb)\
	((fb).head != (fb).tail)

#define byte_buffer_not_full(fb)\
	(((fb).tail+1)%(fb).size != (fb).head)

#define byte_buffer_put(fb, b)\
	{\
		fb.buffer[fb.tail] = b;\
		fb.tail = (fb.tail+1)%BYTE_BUFFER_SIZE;\
	}

#define byte_buffer_get(fb, b)\
	{\
		b = fb.buffer[fb.head];\
		fb.head = (fb.head+1)%BYTE_BUFFER_SIZE;\
	}




#include "usb_app_conf.h"

#define USB_PACKET_RING_BUFFER_SIZE		32

typedef struct __usb_packet_t {
	uint8_t data[MAX_CDC_PACKET_SIZE];
	int size;
} usb_packet_t;

typedef struct __usb_packet_buffer_t {
	uint16_t head, tail;
	usb_packet_t packets[USB_PACKET_RING_BUFFER_SIZE];
} usb_packet_buffer_t;

#define usb_packet_buffer_init(fb)\
	{\
		(fb).head = 0;\
		(fb).tail = 0;\
	}

#define usb_packet_buffer_empty(fb)\
	((fb).head == (fb).tail)
#define usb_packet_buffer_not_empty(fb)\
	((fb).head != (fb).tail)

#define usb_packet_buffer_not_full(fb)\
	(((fb).tail+1)%USB_PACKET_RING_BUFFER_SIZE != (fb).head)





extern usb_packet_t *usb_packet_buffer_put(usb_packet_buffer_t *rb, const uint8_t *p, int n);
extern usb_packet_t *usb_packet_buffer_get(usb_packet_buffer_t *rb);
extern usb_packet_t *usb_packet_buffer_head(usb_packet_buffer_t *rb);
extern void usb_packet_buffer_remove_head(usb_packet_buffer_t *rb);


#endif	// __RING_BUFFER_H__
