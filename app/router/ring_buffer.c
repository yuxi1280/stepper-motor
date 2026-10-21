#include "ring_buffer.h"
#include <string.h>

int can_frame_get_free_size(can_tx_frame_buffer_t *rb)
{
	int tail = rb->tail;	// 在主程序中增加
	int head = rb->head;	// 在中断中减少

	if (tail < head) {
		return (CAN_TX_RING_BUFFER_SIZE-1) - (tail + CAN_TX_RING_BUFFER_SIZE - head);
	} else {
		return (CAN_TX_RING_BUFFER_SIZE-1) - (tail - head);
	}
}

usb_packet_t *usb_packet_buffer_put(usb_packet_buffer_t *rb, const uint8_t *p, int n)
{
	usb_packet_t *packet = rb->packets + rb->tail;
	if (p != NULL && n > 0) {
		memcpy(packet->data, p, n);
	}
	packet->size = n;
	rb->tail = (rb->tail+1)%USB_PACKET_RING_BUFFER_SIZE;
	return packet;
}


usb_packet_t *usb_packet_buffer_get(usb_packet_buffer_t *rb)
{
	usb_packet_t *packet = rb->packets + rb->head;
	rb->head = (rb->head+1)%USB_PACKET_RING_BUFFER_SIZE;
	return packet;
}

usb_packet_t *usb_packet_buffer_head(usb_packet_buffer_t *rb)
{
	return rb->packets + rb->head;
}

void usb_packet_buffer_remove_head(usb_packet_buffer_t *rb)
{
	rb->head = (rb->head+1)%USB_PACKET_RING_BUFFER_SIZE;
}
