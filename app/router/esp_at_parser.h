#ifndef ESP_AT_PARSER_H
#define ESP_AT_PARSER_H

#include <stdint.h>
#include "esp_at_report_callback.h"
#include "new_memory_pool.h"

#define ESP_AT_MALLOC	new_mp_malloc
#define ESP_AT_FREE		new_mp_free

#define ESP_AT_COMMAND_SIZE		1024
#define ESP_AT_LINE_SIZE		4096
#define ESP_AT_TX_DATA_SIZE		1500

struct at_context_t;

typedef void (*ESP_AT_COMMAND_RESPONSE_CALLBACK)(struct at_context_t *ctx);
typedef void (*ESP_AT_COMMAND_TIMEOUT_CALLBACK)(struct at_context_t *ctx);

struct at_command_t {
	char *_command;
	int _command_size;
	ESP_AT_COMMAND_RESPONSE_CALLBACK response_callback;
	ESP_AT_COMMAND_REPORT_CALLBACK report_callback;
	ESP_AT_COMMAND_TIMEOUT_CALLBACK timeout_callback;
	uint8_t *_extra_data;
	int _extra_size;
};


#define ESP_AT_COMMAND_QUEUE_SIZE		32

struct at_command_queue_t {
	int _head, _tail;
	struct at_command_t *_commands[ESP_AT_COMMAND_QUEUE_SIZE];
};


struct at_context_t {
	int _line_state;
	uint8_t _line[ESP_AT_LINE_SIZE];
	int _line_index;

	uint8_t _tx_data[ESP_AT_TX_DATA_SIZE];
	int _tx_data_size;

	int _ipd_length_index;
	int _ipd_data_index;
	int _ipd_length;
	int _ipd_index;


	struct at_command_queue_t _command_q;
	struct at_command_t *_command;
	int64_t _command_timestamp;
	int _timeout_count;
};


extern int esp_at_init(struct at_context_t *ctx);

int esp_at_try_tx_command(struct at_context_t *ctx, const char *p, int n,
		ESP_AT_COMMAND_RESPONSE_CALLBACK rc,
		ESP_AT_COMMAND_REPORT_CALLBACK report_callback,
		ESP_AT_COMMAND_TIMEOUT_CALLBACK tc,
		uint8_t *extra_data,
		int extra_size);

extern int esp_at_on_rx(struct at_context_t *ctx, const uint8_t *p, int n);

extern int esp_at_poll(struct at_context_t *ctx, int64_t now);




#endif	// ESP_AT_PARSER_H
