#ifndef ESP_AT_REPORT_CALLBACK_H
#define ESP_AT_REPORT_CALLBACK_H

#include <stdint.h>
#include <stdbool.h>

#define AT_REPORT_CALLBACK_SLOT_SIZE		128		// 2**7=128
#define AT_REPORT_CALLBACK_KNUTH_HASH_P		7

struct at_context_t;
typedef void (*ESP_AT_COMMAND_REPORT_CALLBACK)(struct at_context_t *ctx);

struct at_report_callback_node_t {
	char *name;
	ESP_AT_COMMAND_REPORT_CALLBACK callback;
	struct at_report_callback_node_t *next;
};

struct at_report_callback_map_t {
	struct at_report_callback_node_t **slot;
	uint16_t slot_size;
	uint16_t elem_size;
};

extern struct at_report_callback_map_t *at_report_callback_map_init(void);
extern bool at_report_callback_map_register(const char *name, ESP_AT_COMMAND_REPORT_CALLBACK callback);
extern ESP_AT_COMMAND_REPORT_CALLBACK at_report_callback_map_get_callback(const char *name);


#endif	// ESP_AT_REPORT_CALLBACK_H
