#include "esp_at_report_callback.h"
#include "esp_at_parser.h"

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>


static struct at_report_callback_map_t *s_report_callback_map;


#define AT_REPORT_CALLBACK_S				2654435769

static int at_report_callback_knuth_hash(const char *k)
{
	unsigned long hash = 5381;
	int c;
	const char *s = k;
	while ((c = *s++) != '\0') {
		hash = ((hash<<5) + hash) + c;	/* hash * 33 + c */
	}

	int64_t result = hash * AT_REPORT_CALLBACK_S;
	int p = AT_REPORT_CALLBACK_KNUTH_HASH_P;
	return ((uint64_t)result&0xFFFFFFFF)>>(32-p);
}

#undef AT_report_callback_S



static struct at_report_callback_node_t *at_report_callback_node_create(const char *name,
		ESP_AT_COMMAND_REPORT_CALLBACK callback)
{
	struct at_report_callback_node_t *x = ESP_AT_MALLOC(sizeof(struct at_report_callback_node_t));
	if (x != NULL) {
		int n = strlen(name);
		x->name = ESP_AT_MALLOC(n+1);
		if (x->name == NULL) {
			ESP_AT_FREE(x);
			return NULL;
		}
		memcpy(x->name, name, n);
		x->name[n] = '\0';
		x->callback = callback;
		x->next = NULL;
		return x;
	} else {
		return NULL;
	}
}

static void at_report_callback_node_destroy(struct at_report_callback_node_t *x)
{
	if (x != NULL) {
		if (x->name != NULL) {
			ESP_AT_FREE(x->name);
		}
		ESP_AT_FREE(x);
	}
}


struct at_report_callback_map_t *at_report_callback_map_init(void)
{
	struct at_report_callback_map_t *t = ESP_AT_MALLOC(sizeof(struct at_report_callback_map_t));
	if (t != NULL) {
		t->slot_size = AT_REPORT_CALLBACK_SLOT_SIZE;
		t->elem_size = 0;

		t->slot = ESP_AT_MALLOC(sizeof(struct at_report_callback_node_t *) * t->slot_size);
		if (t->slot != NULL) {
			for (uint16_t i = 0; i < t->slot_size; ++i) {
				t->slot[i] = NULL;
			}

			s_report_callback_map = t;
			return t;
		} else {
			ESP_AT_FREE(t);
			return NULL;
		}
	} else {
		return NULL;
	}
}

void at_report_callback_map_free_node(const char *name)
{
	struct at_report_callback_node_t *x;
	int slot_index;

	slot_index = at_report_callback_knuth_hash(name);
	x = s_report_callback_map->slot[slot_index];
	struct at_report_callback_node_t *prev_x = x;


	while (x != NULL) {
		if (strcmp(x->name, name) == 0) {
//			printf("free %s @[%d,%d]\n", path, slot_index, j);
			if (x == s_report_callback_map->slot[slot_index]) {
				s_report_callback_map->slot[slot_index] = x->next;
			} else {
				prev_x->next = x->next;
			}

			at_report_callback_node_destroy(x);

			return;
		}

		prev_x = x;
		x = x->next;
	}
}

bool at_report_callback_map_register(const char *name, ESP_AT_COMMAND_REPORT_CALLBACK callback)
{
	struct at_report_callback_node_t *x;
	int slot_index;


	x = at_report_callback_node_create(name, callback);
	if (x == NULL) {
		return false;
	}

	slot_index = at_report_callback_knuth_hash(name);

	if (s_report_callback_map->slot[slot_index] == NULL) {
		s_report_callback_map->slot[slot_index] = x;
	} else {
		x->next = s_report_callback_map->slot[slot_index];
		s_report_callback_map->slot[slot_index] = x;
	}

	return true;
}

ESP_AT_COMMAND_REPORT_CALLBACK at_report_callback_map_get_callback(const char *name)
{
	struct at_report_callback_node_t *x;
	int slot_index;

	slot_index = at_report_callback_knuth_hash(name);
	x = s_report_callback_map->slot[slot_index];

	while (x != NULL) {
		if (strcmp(x->name, name) == 0) {
			printf("found:%s,%d\n", x->name, slot_index);
			return x->callback;
		}

		x = x->next;
	}

	return NULL;
}

