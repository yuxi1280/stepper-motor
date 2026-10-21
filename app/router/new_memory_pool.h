#ifndef __NEW_MEMORY_POOL_H__
#define __NEW_MEMORY_POOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MP_BLOCK_TYPE_COUNT		(28)
#define MP_ALLOC_SIZE_MAX		(10*512*1024)

struct new_mp_block_t {
	uint32_t _prev;
	uint32_t _size;
	struct new_mp_block_t *_slot_prev, *_slot_next;

	uint8_t _data[0];
};

struct new_mp_span_t {
	uint32_t _addr;
	uint32_t _size;
};


extern void new_mp_init(struct new_mp_span_t *spans, int n);

extern void *new_mp_malloc(int size);
extern void *new_mp_calloc(int num, int size);
extern void *new_mp_realloc(void *ptr, int new_size);

extern void new_mp_free(void *ptr);


extern void new_mp_dump(struct new_mp_span_t *spans, int n, uint32_t *used_memory_ptr, uint32_t *free_memory_ptr);



#ifdef __cplusplus
}
#endif

#endif	// __NEW_MEMORY_POOL_H__
