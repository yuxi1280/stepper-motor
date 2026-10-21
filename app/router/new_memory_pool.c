#include "new_memory_pool.h"

#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

extern void mp_lock(void);
extern void mp_unlock(const char *tag);



struct new_mp_block_t *s_free_slots[] = {
	NULL,		// 2**0,1
	NULL,		// 2**1
	NULL,		// 2**2
	NULL,		// 2**3
	NULL,		// 2**4
	NULL,		// 2**5	// >= 16B
	NULL,		// 2**6
	NULL,		// 2**7
	NULL,		// 2**8
	NULL,		// 2**9
	NULL,		// 2**10
	NULL,		// 2**11
	NULL,		// 2**12
	NULL,		// 2**13
	NULL,		// 2**14
	NULL,		// 2**15
	NULL,		// 2**16
	NULL,		// 2**17
	NULL,		// 2**18
	NULL,		// 2**19
	NULL,		// 2**20,1MiB
	NULL,		// 2**21,2MiB
	NULL,		// 2**22,4MiB
	NULL,		// 2**23,8MiB
	NULL,		// 2**24,16MiB
	NULL,		// 2**25,32MiB
	NULL,		// 2**26,64MiB
	NULL,		// 2**27,128MiB
};

extern void new_mp_insert_free_block(struct new_mp_block_t *new_block);

void new_mp_init(struct new_mp_span_t *spans, int n)
{
	struct new_mp_block_t *block = NULL;
	struct new_mp_span_t *span = NULL;

	for (int i = 0; i < n; ++i ) {
		span = spans + i;

		block = (struct new_mp_block_t *)(uintptr_t)span->_addr;
		block->_prev = 0x2;
		block->_size = span->_size-sizeof(struct new_mp_block_t);
		block->_slot_next = NULL;
		block->_slot_prev = NULL;
		new_mp_insert_free_block(block);
	}
}

int new_mp_get_block_index(int n)
{
	int pos = 31-__builtin_clz(n);
	if ((1<<pos) == n) {
		return pos;
	} else {
		return pos+1;
	}
}


void new_mp_insert_free_block(struct new_mp_block_t *new_block)
{
//	printf("@@new_block:[%08x]%p,%u\n", new_block->_prev, new_block, new_block->_size);
	int j = new_mp_get_block_index(new_block->_size);

	if (s_free_slots[j] == NULL) {
		s_free_slots[j] = new_block;
	} else {
		struct new_mp_block_t *tmp_block = s_free_slots[j];
		struct new_mp_block_t *prev_block = NULL;
		while (tmp_block != NULL) {
			if (new_block->_size > tmp_block->_size) {
				prev_block = tmp_block;
				tmp_block = tmp_block->_slot_next;
			} else {
				break;
			}
		}
		if (prev_block == NULL) {
			// 新块的尺寸最小
			new_block->_slot_next = tmp_block;
			tmp_block->_slot_prev = new_block;
			s_free_slots[j] = new_block;
		} else {
			if (prev_block->_slot_next != NULL) {
				prev_block->_slot_next->_slot_prev = new_block;
			}
			new_block->_slot_next = prev_block->_slot_next;
			new_block->_slot_prev = prev_block;

			prev_block->_slot_next = new_block;
		}
	}
}

void new_mp_remove_free_block(int i, struct new_mp_block_t *block)
{
	if (block->_slot_next != NULL) {
		block->_slot_next->_slot_prev = block->_slot_prev;
	}
	if (block->_slot_prev != NULL) {
		block->_slot_prev->_slot_next = block->_slot_next;
	} else {
		s_free_slots[i] = block->_slot_next;
	}

	block->_slot_prev = NULL;
	block->_slot_next = NULL;
}


void *new_mp_malloc(int n)
{
	struct new_mp_block_t *block_head;
	struct new_mp_block_t *block;

	if (n == 0) {
		return NULL;
	}

	int pos = new_mp_get_block_index(n);
	int nn = 0;

	if (n <= 32) {
		if (n <= 8) {
			nn = 8;
		} else if (n <= 16) {
			nn = 16;
		} else {
			nn = 32;
		}
	} else {
		nn = (n&~0xf) + (n&0xf ? 16 : 0);
	}
//	printf("alloc:n:%d->nn:%d,pos:%d\n", n, nn, pos);
	mp_lock();

	for (int i = pos; i < 28; ++i)
	{
		block_head = s_free_slots[i];
		block = block_head;
		while (block != NULL) {
			// 尽量找最小能满足的块
			if (block->_size >= nn) {
				if (block->_size - nn < 24) {
					// 不够分成最小的块，则直接返回此块
					new_mp_remove_free_block(i, block);
					block->_prev |= 0x1;
					//printf("[%08x]block:%d->%p\n", block->_prev, block->_size, block);
					mp_unlock("163");
					return block->_data;
				} else {
					// 将大块分成更小的块
					struct new_mp_block_t *next_block;
					if (block->_prev&0x2) {
						next_block = NULL;
					} else {
						next_block = (struct new_mp_block_t *)((uintptr_t)block+sizeof(struct new_mp_block_t)+block->_size);
					}

					// 分出的新的块，将其放入合适的槽中
					// 按照从小到大顺序排列
					struct new_mp_block_t *new_block
						= (struct new_mp_block_t *)((uint32_t)block + sizeof(struct new_mp_block_t) + nn);

					new_block->_prev = (uint32_t)block;

					if (next_block == NULL) {
						block->_prev &= ~0x2;
						new_block->_prev |= 0x2;
					} else {
						next_block->_prev = (next_block->_prev&0x3) | (uintptr_t)new_block;
					}

					new_block->_size = block->_size - (nn + sizeof(struct new_mp_block_t));
					new_block->_slot_next = NULL;
					new_block->_slot_prev = NULL;


					new_mp_insert_free_block(new_block);


					block->_size = nn;
					new_mp_remove_free_block(i, block);
					block->_prev |= 0x1;
					//printf("$[%08x]block:%d->%p\n", block->_prev, block->_size, block);
					//printf("$[%08x]new_block:%d->%p\n", new_block->_prev, new_block->_size, new_block);
					mp_unlock("201");
					return block->_data;
				}
			}

			block = block->_slot_next;
		}
	}
	mp_unlock("209");
	return NULL;
}


void new_mp_free(void *p)
{
	struct new_mp_block_t *block;
	if (p == NULL) {
		return;
	}
	// printf("free:%d,%d->%p\n", i, block->_size, block);

	mp_lock();

	block = (struct new_mp_block_t *)((uintptr_t)p-sizeof(struct new_mp_block_t));
	block->_prev &= ~0x1;
	new_mp_insert_free_block(block);

	// 合并相邻的空闲块
	// 向前找，直到块不再空闲
	struct new_mp_block_t *tmp_block = block;
	struct new_mp_block_t *prev_block;
	while (tmp_block != NULL) {
//		printf("tmp_block:%p,%d\n", tmp_block, tmp_block->_size);

		prev_block = (struct new_mp_block_t *)((uintptr_t)tmp_block->_prev&~0x3);
		if (prev_block == NULL) {
			break;
		}
		if (prev_block->_prev&0x1) {
			break;
		}
		tmp_block = prev_block;
	}

	struct new_mp_block_t *next_block;
	if (block->_prev&0x2) {
		next_block = NULL;
	} else {
		next_block = (struct new_mp_block_t *)((uintptr_t)block+sizeof(struct new_mp_block_t)+block->_size);
	}
	if (tmp_block == block && (next_block == NULL || next_block->_prev&0x1)) {
		// 不需要合并
		mp_unlock("253");
		return;
	}


	// 向后合并，直到块不再空闲
	struct new_mp_block_t *new_block = tmp_block;
	int n = 0;
	while (tmp_block != NULL) {
		//printf("collect:%p,%d\n", tmp_block, tmp_block->_size);
		new_mp_remove_free_block(new_mp_get_block_index(tmp_block->_size), tmp_block);
	
		n = sizeof(struct new_mp_block_t) + tmp_block->_size;

		if (tmp_block != new_block) {
			new_block->_size += n;
		}

		// 最后一块标志
		if (tmp_block->_prev&0x2) {
			// 新块变成最后一块
			new_block->_prev |= 0x2;
			break;
		}

		tmp_block = (struct new_mp_block_t *)((uintptr_t)tmp_block+n);
		if (tmp_block->_prev&0x1) {
			break;
		}
	}

	// 新块不是最后一块
	if (!(new_block->_prev&0x2)) {
		tmp_block->_prev = (tmp_block->_prev&0x3) | (uintptr_t)new_block;;
	}

	new_mp_insert_free_block(new_block);

	mp_unlock("291");
	//printf("new_block:%d\n", new_block->_size);
}

void new_mp_dump(struct new_mp_span_t *spans, int n, uint32_t *used_memory_ptr, uint32_t *free_memory_ptr)
{
	struct new_mp_block_t *block;
	struct new_mp_block_t *prev_block;
	(void)prev_block;
	struct new_mp_span_t *span;

	uint32_t free_memory = 0;
	uint32_t used_memory = 0;
	uint32_t total_free_memory = 0;
	uint32_t total_used_memory = 0;

	mp_lock();

	for (int i = 0; i < n; ++i) {
		span = spans + i;
		block = (struct new_mp_block_t *)(uintptr_t)span->_addr;

		free_memory = 0;
		used_memory = 0;
		while (block != NULL) {

#if	0
			if ((uintptr_t)block < span->_addr || (uintptr_t)block >= span->_addr+span->_size) {

				printf("<%p>[%p]:", prev_block, block);
				printf("%08x,%d->%d\n", prev_block->_prev, prev_block->_size, new_mp_get_block_index(prev_block->_size));
				printf("%08x,%d->%d\n", block->_prev, block->_size, new_mp_get_block_index(block->_size));
			}
#endif
			if (block->_prev&0x1) {
				used_memory += block->_size;
			} else {
				free_memory += block->_size;
			}
			if (block->_prev&0x2) {
				break;
			}
			prev_block = block;
			block = (struct new_mp_block_t *)((uintptr_t)block+sizeof(struct new_mp_block_t)+block->_size);
		}

		printf("[%08"PRIx32",%"PRIi32"]free_memory:%"PRIu32",used_memory:%"PRIu32"\n", span->_addr, span->_size, free_memory, used_memory);

		total_free_memory += free_memory;
		total_used_memory += used_memory;
	}

	mp_unlock("343");

	*free_memory_ptr = total_free_memory;
	*used_memory_ptr = total_used_memory;
}



void *new_mp_calloc(int num, int size)
{
	void *ptr = new_mp_malloc(num*size);
	if (ptr != NULL) {
		memset(ptr, 0, num*size);
	}
	return ptr;
}

void *new_mp_realloc(void *ptr, int new_size)
{
	if (ptr == NULL) {
		void *new_ptr = new_mp_malloc(new_size);
		return new_ptr;
	}

	struct new_mp_block_t *block = (struct new_mp_block_t *)((uintptr_t)ptr-sizeof(struct new_mp_block_t));
	if (block->_size >= new_size) {
		return ptr;
	}

	void *new_ptr = new_mp_malloc(new_size);
	if (new_ptr != NULL) {
		memcpy(new_ptr, ptr, block->_size);
		new_mp_free(ptr);
		return new_ptr;
	} else {
		new_mp_free(ptr);
		return NULL;
	}
}
