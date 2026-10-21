#include "newlib_sys_calls.h"
#include "board.h"

#include <stdlib.h>
#include <sys/stat.h>	// struct stat


extern uint8_t __HeapBase[];
extern uint8_t __HeapLimit[];

// startup_hc32f460jeta.S
char *_heap_ptr = (char *)__HeapBase;
char *_heap_limit = (char *)__HeapLimit;

caddr_t _sbrk(int n)
{
	// 00 00 00 00
	// 00 00 04 08	-> 1032
#if	0
	debug_usart_tx_byte('$');
	debug_usart_tx_byte((n>>24)&0xFF);
	debug_usart_tx_byte((n>>16)&0xFF);
	debug_usart_tx_byte((n>>8)&0xFF);
	debug_usart_tx_byte((n>>0)&0xFF);
#endif

	char *p = _heap_ptr;
	if (_heap_ptr+n > _heap_limit) {
		return (caddr_t)-1;
	}

	_heap_ptr += n;

	return p;
}

int _close(int fd)
{
	return -1;
}

int _fstat(int fd, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int fd)
{
	return 1;
}

int _lseek(int fd, int offset, int whence)
{
	return 0;
}

void _exit(int status)
{
	__asm("BKPT	#0");
	return;
}

void _kill(int pid, int sig)
{

}

int _getpid(void)
{
	return -1;
}

/*
int _write(int fd, char *p, int n)
{
	return 0;
}
*/

int _read(int fd, char *p, int n)
{
	return 0;
}

#include <sys/time.h>

/*
time() returns the time as the number of seconds since the Epoch,
       1970-01-01 00:00:00 +0000 (UTC).
*/

int _gettimeofday(struct timeval *tv, struct timezone *tz)
{

	return 0;
}
