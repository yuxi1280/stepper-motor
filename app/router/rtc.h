#ifndef __RTC_H__
#define __RTC_H__


#include <stdint.h>

extern void rtc_init(void);
extern void rtc_poll(void);

#if	0
extern void rtc_write_backup_register(const uint8_t *p, int n);
extern void rtc_read_backup_register(uint8_t *p, int n);
#endif
extern void rtc_set_date_time(uint16_t year, uint8_t month, uint8_t day, uint8_t wday,
		uint8_t hour, uint8_t minute, uint8_t second);

#endif	// __RTC_H__
