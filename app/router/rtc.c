#include "rtc.h"

#include "hc32_ll.h"
#include "board.h"


#define RTC_BACKUP_REGISTER_OFFSET		64
#define RTC_BACKUP_REGISTER_SIZE		32

//#define RTC_CLK_SRC_SEL					(RTC_CLK_SRC_LRC)
#define RTC_CLK_SRC_SEL					(RTC_CLK_SRC_XTAL32)


static __IO uint8_t s_rtc_period_flag = 0;

static void rtc_period_irq_callback(void)
{
    s_rtc_period_flag = 1;
    RTC_ClearStatus(RTC_FLAG_PERIOD);
}

#if	1
static const uint8_t s_rtc_wb[RTC_BACKUP_REGISTER_SIZE] = {
	0x19, 0x80, 0x11, 0x02, 0x19, 0x91, 0x10, 0x24,
	0x20, 0x16, 0x02, 0x22, 0x20, 0x16, 0x02, 0x22,
	0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
	0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
};
static uint8_t s_rtc_rb[RTC_BACKUP_REGISTER_SIZE];


static void rtc_write_backup_register(void)
{
	for (uint8_t i = 0U; i < RTC_BACKUP_REGISTER_SIZE; ++i) {
		PWC_BKR_Write(RTC_BACKUP_REGISTER_OFFSET + i, s_rtc_wb[i]);
	}
}

static int32_t rtc_check_backup_register(void)
{
	uint8_t i;
	int32_t result = LL_OK;

	for (i = 0U; i < RTC_BACKUP_REGISTER_SIZE; i++) {
		s_rtc_rb[i] = PWC_BKR_Read(RTC_BACKUP_REGISTER_OFFSET + i);
	}
	for (i = 0U; i < RTC_BACKUP_REGISTER_SIZE; i++) {
		if (s_rtc_rb[i] != s_rtc_wb[i]) {
			result = LL_ERR;
			break;
		}
	}

	return result;
}
#endif


#if	0
void rtc_write_backup_register(const uint8_t *p, int n)
{
	for (int i = 0; i < n; ++i) {
		PWC_BKR_Write(RTC_BACKUP_REGISTER_OFFSET + i, p[i]);
	}
}

void rtc_read_backup_register(uint8_t *p, int n)
{
	for (int i = 0; i < n; i++) {
		p[i] = PWC_BKR_Read(RTC_BACKUP_REGISTER_OFFSET + i);
	}
}

#endif

#if	1
static int __kim_larsen_wday(int year, int month, int mday)
{
	return (mday + 2 * month + 3 * (month + 1) / 5 + year + year / 4 - year / 100 + year / 400+1) % 7;
}
#endif

void rtc_set_date_time(uint16_t year, uint8_t month, uint8_t day, uint8_t wday,
		uint8_t hour, uint8_t minute, uint8_t second)
{
	stc_rtc_date_t rtc_date;
	stc_rtc_time_t rtc_time;

	rtc_date.u8Year		= year-2000;
	rtc_date.u8Month	= month;		// 1~12
	rtc_date.u8Day		= day;			// 1~31
	rtc_date.u8Weekday	= __kim_larsen_wday(year, month, day);	// (0)sunday~>(6)saturday
//	rtc_date.u8Weekday	= wday;

	rtc_time.u8Hour		= hour;
	rtc_time.u8Minute	= minute;
	rtc_time.u8Second	= second;
	rtc_time.u8AmPm		= RTC_HOUR_24H;

	if (RTC_SetDate(RTC_DATA_FMT_DEC, &rtc_date) != LL_OK) {
		log_printf("RTC_SetDate failed\n");
	}

	if (RTC_SetTime(RTC_DATA_FMT_DEC, &rtc_time) != LL_OK) {
		log_printf("RTC_SetTime failed\n");
	}
}

void rtc_init(void)
{
	int32_t result;
	stc_rtc_init_t rtc_init;
	stc_irq_signin_config_t irq_cfg;

	irq_cfg.enIntSrc		= INT_SRC_RTC_PRD;
	irq_cfg.enIRQn			= INT052_IRQn;
	irq_cfg.pfnCallback		= &rtc_period_irq_callback;
	(void)INTC_IrqSignOut(irq_cfg.enIRQn);
	result = INTC_IrqSignIn(&irq_cfg);
	if (result != LL_OK) {
		for (;;) {
		}
	}

	NVIC_ClearPendingIRQ(irq_cfg.enIRQn);
	NVIC_SetPriority(irq_cfg.enIRQn, DDL_IRQ_PRIO_DEFAULT);
	NVIC_EnableIRQ(irq_cfg.enIRQn);
	
#if	1
	if (rtc_check_backup_register() == LL_OK) {
		if (RTC_EnterRwMode() == LL_OK) {
			if (RTC_ExitRwMode() == LL_OK) {
				return;
			}
		}
	}
#endif

	PWC_VBAT_Reset();

#if RTC_CLK_SRC_SEL == RTC_CLK_SRC_XTAL32
	if (xtal32_init() != LL_OK) {
		log_printf("xtal32_init failed\n");
		return;
	}
#endif

	if (RTC_DeInit() == LL_ERR_TIMEOUT) {
		log_printf("RTC_DeInit timeout\n");
	} else {
		RTC_Cmd(DISABLE);
		(void)RTC_StructInit(&rtc_init);

		rtc_init.u8ClockSrc		= RTC_CLK_SRC_SEL;
		rtc_init.u8HourFormat	= RTC_HOUR_FMT_24H;
		rtc_init.u8IntPeriod	= RTC_INT_PERIOD_PER_SEC;
		(void)RTC_Init(&rtc_init);

		rtc_set_date_time(2026, 4, 3, 5, 11, 28, 0);

		RTC_ClearStatus(RTC_FLAG_CLR_ALL);
		RTC_IntCmd(RTC_INT_PERIOD, ENABLE);

		RTC_Cmd(ENABLE);

		rtc_write_backup_register();
	}
}

void rtc_poll(void)
{
	stc_rtc_date_t rtc_date;
	stc_rtc_time_t rtc_time;

	if (s_rtc_period_flag == 1) {
		s_rtc_period_flag = 0;

		if (LL_OK == RTC_GetDate(RTC_DATA_FMT_DEC, &rtc_date)) {
			if (LL_OK == RTC_GetTime(RTC_DATA_FMT_DEC, &rtc_time)) {
				log_printf("20%02d/%02d/%02d %02d:%02d:%02d %u\n", rtc_date.u8Year, rtc_date.u8Month,
						rtc_date.u8Day, rtc_time.u8Hour,
						rtc_time.u8Minute, rtc_time.u8Second, rtc_date.u8Weekday);
			} else {
				log_printf("RTC_GetTime failed\n");
			}
		} else {
			log_printf("RTC_GetDate failed\n");
		}
	}
}
