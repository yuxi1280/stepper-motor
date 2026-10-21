#include "esp_at_parser.h"
#include "app.h"
#include "hc32_ll_utility.h"
#include "shelf.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

//#define ESP_AT_DEBUG


void esp_at_command_destroy(struct at_command_t *cmd)
{
	if (cmd != NULL) {
		if (cmd->_extra_data) {
			ESP_AT_FREE(cmd->_extra_data);
			cmd->_extra_data = NULL;
		}

		if (cmd->_command != NULL) {
			ESP_AT_FREE(cmd->_command);
			cmd->_command = NULL;
		}

		ESP_AT_FREE(cmd);
	}
}

struct at_command_t *esp_at_command_get(struct at_command_queue_t *cmd_q)
{
	if (cmd_q->_head == cmd_q->_tail) {
		//printf("cmd_q empty\n");
		return NULL;
	}

	return cmd_q->_commands[cmd_q->_head];
}

bool esp_at_command_enqueue(struct at_command_queue_t *cmd_q, struct at_command_t *cmd)
{
	if ((cmd_q->_tail+1)%ESP_AT_COMMAND_QUEUE_SIZE == cmd_q->_head) {
		printf("cmd_q full\n");
		return false;
	}

	cmd_q->_commands[cmd_q->_tail] = cmd;
	cmd_q->_tail = (cmd_q->_tail+1)%ESP_AT_COMMAND_QUEUE_SIZE;
	//printf("cmd_q:%d,%d\n", cmd_q->_head, cmd_q->_tail);

	return true;
}

extern int esp_at_try_tx_command_from_command_q(struct at_context_t *ctx);

bool esp_at_command_dequeue(struct at_context_t *ctx, struct at_command_queue_t *cmd_q, bool try)
{
	//printf("esp_at_command_dequeue\n");
	if (cmd_q->_head == cmd_q->_tail) {
		printf("cmd_q empty\n");
		return false;
	}

	struct at_command_t *cmd = cmd_q->_commands[cmd_q->_head];
	esp_at_command_destroy(cmd);

	cmd_q->_head = (cmd_q->_head+1)%ESP_AT_COMMAND_QUEUE_SIZE;
	
	ctx->_command = NULL;
	
	if (try) {
		esp_at_try_tx_command_from_command_q(ctx);
	}

	return true;
}


int esp_at_init(struct at_context_t *ctx)
{
	ctx->_line_state = 0;
	ctx->_line_index = 0;
	ctx->_command = NULL;
	ctx->_tx_data_size = 0;

	return 0;
}

extern void wifi_usart_tx_bytes(const uint8_t *p, int n);


int esp_at_tx_command(struct at_context_t *ctx, struct at_command_t *cmd)
{
	//printf("esp_at_tx_command:%s\n", cmd->_command);
	ctx->_command = cmd;
	ctx->_command_timestamp = sys_tick_get_infinite_ticks();
	wifi_usart_tx_bytes((const uint8_t *)cmd->_command, cmd->_command_size);

	return 0;
}

extern void wifi_usart_reset(void);

int esp_at_try_tx_command_from_command_q(struct at_context_t *ctx)
{
	//printf("esp_at_try_tx_command_from_command_q\n");
	if (ctx->_line_state == -1) {
		// 删除当前执行的命令
		esp_at_command_dequeue(ctx, &ctx->_command_q, false);

		// 清空所有缓冲区
		wifi_usart_reset();
		ctx->_line_state = 0;
	}


	struct at_command_t *cmd = NULL;
	if (ctx->_command == NULL) {
		if ((cmd = esp_at_command_get(&ctx->_command_q)) != NULL) {
			esp_at_tx_command(ctx, cmd);
			return 0;
		} else {
			return -1;
		}
	} else {
		return -1;
	}
}

int esp_at_try_tx_command(struct at_context_t *ctx, const char *p, int n,
		ESP_AT_COMMAND_RESPONSE_CALLBACK rc,
		ESP_AT_COMMAND_REPORT_CALLBACK report_callback,
		ESP_AT_COMMAND_TIMEOUT_CALLBACK tc,
		uint8_t *extra_data,
		int extra_size)
{
//	printf("esp_at_try_tx_command:@%s@", p);
	struct at_command_t *cmd = (struct at_command_t *)ESP_AT_MALLOC(sizeof(struct at_command_t));
	if (cmd == NULL) {
		return -1;
	}
	cmd->_command = (char *)ESP_AT_MALLOC(n+1);
	if (cmd->_command == NULL) {
		ESP_AT_FREE(cmd);
		return -1;
	}
	memcpy(cmd->_command, p, n);
	cmd->_command[n] = '\0';
	cmd->_command_size = n;
	cmd->response_callback = rc;
	cmd->report_callback = report_callback;
	cmd->timeout_callback = tc;
	cmd->_extra_data = extra_data;
	cmd->_extra_size = extra_size;

	esp_at_command_enqueue(&ctx->_command_q, cmd);


	return esp_at_try_tx_command_from_command_q(ctx);
}

extern void esp_at_udp_recv_callback(const uint8_t *addr, uint16_t port, const uint8_t *p, int n);

void get_ipd_addr_port(struct at_context_t *ctx, int i, uint8_t *addr, uint16_t *port_ptr)
{
	while (ctx->_line[i] != '"') {
		if (ctx->_line[i] == ':') {
			return;
		}
		++i;
	}
	++i;
	addr[0] = atoi((const char *)ctx->_line+i);

	while (ctx->_line[i] != '.') {
		if (ctx->_line[i] == ':') {
			return;
		}
		++i;
	}
	++i;
	addr[1] = atoi((const char *)ctx->_line+i);

	while (ctx->_line[i] != '.') {
		if (ctx->_line[i] == ':') {
			return;
		}
		++i;
	}
	++i;
	addr[2] = atoi((const char *)ctx->_line+i);

	while (ctx->_line[i] != '.') {
		if (ctx->_line[i] == ':') {
			return;
		}
		++i;
	}
	++i;
	addr[3] = atoi((const char *)ctx->_line+i);

	while (ctx->_line[i] != ',') {
		if (ctx->_line[i] == ':') {
			return;
		}
		++i;
	}
	++i;
	*port_ptr = atoi((const char *)ctx->_line+i);

}

int esp_at_on_rx(struct at_context_t *ctx, const uint8_t *p, int n)
{
	uint8_t b;
	for (int i = 0; i < n; ++i) {
		b = p[i];
		switch (ctx->_line_state) {
		case 0:
			{
				ctx->_line_index = 0;
				if (b == '\r') {
					ctx->_line_state = 1;
				} else {
					ctx->_line[ctx->_line_index++] = b;
					ctx->_line_state = 81;
				}
			}
			break;

		case 1:
			{
				if (b == '\n') {
					ctx->_line_state = 2;
				} else {
					printf("esp_at_on_rx:%d\n", __LINE__);
					ctx->_line_state = -1;
				}
			}
			break;

		case 2:
			{
				if (b == '>') {
					// 等待输入
#if	defined(ESP_AT_DEBUG)
					printf("[>]\n");
#endif
					ctx->_line_state = 0;

					if (ctx->_command) {
						struct at_command_t *cmd = ctx->_command;
						if (cmd->_extra_data != NULL && cmd->_extra_size > 0) {
							wifi_usart_tx_bytes(cmd->_extra_data, cmd->_extra_size);
						} else {
							printf("esp_at_on_rx:%d\n", __LINE__);
							ctx->_line_state = -1;
						}
					} else {
						printf("esp_at_on_rx:%d\n", __LINE__);
						ctx->_line_state = -1;
					}

					break;
				} else if (b == '+') {
					ctx->_line[ctx->_line_index++] = b;
					ctx->_line_state = 21;
					break;
				} else {
					// [[fallthrough]]
				}
		case 3:
				if (b == '\r') {
					ctx->_line_state = 4;
				} else {
					if (ctx->_line_index < ESP_AT_LINE_SIZE-1) {
						ctx->_line[ctx->_line_index++] = b;
					} else {
						printf("esp_at_on_rx:%d\n", __LINE__);
						ctx->_line_state = -1;
					}
				}
			}
			break;

		case 4:
			{
				if (b == '\n') {
					ctx->_line[ctx->_line_index] = '\0';
#if	defined(ESP_AT_DEBUG)
					printf("[R]@%s@\n", ctx->_line);
#endif
					if (ctx->_command != NULL) {
						if (ctx->_command->response_callback) {
							ctx->_command->response_callback(ctx);
						}

						//printf("extra_data:%p,extra_size:%d\n", ctx->_command->_extra_data, ctx->_command->_extra_size);
						if (ctx->_command->_extra_data) {
							if (strcmp((const char *)ctx->_line, "SEND OK") == 0
									|| strcmp((const char *)ctx->_line, "SEND FAIL") == 0
									|| strcmp((const char *)ctx->_line, "ERROR") == 0) {
								// FIXME 针对发送数据
								ctx->_command_timestamp = 0;
								esp_at_command_dequeue(ctx, &ctx->_command_q, true);
							}
						} else {
							//if (strcmp((const char *)ctx->_command->_command, "AT\r\n") == 0) {
								// 测试，强制超时！
							//} else {
								ctx->_command_timestamp = 0;
								esp_at_command_dequeue(ctx, &ctx->_command_q, true);
							//}
						}
					}

					ctx->_line_state = 0;
				} else {
					printf("esp_at_on_rx:%d\n", __LINE__);
					ctx->_line_state = -1;
				}
			}
			break;

		case 21:
			if (b == '\r') {
				ctx->_line_state = 4;
			} else {
				if (ctx->_line_index < ESP_AT_LINE_SIZE-1) {
					ctx->_line[ctx->_line_index++] = b;

					if (ctx->_line_index == 5) {
						if (memcmp(ctx->_line, "+IPD,", 5) == 0) {
							// 特殊处理！
							ctx->_line_state = 22;
							// FIXME 只针对+IPD,<length>,...
							// FIXME 未处理+IPD,<link_id>,<length>,...
							ctx->_ipd_length_index = ctx->_line_index;

						} else {
							// 返回正常处理
							ctx->_line_state = 3;
						}
					}
				} else {
					printf("esp_at_on_rx:%d\n", __LINE__);
					ctx->_line_state = -1;
				}
			}
			break;

		case 22:
			{
				if (ctx->_line_index < ESP_AT_LINE_SIZE-1) {
					ctx->_line[ctx->_line_index++] = b;
					// +IPD,<length>,<remote_ip>,<remote_port>:<data>
					if (b == ':') {
						// 准备接收数据
						ctx->_line_state = 23;
						ctx->_ipd_length = atoi((const char *)ctx->_line+ctx->_ipd_length_index);
#if	defined(ESP_AT_DEBUG)
						printf("_ipd_length:%d\n", ctx->_ipd_length);
#endif

						ctx->_ipd_index = 0;
						// 数据偏移
						ctx->_ipd_data_index = ctx->_line_index;
					}
				} else {
					printf("esp_at_on_rx:%d\n", __LINE__);
					ctx->_line_state = -1;
				}
			}
			break;

		case 23:
			{
				if (ctx->_ipd_index < ctx->_ipd_length) {
					if (ctx->_line_index < ESP_AT_LINE_SIZE-1) {
						ctx->_line[ctx->_line_index++] = b;
						++ctx->_ipd_index;
						if (ctx->_ipd_index == ctx->_ipd_length) {
							uint8_t addr[4];
							uint16_t port;

							get_ipd_addr_port(ctx, ctx->_ipd_length_index, addr, &port);
							esp_at_udp_recv_callback(addr, port, ctx->_line+ctx->_ipd_data_index, ctx->_ipd_length);

#if	defined(ESP_AT_DEBUG)
							printf("@%.*s@\n", ctx->_ipd_data_index, ctx->_line);
							printf("data:%d,%d\n", ctx->_ipd_data_index, ctx->_ipd_length);
							for (int i = ctx->_ipd_data_index; i < ctx->_ipd_data_index+ctx->_ipd_length; ++i) {
								printf("%02x ", ctx->_line[i]);
							}
							printf("\n");
#endif
							ctx->_line_state = 24;
						}
					} else {
						printf("esp_at_on_rx:%d\n", __LINE__);
						ctx->_line_state = -1;
					}
				}
			}
			break;

		case 24:
			if (b == '\r') {
				ctx->_line_state = 25;
			} else {
				printf("esp_at_on_rx:%d\n", __LINE__);
				ctx->_line_state = -1;
			}
			break;

		case 25:
			if (b == '\n') {
				// 数据接收完毕！
				ctx->_line_state = 0;
			} else {
				printf("esp_at_on_rx:%d\n", __LINE__);
				ctx->_line_state = -1;
			}
			break;


		case 81:
			{
				if (b == '\r') {
					ctx->_line_state = 82;
				} else {
					if (ctx->_line_index < ESP_AT_LINE_SIZE-1) {
						ctx->_line[ctx->_line_index++] = b;
					} else {
						printf("esp_at_on_rx:%d\n", __LINE__);
						ctx->_line_state = -1;
					}
				}
			}
			break;

		case 82:
			{
				if (b == '\n') {
					ctx->_line[ctx->_line_index] = '\0';
					if (ctx->_command != NULL && ctx->_line_index == ctx->_command->_command_size-2
							&& memcmp(ctx->_line, ctx->_command->_command, ctx->_line_index) == 0) {
#if	defined(ESP_AT_DEBUG)
						printf("[C]@%s@\n", ctx->_line);
#endif
#if	0
						// 测试如何从-1中恢复
						if (strcmp((const char *)ctx->_line, "AT") == 0) {
							ctx->_line_state = -1;
							break;
						}
#endif

						ctx->_command->_command_size = 0;
					} else {
#if	defined(ESP_AT_DEBUG)
						printf("[X]@%s@\n", ctx->_line);
#endif
						ESP_AT_COMMAND_REPORT_CALLBACK callback = at_report_callback_map_get_callback((const char *)ctx->_line);
						if (callback) {
							callback(ctx);
						}
					}
					ctx->_line_state = 0;
				} else {
					printf("esp_at_on_rx:%d\n", __LINE__);
					ctx->_line_state = -1;
				}
			}
			break;

		case -1:
			{
				// 在发送下一个命令前，清空所有缓冲区，重置状态！
			}
			break;


		default:
			break;
		}
	}

	return 0;
}

#include "board.h"

int esp_at_poll(struct at_context_t *ctx, int64_t now)
{
	if (ctx->_command_timestamp > 0) {
		if (now - ctx->_command_timestamp >= 5000) {
			++ctx->_timeout_count;
			printf("timeout:@%s@,%d,%d,%d\n", ctx->_command->_command, (int)now, (int)ctx->_command_timestamp, ctx->_timeout_count);
			ctx->_command_timestamp = 0;

			// 超时
			if (ctx->_command != NULL) {
				if (ctx->_command->timeout_callback) {
					ctx->_command->timeout_callback(ctx);
				}
			}

			// 超时命令超过5次，则重启模块！
			if (ctx->_timeout_count >= 5) {
				ctx->_timeout_count = 0;

				// 重启模块！
				printf("reset esp32...\n");
				wifi_disable();
				wifi_usart_reset();
				DDL_DelayMS(200);
				wifi_enable();


			} else {
				esp_at_command_dequeue(ctx, &ctx->_command_q, true);
			}
		}
	}

	return 0;
}

