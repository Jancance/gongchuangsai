#ifndef SCREEN_H
#define SCREEN_H

#include <stdint.h>

/* 结果1仅表示串口发完，屏幕实际显示仍需目视验收 */
#define SCREEN_OK        1U
#define SCREEN_BAD_PARAM 2U
#define SCREEN_NOT_READY 3U
#define SCREEN_TIMEOUT   4U

extern volatile uint32_t g_screen_init_done;
extern volatile uint32_t g_screen_command_count;

void Screen_Init(void);
uint32_t Screen_SendByte(uint8_t data);
uint32_t Screen_SendString(const char *text);
uint32_t Screen_SendCommand(const char *command);
uint32_t Screen_ShowTaskCode(const char *task_code);

#endif
