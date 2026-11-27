#ifndef BOOT_READY_LINK_H
#define BOOT_READY_LINK_H
#include <stdint.h>
extern volatile uint32_t g_boot_main_init_done,g_boot_app_init_done,g_boot_app_selftests_skipped;
void BootReady_LinkPoll(uint32_t now);
uint32_t BootReady_SharedByte(uint8_t byte,uint32_t now,uint32_t binary_active);
void BootReady_BeepStep(void);
#endif
