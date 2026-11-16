uint32_t xTaskGetTickCount(void);
void TestEnter(void);void TestExit(void);
#define taskENTER_CRITICAL() TestEnter()
#define taskEXIT_CRITICAL() TestExit()
