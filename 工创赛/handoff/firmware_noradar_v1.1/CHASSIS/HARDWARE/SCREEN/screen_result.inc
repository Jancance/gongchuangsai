uint32_t Screen_ShowResult(uint32_t grabs, uint32_t placements)
{
    char command[48];
    uint32_t result = Screen_SendCommand("page 1");

    if (result != SCREEN_OK)
    {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(20U));
    /* HMI page1: t2/t3 are fixed labels; t0/t1 hold the counts. */
    sprintf(command, "page1.t0.txt=\"%lu\"",
        (unsigned long)grabs);
    result = Screen_SendCommand(command);
    if (result != SCREEN_OK)
    {
        return result;
    }
    sprintf(command, "page1.t1.txt=\"%lu\"",
        (unsigned long)placements);
    return Screen_SendCommand(command);
}
