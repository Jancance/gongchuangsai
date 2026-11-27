uint32_t Screen_ShowResult(uint32_t grabs, uint32_t placements)
{
    char command[48];
    uint32_t result = Screen_SendCommand("page 1");

    if (result != SCREEN_OK)
    {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(20U));
    /* Existing HMI font uses GB2312. */
    sprintf(command, "page1.t2.txt=\"\xB3\xC9\xB9\xA6\xD7\xA5\xC8\xA1\xB4\xCE\xCA\xFD:%lu\"",
        (unsigned long)grabs);
    result = Screen_SendCommand(command);
    if (result != SCREEN_OK)
    {
        return result;
    }
    sprintf(command, "page1.t3.txt=\"\xB3\xC9\xB9\xA6\xB7\xC5\xD6\xC3\xB4\xCE\xCA\xFD:%lu\"",
        (unsigned long)placements);
    return Screen_SendCommand(command);
}
