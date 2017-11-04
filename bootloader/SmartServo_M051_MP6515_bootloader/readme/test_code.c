    FMC_EnableAPUpdate();
    if(FlashTest(AP_START_ADDRESS, AP_START_ADDRESS+FMC_FLASH_PAGE_SIZE, (uint32_t*)file_unpack_buffer) < 0)
    {
        printf("\n\nAPROM test failed!\n");
        // goto lexit;
    }
    else
    {
        printf("\n\nAPROM test successfully!\n");
    }
    FMC_DisableAPUpdate();

    printf("ldrom\r\n");
    UART_WAIT_TX_EMPTY(UART0);
    do
    {

       while(Uart0Revhead == Uart0RevRtail);

        {
           u8Item = Uart0RecData[Uart0Revhead];
           Uart0Revhead = (Uart0Revhead == (uint16_t)(UART0_REV_BUF_SIZE - 1)) ? 0 : (Uart0Revhead + 1);
        }

        switch(u8Item)
        {
        case '0':
            printf("seleted 0!\r\n");
            UART_WAIT_TX_EMPTY(UART0);
            goto _ISP;

        case '1':
            printf("seleted 1!\r\n");
            UART_WAIT_TX_EMPTY(UART0);
            while(1)
            {
               goto _APROM;
            }

            //break;
        default :
            break;
        }
    }
    while(1);


int8_t  FlashTest(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t* buffer)
{
    uint32_t    u32Addr;
    //uint32_t    u32Data_temp;

    FMC_EnableAPUpdate();
    for(u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        // printf("    Flash test address: 0x%x    \r", u32Addr);

        // Erase page
        FMC_Erase(u32Addr);

        // Verify if page contents are all 0xFFFFFFFF
        // for(u32Addr = u32StartAddr; u32Addr < u32Addr+FMC_FLASH_PAGE_SIZE; u32Addr += 4)
        // {
        //     u32Data_temp = FMC_Read(u32Addr);
        //     if(u32Data_temp != 0xFF)
        //     {
        //         // printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32Data_temp, u32Data);
        //         return -1;
        //     }
        // }

        // Write test pattern to fill the whole page
        if(WriteData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, buffer) < 0)//write a page
        {
            // printf("Failed to write page 0x%x!\n", u32Addr);
            return -1;
        }

        // Verify if page contents are all equal to test pattern
        // if(VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        // {
        //     // printf("\nData verify failed!\n ");
        //     return -1;
        // }

        // FMC_Erase(u32Addr);

        // // Verify if page contents are all 0xFFFFFFFF
        // if(VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        // {
        //     // printf("\nPage 0x%x erase verify failed!\n", u32Addr);
        //     return -1;
        // }
    }
    FMC_DisableAPUpdate();
    return 0;
}
