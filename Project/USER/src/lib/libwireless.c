#include "fsl_common.h"
#include "fsl_gpio.h"
#include "zf_systick.h"
#include "zf_gpio.h"
#include "zf_iomuxc.h"
#include "zf_uart.h"
#include "SEEKFREE_WIRELESS.h"

void wirelessInit(void)
{
    seekfree_wireless_init();    
}

uint32 sendBuff(uint8 *buff, uint32 len)
{
    while(len>30)
    {
        if(gpio_get(RTS_PIN))  
        {
            return len;
        }
        uart_putbuff(WIRELESS_UART,buff,30);

        buff += 30; 
        len -= 30;
    }
    
    if(gpio_get(RTS_PIN))  
    {
    }
    uart_putbuff(WIRELESS_UART,buff,len);
    
    return 0;
}

void sendWare(void *wareaddr, uint32 waresize)
{
#define VCAN_PORT USART_8
#define CMD_WARE     3

    int16 i, tem;
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};
    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));
    uart_putbuff(VCAN_PORT, (uint8 *) wareaddr, waresize);
    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));
#undef CMD_WARE
}

void sendImage(void *wareaddr, uint32 waresize)
{
#define VCAN_PORT USART_8
#define CMD_WARE     1

    int16 i, tem;
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};
    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));
    uart_putbuff(VCAN_PORT, (uint8 *) wareaddr, waresize);
    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));
#undef CMD_WARE
}