#include "init.h"

void system_init(void)
{
    lcd_init();
    my_wifi_spi_init();  
    key_init(10);
    pit_ms_init(CCU60_CH0, 10);
    pit_ms_init(CCU60_CH1, 1000);
}
