#include "headfile.h"
#define BEEP_PIN B11
void buzzer(uint8 timems){
    gpio_set(BEEP_PIN,1);
    systick_delay_ms(timems);
    gpio_set(BEEP_PIN,0);
}
void Beep_on()
{
	gpio_set(BEEP_PIN,1);
}

void Beep_off()
{
	gpio_set(BEEP_PIN,0);
}
