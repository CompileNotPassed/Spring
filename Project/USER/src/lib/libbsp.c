#include "common.h"
#include "headfile.h"
#include "zf_pwm.h"
#include "zf_qtimer.h"
#include "zf_pit.h"
#include "zf_gpio.h"
#include "MIMXRT1064.h"
void bspInit()
{
    gpio_init(B11, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(D0, GPO, 1, GPIO_PIN_CONFIG);  //leftbot
    gpio_init(D1, GPO, 0, GPIO_PIN_CONFIG);  //rightbot
    gpio_init(D14, GPO, 0, GPIO_PIN_CONFIG); //lefttop
    gpio_init(D15, GPO, 0, GPIO_PIN_CONFIG); //righttop

    pwm_init(PWM2_MODULE3_CHB_D3, 17000, 0);  //  0  rightbot
    pwm_init(PWM2_MODULE3_CHA_D2, 17000, 0);  // 1  leftbot
    pwm_init(PWM1_MODULE0_CHB_D13, 17000, 0); //  0  righttop
    pwm_init(PWM1_MODULE0_CHA_D12, 17000, 0); // 0  lefttop

    qtimer_quad_init(QTIMER_1, QTIMER1_TIMER0_C0, QTIMER1_TIMER1_C1); 
    qtimer_quad_init(QTIMER_1, QTIMER1_TIMER2_C2, QTIMER1_TIMER3_C24);
    qtimer_quad_init(QTIMER_2, QTIMER2_TIMER0_C3, QTIMER2_TIMER3_C25);
    qtimer_quad_init(QTIMER_3, QTIMER3_TIMER2_B18, QTIMER3_TIMER3_B19);

    pit_init();                    //³õÊ¼»¯pitÍâÉè
    pit_interrupt_ms(PIT_CH0, 10); //³õÊ¼»¯pitÍ¨µÀ0 ÖÜÆÚ
    NVIC_SetPriority(PIT_IRQn, 0);
}