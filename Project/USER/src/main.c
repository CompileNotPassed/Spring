#include "headfile.h"
#include <lib/function.h>
int i, j, cnt = 0;
unsigned char frame[128][160];
int16 horLine[2][128];
int16 verLine[160][2];

uint8 uart_flag = 0;
uint8 uart_data = 0;

extern int16 encoder[4];
extern pid_t Motor[4];
extern float speed[4];

// TESTUSE
float setTarget[4] = {500, 300, 300, 300};

int main(void)
{
    DisableGlobalIRQ();

    board_init();
    systick_delay_ms(300);

    bspInit();
    // Position_PID_Init(&Motor[0], 110, 0.1, 0, 30000, 30000);
    // Position_PID_Init(&Motor[1], 110, 0.1, 0, 30000, 30000);
    // Position_PID_Init(&Motor[2], 110, 0.1, 0, 30000, 30000);
    // Position_PID_Init(&Motor[3], 110, 0.1, 0, 30000, 30000);
    Position_PID_Init(&Motor[0], 10, 0.1, 0, 30000, 30000);
    Position_PID_Init(&Motor[1], 10, 0.1, 0, 30000, 30000);
    Position_PID_Init(&Motor[2], 30, 0.1, 0, 30000, 30000);
    Position_PID_Init(&Motor[3], 10, 0.1, 0, 30000, 30000);
    wirelessInit();

    lcd_init();
    lcd_showstr(0, 0, "Initializing...");

    initMenu();
    mt9v03x_csi_init();

    lcd_showstr(0, 1, "Init OK.");
    systick_delay_ms(500);

    EnableGlobalIRQ(0);
    lcd_clear(WHITE);
    // buzzer(200);
    while (1)
    {

        while (!mt9v03x_csi_finish_flag)
        {
        }
        // speed[0]=Position_PID_Implementation(&Motor[0],2000,encoder[0]);
        // MotorOutput(Motor,speed);
        //MotorOutput(Motor, setTarget);
        // EnableMotor(15000,0);
        // ImageProcess
        getBin(mt9v03x_csi_image, frame, 2);
        // sharpenImage(frame);
        imgFilter(frame, 2);
        // getBin(frame, frame, 2);
        horizontalDetect(frame);
        // verticalDetect(frame, verLine);
        // sendImage(frame[0],160*128);
        // lcdOutput(frame[0], 160, 128, 160, 128);
        sendWare(encoder, sizeof(encoder));
        // sendWare(speed,sizeof(speed));
        // printEncoder();
        //printSpeed();
    }
}
