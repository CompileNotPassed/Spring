#include <headfile.h>

#define L 'L'
#define R 'R'
//ImageProcess
void getBin(uint8 (*in)[160], uint8 out[][160], char mode);
void imgFilter(uint8 img[128][160], uint8 mode);
void sharpenImage(uint8 img[128][160]);
//BorderDetect
void horizontalDetect(uint8 img[128][160]);
void verticalDetect(uint8 img[128][160], int16 verLine[160][2], uint8 mode);
//Filter

//Calc
float linearFitting(uint8 line[MT9V03X_CSI_H], int16 lineFitted[MT9V03X_CSI_H]);

//Element Recognize

//Menu
void initMenu(void);
void readKey(void);
void showFirstMenu(char sel);

//PID
typedef struct
{
	int SetValue;
	int ActualValue;
	int err;
	int err_last;
	int err_last_last;
	float Kp, Ki, Kd;
	float result;
	float voltage;
	int integral;
	int maxintegral;
	int integrallimit;
} pid_t;

void Position_PID_Init(pid_t *PID_Motor, float Kp, float Ki, float Kd, int maxintegral, int integrallimit);
float Position_PID_Realize(pid_t *PID_Motor, float Target, float Input);
float Position_PID_Implementation(pid_t *PID_Motor, float Target, float Input);
float PID_Implementation(pid_t *PID_Motor, float Target, float Input);

//Motor
void MotorOutput(pid_t *Mot, float *Target);
void EnableMotor(float speed, uint8 num);
void printSpeed(void);

//Move
void mecmove(uint8 vx, uint8 vy, uint8 va);

//Wireless
void wirelessInit(void);
uint32 sendBuff(uint8 *buff, uint32 len);
void sendWare(void *wareaddr, uint32 waresize);
void sendImage(void *wareaddr, uint32 waresize);

//BSP
void bspInit(void);

//Encoder
void getEncoder(void);
void printEncoder(void);

//Buzzer
void buzzer(uint8 timems);
void Beep_on(void);
void Beep_off(void);
