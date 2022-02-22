#include<headfile.h>
#define KEY1    C31
#define KEY2    C27
#define KEY3    C26
#define KEY4    C4
//开关状态变量
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

//上一次开关状态变量
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
uint8 key4_last_status;

//开关标志位
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;

uint8 level=1,selFirst=0,selSecond=0;

//RED BLUE YELLOW GREEN WHITE BLACK GRAY BROWN PURPLE PINK

void showFirstMenu(char sel);
void showSecondMenu(char sel);

void initMenu(){
    gpio_init(KEY1,GPI,0,GPIO_PIN_CONFIG);
    gpio_init(KEY2,GPI,0,GPIO_PIN_CONFIG);
    gpio_init(KEY3,GPI,0,GPIO_PIN_CONFIG);
    gpio_init(KEY4,GPI,0,GPIO_PIN_CONFIG);
}

void readKey(){
        //使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费
        //保存按键状态
        key1_last_status = key1_status;
        key2_last_status = key2_status;
        key3_last_status = key3_status;
        key4_last_status = key4_status;
        //读取当前按键状态
        key1_status = gpio_get(KEY1);
        key2_status = gpio_get(KEY2);
        key3_status = gpio_get(KEY3);
        key4_status = gpio_get(KEY4);
        
        //检测到按键按下之后  并放开置位标志位
        if(key1_status && !key1_last_status)    key1_flag = 1;
        if(key2_status && !key2_last_status)    key2_flag = 1;
        if(key3_status && !key3_last_status)    key3_flag = 1;
        if(key4_status && !key4_last_status)    key4_flag = 1;
        
        //标志位置位之后，可以使用标志位执行自己想要做的事件
        if(key1_flag)   
        {
            key1_flag = 0;//使用按键之后，应该清除标志位
            if(level==1){
                if(selFirst>0&&selFirst<=3){
                    selFirst-=1;
                }
                showFirstMenu(selFirst);
            }
            if(level==2){
                if(selSecond>0&&selSecond<=3){
                    selSecond-=1;
                }
                showSecondMenu(selSecond);
            }

        }
        
        if(key2_flag)   
        {
            key2_flag = 0;//使用按键之后，应该清除标志位
            if(level==1){
                if(selFirst>=0&&selFirst<3){
                    selFirst+=1;
                }
                showFirstMenu(selFirst);
            }
            if(level==2){
                if(selSecond>=0&&selSecond<3){
                    selSecond+=1;
                }
                showSecondMenu(selSecond);
            }
        }
        
        if(key3_flag)   
        {
            key3_flag = 0;//使用按键之后，应该清除标志位
            if(level==1){
                lcd_clear(WHITE);
                showSecondMenu(selSecond);
            }
            if(level==2){
                switch(selSecond){
                    case 3:{
                        lcd_clear(WHITE);
                        showFirstMenu(selFirst);
                        break;
                    }
                    default:{
                        if(selFirst==2){
                            lcd_clear(WHITE);
                            showFirstMenu(selFirst);
                            break;
                        }
                        lcd_clear(WHITE);
                        showSecondMenu(selSecond);
                        break;
                    }
                }
                    
            }
            
        }
        
        if(key4_flag)   
        {
            key4_flag = 0;//使用按键之后，应该清除标志位

        }
}

void showFirstMenu(char sel){
    level=1;
    while (1){
        readKey();
        lcd_showstrColor(1, 0, "Index",PURPLE);
        lcd_showstrColor(1, 2, "1. Run",sel==0?BLUE:RED);
        lcd_showstrColor(1, 3, "2. Settings",sel==1?BLUE:RED);
        lcd_showstrColor(1, 4, "3. Status",sel==2?BLUE:RED);
        lcd_showstrColor(1, 5, "4. Run with LCD Off",sel==3?BLUE:RED);
    }
}

void showSecondMenu(char sel){
    level=2;
    while (1){
        readKey();
        switch (selFirst){
            case 1:{
                lcd_showstrColor(1, 0, "Settings",PURPLE);
                lcd_showstrColor(1, 2, "1. PID Arguments",sel==0?BLUE:RED);
                lcd_showstrColor(1, 3, "2. Image Adjust",sel==1?BLUE:RED);
                lcd_showstrColor(1, 4, "3. Undefined",sel==2?BLUE:RED);
                lcd_showstrColor(1, 5, "4. Back to Index",sel==3?BLUE:RED);
                break;
            }
            case 2:{
                lcd_showstrColor(1, 0, "Status",PURPLE);
                lcd_showstrColor(1, 2, "PID Arguments",BLUE);
                lcd_showstrColor(1, 3, "P:11 I:45 D:14 ",RED);
                lcd_showstrColor(1, 4, "LCD Offset",BLUE);
                lcd_showstrColor(1, 5, "L:0 R:0",RED);
                lcd_showstrColor(1, 6, "Back to Index",PURPLE);
                break;
            }
            default:{
                showFirstMenu(selFirst);
            }
        }
    }
}









