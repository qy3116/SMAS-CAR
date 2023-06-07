#include "Buzzer.h"
#include "msp.h"
#include <stdio.h>
#include "Clock.h"
#include "Motor.h"
#include "PWM.h"
#include "UART0.h"
#include "LCD.h"
#include "Reflectance.h"

/**
 * main.c
 */
uint8_t Data;
int32_t position;
int L=4000;
int R=4000;
int b[8];int i,sum_b=0;

void reflect(void);

void main(void)
{

    UART0_Init();
    Clock_Init48MHz();
    Reflectance_Init();
    Motor_Init();
    Port5_Init();
    LCD_Init();

    while(1)
    {
        LCD_Show();
        reflect();
        /*
        int flag=Reflectance_Center(1000);
        switch(flag)
        {
            case 0: Motor_Stop();break; // off road
            case 1: Motor_Left(L,R);break;//off to right  position>0
            case 2: Motor_Right(L,R);break;//off to left  position<0
            case 3: Motor_Forward(L,R);break; // on road
        }*/
    }
}

void reflect(void)
{
    Data = Reflectance_Read(400);
    position = Reflectance_Position(Data);
    sum_b=0;

    for(i=0;i<8;i++)
          {
            b[i] = (~Data>>i)&0x01;
            sum_b += b[i];
          }
if(sum_b>=4){
        if(b[0]==1&&b[7]==1)
            {Motor_Stop();}
        else if(b[0]==1&&b[7]==0)
        {Port5_Output(1); Motor_Right(L,R);Clock_Delay1ms(600);}//右转弯
        else if (b[0]==0&&b[7]==1)
        { Port5_Output(1);Motor_Left(L,R);Clock_Delay1ms(600);}//左转弯
                    }
else{
    if(position==666)
    {
        Motor_Stop();
    }
    else
    {
      //直线行走
      if(position==0)
      {
          Motor_Forward(L,R);
      }
      else if(position>=237)
      {
          Motor_Forward(7000,2000);
      }
      else if(position<=-237)
      {
          Motor_Forward(2000,7000);
      }
      else if(position>=0 && position<237)
      {
          Motor_Forward(3700,3000-8*position);
      }
      else if(position<=0 && position>-237)
      {
          Motor_Forward(3000+8*position,3700);
      }
    }
    }
}





/*
//直角弯右转
void T_Right(void)
{
    uint8_t d;
    d=Reflectance_Read(1000);
    if(d==0xe0 || d==0xc0 || d==0xf0 || d==0xf8)
    {
        Motor_Right(9000,9000);
    }
}*/




/*while(1)
{
   // # 读取红外传感器的数据
 float   sensor_data =  Reflectance_Position(Reflectance_Read(500));

   // # 计算当前位置偏差
 float   error = target_position - sensor_data;//相对白线位置

  //  # 计算PID控制量
 float   proportional = Kp * error;
    integral += Ki * error;
 float   derivative = Kd * (error - previous_error);

   // # 计算转向控制量
 float   control_signal = proportional + integral + derivative;

    //# 控制小车转向
    if (control_signal > 0)
    Motor_Left(L,R);
    if (control_signal < 0)
    Motor_Right(L,R);
    else
    Motor_Forward(L,R);

    //# 更新上一次的偏差值
    previous_error = error;
}*/
/*
    //*float PID_Output(void);
    float kp=0,ki=0,kd=100;  //PID更改
    float PID_Output(void)
    {
    int position = Reflectance_Position(Reflectance_Read(50));
    float error,last_error=0;    //此次和上次的误差
    static float integral;    //积分累加项
    float output;      //PID输出
    if(position<0&&position>-64)    //
     error = -1;
    else if(position<-64&&position>-100)   //
     error = -2;
    else if(position<-100)   //
     error = -3;
    else if(position>0&&position<64)   //
     error = 1;
    else if(position>64&&position<100)   //
     error = 2;
    else if(position>100)   //
     error = 3;
    else
     error = 0;
    integral += error;
    output = kp * error + ki * integral + kd * (error - last_error);
    last_error = error;
    return output;
    }
*/
