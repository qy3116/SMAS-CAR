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
    Data = Reflectance_Read(500);
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
        { Motor_Right(L,R);Clock_Delay1ms(600);}//右转弯
        else if (b[0]==0&&b[7]==1)
        { Motor_Left(L,R);Clock_Delay1ms(600);}//左转弯
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
          Motor_Forward(4000,4000);
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
