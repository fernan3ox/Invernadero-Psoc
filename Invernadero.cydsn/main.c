#include "project.h"
#include <stdio.h>

int32 entero, voltaje;
float32 temp;
char str[1];
char datoRX;
CY_ISR(InterruptRX)
{
    datoRX=HS05_GetChar();
    if(datoRX=='a')
    {
        MotorA_Write(0);
        MotorB_Write(1);
            
    }
    if(datoRX=='b')
    {
        MotorA_Write(1);
        MotorB_Write(0);
       
    }
}
int main(void)
{
CyGlobalIntEnable;
Opamp_Start();
ADC_Start();
HS05_Start();
isrRX_StartEx(InterruptRX);
PWM_Start();
for(;;)
{
    ADC_StartConvert();
    ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
    entero=ADC_GetResult32();
    ADC_StopConvert();
    voltaje=ADC_CountsTo_mVolts(entero);
    temp=((voltaje/10.00)-3);
    sprintf(str, "%.1f", temp);
    HS05_PutString(str);
    if(temp>35)
        {
            Cooler1_Write(1);
            Cooler2_Write(0);
            LED_Write(1);
        }
    if(temp<=35)
        {
            Cooler1_Write(0);
            Cooler2_Write(0);
            LED_Write(0);
        }
    }
}