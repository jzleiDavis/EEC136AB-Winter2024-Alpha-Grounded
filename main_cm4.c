/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "stdio.h" //UART functions
#include "math.h"//math functions
#include "stdlib.h"

#define humiditySensor 0x40     //0x40 connected to GND, 0x41 connected to VDD
#define HTU31D_READTEMPHUM 0x00
#define HTU31D_CONVERSION 0x40

uint8 humidity[100];
uint8 temperature[100];
uint8 outputValues[200];
uint16 vOut[100];
int i;
int k = 0;

static cy_stc_scb_i2c_master_xfer_config_t register_setting; //struct

static uint8 rbuff[5]; // Read buffer
static uint8 wbuff[2]; // Write buffer

static void WaitForOperation()     // function that check to make sure either a write or read function has completed

{
    while(0 != (SensorBus_MasterGetStatus() & CY_SCB_I2C_MASTER_BUSY)) {}
    {
        CyDelayUs (1);       // Keep Waiting
    }
}

static void WriteRegister(uint8 reg_addr, uint8 data)
{
    wbuff[0] = reg_addr; // Asign the first element to be the register you want to writte to (Parameter 1) "register address"
    wbuff[1] = data;     // Assign the second elemeent to be the value you wish to write to the register (Parameter 2)
    
    register_setting.buffer = wbuff;
    register_setting.bufferSize = 2;
    register_setting.xferPending = false;
    
    
    SensorBus_MasterWrite(&register_setting);
    WaitForOperation();
}

void PressedHandler()
{ 
    NVIC_ClearPendingIRQ(Pressed_IRQ_cfg.intrSrc);
    UART_PutArray(temperature, 64);   
    CyDelay(100);
    UART_PutArray(humidity, 64);
    CyDelay(100);
    UART_PutArray(vOut, 128);  
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    UART_Start();
    SensorBus_Start();
    
    Cy_SysInt_Init(&Pressed_IRQ_cfg, PressedHandler);
    NVIC_EnableIRQ(Pressed_IRQ_cfg.intrSrc);
    //ADC init--------------------
    setvbuf(stdin,NULL,_IONBF,0);
    ADC_Start();
    
    float voltages[3];
    uint16_t on_state;
    int count = 1;
    
    Cy_GPIO_Write(WS_PORT,WS_NUM,0);
    Cy_GPIO_Write(MW_PORT,MW_NUM,0);
    
    CyDelay(500);
    //ADC init end-----------------

    //HTU31 init-------------------
    uint16_t humidityRaw;
    uint16_t temperatureRaw;
    uint64 sensorRaw;
    uint16_t i = 0;
    register_setting.slaveAddress = humiditySensor;
    //HTU31 init end---------------

    for(;;)
    {
    //ADD HIGH POWER MODE    
    Cy_SysPm_ExitLowPowerMode;
    //Indexing START--------------------
        k++;
        if (k == 100)
        {
            k = 0;
        }
    //Indexing END----------------------    
        
    //ADC Readings START-----------------------
        CyDelay(500);
        Cy_GPIO_Write(WS_PORT,WS_NUM,1);
        Cy_GPIO_Write(MW_PORT,MW_NUM,1);
        CyDelay(1700);
        for (uint32_t channel = 1; channel < 3; channel++)
        {
            Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_SINGLE_SHOT);
            CyDelay(500);
            volatile float value = Cy_SAR_GetResult16(SAR, channel);
            CyDelay(500);
            volatile float volts = Cy_SAR_CountsTo_mVolts(SAR, channel, value);
            CyDelay(500);
            voltages[channel] = volts;
            CyDelay(500);
        }
        Cy_GPIO_Write(WS_PORT,WS_NUM,0);
        Cy_GPIO_Write(MW_PORT,MW_NUM,0);
        Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_SINGLE_SHOT);
        CyDelay(500);
        volatile float value = Cy_SAR_GetResult16(SAR, 0);
        CyDelay(500);
        volatile float volts = Cy_SAR_CountsTo_mVolts(SAR, 0, value);
        CyDelay(500);
        voltages[0] = volts;
        CyDelay(500);	  
    //ADC Reading END---------------
    
    //Sensor Reading START----------
        WriteRegister(HTU31D_CONVERSION,0x01);
        CyDelay(5);
        wbuff[0] = HTU31D_READTEMPHUM;  // Buffer that will contain the register that will be read from
    
        register_setting.buffer = wbuff;
        register_setting.bufferSize = 1;
        register_setting.xferPending = true;

        SensorBus_MasterWrite(&register_setting);
        WaitForOperation();

        register_setting.buffer = rbuff;    // Buffer that will store the value read from the register
        register_setting.bufferSize = 5;
        register_setting.xferPending = false;

        SensorBus_MasterRead(&register_setting);
        WaitForOperation();
        
        temperatureRaw = (rbuff[0] << 8) | rbuff[1];
        humidityRaw = (rbuff[3] << 8) | rbuff[4];    
        
        float actualHumidity = 100.0 * (humidityRaw/(pow(2,16)-1));
        float actualTemperature = -40 + 165 * (temperatureRaw / (pow(2,16)-1));
    //Sensor Reading END------------
        
    //Output Values
    vOut[k] = (int16_t) voltages[0];
    humidity[k] = (uint8) actualHumidity;    
    temperature[k] =  (uint8) actualTemperature;
    //ADD LOW POWER MODE AND DELAY 
    Cy_SysPm_EnterLowPowerMode;
    CyDelay(10000); //Variable Delay Time// 600000 = 10 min // 10800000 = 3 hours
    }
}

/* [] END OF FILE */
