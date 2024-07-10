/*
 * Copyright (c) 2020 Nanjing Xiaoxiongpai Intelligent Technology Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "ohos_init.h"
#include "cmsis_os2.h"

#include "E53_ST1.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_pwm.h"
#include "wifiiot_uart.h"

#include "wifiiot_i2c.h"
#include "wifiiot_i2c_ex.h"

gps_msg             gpsmsg;
static unsigned char gps_uart[1000];

/***************************************************************
* 函数名称: SHT30_reset
* 说    明: SHT30复位
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void SHT30_reset(void)
{
    WifiIotI2cData sht30_i2c_data = {0};
    uint8_t send_data[2] = {0x30, 0xA2};
    sht30_i2c_data.sendBuf = send_data;
    sht30_i2c_data.sendLen = 2;
    I2cWrite(WIFI_IOT_I2C_IDX_1, (SHT30_Addr << 1) | 0x00, &sht30_i2c_data);
}

/***************************************************************
* 函数名称: Init_SHT30
* 说    明: 初始化SHT30，设置测量周期
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void Init_SHT30(void)
{
    WifiIotI2cData sht30_i2c_data = {0};
    uint8_t send_data[2] = {0x22, 0x36};
    sht30_i2c_data.sendBuf = send_data;
    sht30_i2c_data.sendLen = 2;
    I2cWrite(WIFI_IOT_I2C_IDX_1, (SHT30_Addr << 1) | 0x00, &sht30_i2c_data);
}

/***************************************************************
* 函数名称: SHT3x_CheckCrc
* 说    明: 检查数据正确性
* 参    数: data：读取到的数据
						nbrOfBytes：需要校验的数量
						checksum：读取到的校对比验值
* 返 回 值: 校验结果，0-成功		1-失败
***************************************************************/
static uint8_t SHT3x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{

    uint8_t crc = 0xFF;
    uint8_t bit = 0;
    uint8_t byteCtr;
    const int16_t POLYNOMIAL = 0x131;
    //calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
    {
        crc ^= (data[byteCtr]);
        for (bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }

    if (crc != checksum)
        return 1;
    else
        return 0;
}

/***************************************************************
* 函数名称: SHT3x_CalcTemperatureC
* 说    明: 温度计算
* 参    数: u16sT：读取到的温度原始数据
* 返 回 值: 计算后的温度数据
***************************************************************/
static float SHT3x_CalcTemperatureC(uint16_t u16sT)
{

    float temperatureC = 0;

    u16sT &= ~0x0003;

    temperatureC = (175 * (float)u16sT / 65535 - 45);

    return temperatureC;
}

/***************************************************************
* 函数名称: SHT3x_CalcRH
* 说    明: 湿度计算
* 参    数: u16sRH：读取到的湿度原始数据
* 返 回 值: 计算后的湿度数据
***************************************************************/
static float SHT3x_CalcRH(uint16_t u16sRH)
{

    float humidityRH = 0;

    u16sRH &= ~0x0003;

    humidityRH = (100 * (float)u16sRH / 65535);

    return humidityRH;
}

/***************************************************************
* 函数名称: GPS_Init
* 说    明: 初始化GPS模块
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void GPS_Init(void)
{

    uint32_t ret;

    WifiIotUartAttribute uart_attr = {

        //baud_rate: 9600
        .baudRate = 9600,

        //data_bits: 8bits
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    //Initialize uart driver
    ret = UartInit(WIFI_IOT_UART_IDX_1, &uart_attr, NULL);
    if (ret != WIFI_IOT_SUCCESS)
    {
        printf("Failed to init uart! Err code = %d\n", ret);
        return;
    }
    
}

/***************************************************************
* 函数名称: Init_E53_ST1
* 说    明: 初始化E53_ST1扩展板
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void Init_E53_ST1(void)
{
    GpioInit();                                                           //初始化GPIO
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_PWM1_OUT); //设置GPIO_8引脚复用功能为PWM
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_GPIO_DIR_OUT);           //设置GPIO_8引脚为输出模式
    PwmInit(WIFI_IOT_PWM_PORT_PWM1);                                      //初始化PWM1端口

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_4, WIFI_IOT_IO_FUNC_GPIO_4_GPIO);     //设置GPIO_4引脚复用功能为GPIO
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_4, WIFI_IOT_GPIO_DIR_OUT);           //设置GPIO_4引脚为输出模式
    GpioSetOutputVal(WIFI_IOT_IO_NAME_GPIO_4, 0);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);     //设置GPIO_2引脚复用功能为GPIO
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_GPIO_DIR_OUT);           //设置GPIO_2引脚为输出模式
    GpioSetOutputVal(WIFI_IOT_IO_NAME_GPIO_2, 0);

	//GPIO_0复用为I2C1_SDA
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_I2C1_SDA);

    //GPIO_1复用为I2C1_SCL
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_I2C1_SCL);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_9, WIFI_IOT_IO_FUNC_GPIO_9_GPIO);//设置GPIO_9为输出模式

     GpioSetDir(WIFI_IOT_GPIO_IDX_9, WIFI_IOT_GPIO_DIR_OUT);

	GPS_Init();
	I2cInit(WIFI_IOT_I2C_IDX_1, 400000);
    I2cSetBaudrate(WIFI_IOT_I2C_IDX_1, 400000);
	Init_SHT30();
}

/***************************************************\
* 函数名称: NMEA_Comma_Pos
*	函数功能：从buf里面得到第cx个逗号所在的位置
*	输入值：
*	返回值：0~0xFE，代表逗号所在位置的便宜
*				 	0xFF，代表不存在第cx个逗号
\***************************************************/

uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{
	uint8_t *p = buf;
	while(cx)
	{
		if(*buf=='*'||*buf<' '||*buf>'z')return 0xFF;
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;
}
/***************************************************\
* 函数名称: NMEA_Pow
*	函数功能：返回m的n次方值
*	输入值：底数m和指数n
*	返回值：m^n
\***************************************************/
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result = 1;
	while(n--)result *= m;
	return result;
}
/***************************************************\
* 函数名称: NMEA_Str2num
*	函数功能：str数字转换为（int）数字，以','或者'*'结束
*	输入值：buf，数字存储区
*				 	dx，小数点位数，返回给调用函数
*	返回值：转换后的数值
\***************************************************/
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p = buf;
	uint32_t ires = 0,fres = 0;
	uint8_t ilen = 0,flen = 0,i;
	uint8_t mask = 0;
	int res;
	while(1)
	{
		if(*p=='-'){mask |= 0x02;p++;}//说明有负数
		if(*p==','||*p=='*')break;//遇到结束符
		if(*p=='.'){mask |= 0x01;p++;}//遇到小数点
		else if(*p>'9'||(*p<'0'))//数字不在0和9之内，说明有非法字符
		{
			ilen = 0;
			flen = 0;
			break;
		}
		if(mask&0x01)flen++;//小数点的位数
		else ilen++;//str长度加一
		p++;//下一个字符
	}
	if(mask&0x02)buf++;//移到下一位，除去负号
	for(i=0;i<ilen;i++)//得到整数部分数据
	{
		ires += NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;//最多取五位小数
	*dx = flen;
	for(i=0;i<flen;i++)//得到小数部分数据
	{
		fres +=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	}
	res = ires*NMEA_Pow(10,flen)+fres;
	if(mask&0x02)res = -res;
	return res;
}
/***************************************************\
* 函数名称: NMEA_BDS_GPRMC_Analysis
*	函数功能：解析GPRMC信息
*	输入值：gpsx,NMEA信息结构体
*				 buf：接收到的GPS数据缓冲区首地址
\***************************************************/
void NMEA_BDS_GPRMC_Analysis(gps_msg *gpsmsg,uint8_t *buf)
{
	uint8_t *p4,dx;			 
	uint8_t posx;     
	uint32_t temp;	   
	float rs;  
	p4=(uint8_t*)strstr((const char *)buf,"$GPRMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
	if(p4 != NULL){
		posx=NMEA_Comma_Pos(p4,3);								//得到纬度
		if(posx!=0XFF)
		{
			temp=NMEA_Str2num(p4+posx,&dx);		 	 
			gpsmsg->latitude_bd=temp/NMEA_Pow(10,dx+2);	//得到°
			rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
			gpsmsg->latitude_bd=gpsmsg->latitude_bd*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
		}
		posx=NMEA_Comma_Pos(p4,4);								//南纬还是北纬 
		if(posx!=0XFF)gpsmsg->nshemi_bd=*(p4+posx);					 
		posx=NMEA_Comma_Pos(p4,5);								//得到经度
		if(posx!=0XFF)
		{												  
			temp=NMEA_Str2num(p4+posx,&dx);		 	 
			gpsmsg->longitude_bd=temp/NMEA_Pow(10,dx+2);	//得到°
			rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
			gpsmsg->longitude_bd=gpsmsg->longitude_bd*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
		}
		posx=NMEA_Comma_Pos(p4,6);								//东经还是西经
		if(posx!=0XFF)gpsmsg->ewhemi_bd=*(p4+posx);	
	}
	  
}

/***************************************************************
* 函数名称: E53_ST1_Read_Data
* 说    明: 获取GPS经纬度信息
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void E53_ST1_Read_Data(E53_ST1_Data_TypeDef *ReadData)
{	
    //通过串口1接收数据
    UartRead(WIFI_IOT_UART_IDX_1, gps_uart, 1000);
	NMEA_BDS_GPRMC_Analysis(&gpsmsg,(uint8_t*)gps_uart);	//分析字符串
	ReadData->Longitude=(float)((float)gpsmsg.longitude_bd/100000);	
	ReadData->Latitude=(float)((float)gpsmsg.latitude_bd/100000);   
}

/***************************************************************
* 函数名称: E53_IA1_Read_Data
* 说    明: 测量光照强度、温度、湿度
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void SHT3x_Read_Data(E53_ST1_Data_TypeDef *ReadData)
{
    //启动传感器采集数据
    

    uint8_t data[3];
    uint16_t dat, tmp;
    uint8_t SHT3X_Data_Buffer[6];

    WifiIotI2cData sht30_i2c_data = {0};
    uint8_t send_data[2] = {0xE0, 0x00};
    sht30_i2c_data.sendBuf = send_data;
    sht30_i2c_data.sendLen = 2;
    sht30_i2c_data.receiveBuf = SHT3X_Data_Buffer;
    sht30_i2c_data.receiveLen = 6;
    I2cWriteread(WIFI_IOT_I2C_IDX_1, (SHT30_Addr << 1) | 0x00, &sht30_i2c_data);

    data[0] = SHT3X_Data_Buffer[0];
    data[1] = SHT3X_Data_Buffer[1];
    data[2] = SHT3X_Data_Buffer[2];

    tmp = SHT3x_CheckCrc(data, 2, data[2]);
    if (!tmp)
    {
        dat = ((uint16_t)data[0] << 8) | data[1];
        ReadData->Temperature = SHT3x_CalcTemperatureC(dat);
    }

    data[0] = SHT3X_Data_Buffer[3];
    data[1] = SHT3X_Data_Buffer[4];
    data[2] = SHT3X_Data_Buffer[5];

    tmp = SHT3x_CheckCrc(data, 2, data[2]);
    if (!tmp)
    {
        dat = ((uint16_t)data[0] << 8) | data[1];
        ReadData->Humidity = SHT3x_CalcRH(dat);
    }
}

/***************************************************************
* 函数名称: Beep_StatusSet
* 说    明: 蜂鸣器报警与否
* 参    数: status,ENUM枚举的数据
*									OFF,蜂鸣器
*									ON,开蜂鸣器
* 返 回 值: 无
***************************************************************/
void Beep_StatusSet(E53_ST1_Status_ENUM status)
{
    if (status == ON)
        PwmStart(WIFI_IOT_PWM_PORT_PWM1, 20000, 40000); //输出不同占空比的PWM波
    if (status == OFF)
        PwmStop(WIFI_IOT_PWM_PORT_PWM1);
}

