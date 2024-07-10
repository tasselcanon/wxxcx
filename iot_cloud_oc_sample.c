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
#include <stdlib.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"

#include "wifi_connect.h"
#include <queue.h>
#include <oc_mqtt_al.h>
#include <oc_mqtt_profile.h>
#include "E53_ST1.h"
#include <dtls_al.h>
#include <mqtt_al.h>

#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_adc.h"
#include "wifiiot_uart.h"

#define UART_TASK_STACK_SIZE 1024 * 8
#define UART_TASK_PRIO 25
#define UART_BUFF_SIZE 1000

#define CONFIG_WIFI_SSID          "dust"                            //修改为自己的WiFi 热点账号

#define CONFIG_WIFI_PWD           "1234567890"                        //修改为自己的WiFi 热点密码

//#define CONFIG_APP_SERVERIP       "121.36.42.100"                       //基础版平台对接地址

#define CONFIG_APP_SERVERIP       "117.78.5.125"                       //标准版平台对接地址

#define CONFIG_APP_SERVERPORT     "1883"

#define CONFIG_APP_DEVICEID       "bearpi001"       //替换为注册设备后生成的deviceid

#define CONFIG_APP_DEVICEPWD      "123456789"        //替换为注册设备后生成的密钥

#define CONFIG_APP_LIFETIME       120     ///< seconds

#define CONFIG_QUEUE_TIMEOUT      (5*1000)

#define MSGQUEUE_OBJECTS 16 // number of Message Queue Objects

uint8_t uart_buff[UART_BUFF_SIZE] = {0};
uint8_t *uart_buff_ptr = uart_buff;

typedef enum
{
    en_msg_cmd = 0,
    en_msg_report,
}en_msg_type_t;

typedef struct
{
    char *request_id;
    char *payload;
} cmd_t;

typedef struct//需要上传的数据加在这里
{
    char Longitude [10];
    char Latitude  [10];
    char Humidity[10];
    char Temperature[10];
    char code      [13];
} report_t;

typedef struct
{
    en_msg_type_t msg_type;
    union
    {
        cmd_t cmd;
        report_t report;
    } msg;
} app_msg_t;

typedef struct
{
    queue_t         *app_msg;
    int             connected;
    int             beep;
    char           *code;
} app_cb_t;
static app_cb_t g_app_cb;

static void deal_report_msg(report_t *report)//这个函数主要是将报告消息的经纬度信息和蜂鸣器状态通过MQTT协议发送给服务器。
{
    oc_mqtt_profile_service_t service;
    oc_mqtt_profile_kv_t Longitude_value;
    oc_mqtt_profile_kv_t Latitude_value;
    oc_mqtt_profile_kv_t beep;
    oc_mqtt_profile_kv_t BarCode;
    oc_mqtt_profile_kv_t Temperature;
    oc_mqtt_profile_kv_t Humidity;

    if(g_app_cb.connected != 1){
        return;
    }

    service.event_time = NULL;
    service.service_id = "ALL";
    service.service_property = &Longitude_value;
    service.nxt = NULL;

    Longitude_value.key = "Longitude";
    Longitude_value.value = &report->Longitude;
    Longitude_value.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    Longitude_value.nxt = &Latitude_value;

    Latitude_value.key = "Latitude";
    Latitude_value.value = &report->Latitude;
    Latitude_value.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    Latitude_value.nxt = &beep;

    beep.key = "BeepStatus";
    beep.value = g_app_cb.beep ? "ON" : "OFF";
    beep.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    beep.nxt = &BarCode;
    
    BarCode.key = "BarCode";
    BarCode.value = g_app_cb.code;
    BarCode.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    BarCode.nxt = &Temperature;

    Temperature.key = "Temperature";
    Temperature.value =  &report->Temperature;
    Temperature.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    Temperature.nxt = &Humidity;

    Humidity.key = "Humidity";
    Humidity.value =  &report->Humidity;
    Humidity.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    Humidity.nxt = NULL;




    oc_mqtt_profile_propertyreport(NULL,&service);
    return;
}

//use this function to push all the message to the buffer
static int msg_rcv_callback(oc_mqtt_profile_msgrcv_t *msg)//这个函数主要负责将接收到的MQTT消息转换为app_msg_t结构体，并将其推送到消息队列中。
{
    int    ret = 0;
    char  *buf;
    int    buf_len;
    app_msg_t *app_msg;

    if((NULL == msg)|| (msg->request_id == NULL) || (msg->type != EN_OC_MQTT_PROFILE_MSG_TYPE_DOWN_COMMANDS)){
        return ret;
    }

    buf_len = sizeof(app_msg_t) + strlen(msg->request_id) + 1 + msg->msg_len + 1;
    buf = malloc(buf_len);
    if(NULL == buf){
        return ret;
    }
    app_msg = (app_msg_t *)buf;
    buf += sizeof(app_msg_t);

    app_msg->msg_type = en_msg_cmd;
    app_msg->msg.cmd.request_id = buf;
    buf_len = strlen(msg->request_id);
    buf += buf_len + 1;
    memcpy(app_msg->msg.cmd.request_id, msg->request_id, buf_len);
    app_msg->msg.cmd.request_id[buf_len] = '\0';

    buf_len = msg->msg_len;
    app_msg->msg.cmd.payload = buf;
    memcpy(app_msg->msg.cmd.payload, msg->msg, buf_len);
    app_msg->msg.cmd.payload[buf_len] = '\0';

    ret = queue_push(g_app_cb.app_msg,app_msg,10);
    if(ret != 0){
        free(app_msg);
    }

    return ret;
}

///< COMMAND DEAL
#include <cJSON.h>
static void deal_cmd_msg(cmd_t *cmd)//这个函数主要是根据接收到的命令消息内容执行相应的操作，并发送命令执行结果的响应消息。
{
    cJSON *obj_root;
    cJSON *obj_cmdname;
    cJSON *obj_paras;
    cJSON *obj_para;

    int cmdret = 1;
    oc_mqtt_profile_cmdresp_t cmdresp;
    obj_root = cJSON_Parse(cmd->payload);
    if (NULL == obj_root)
    {
        goto EXIT_JSONPARSE;
    }

    obj_cmdname = cJSON_GetObjectItem(obj_root, "command_name");
    if (NULL == obj_cmdname)
    {
        goto EXIT_CMDOBJ;
    }
    if (0 == strcmp(cJSON_GetStringValue(obj_cmdname), "Track_Control_Beep"))
    {
        obj_paras = cJSON_GetObjectItem(obj_root, "paras");
        if (NULL == obj_paras)
        {
            goto EXIT_OBJPARAS;
        }
        obj_para = cJSON_GetObjectItem(obj_paras, "Beep");
        if (NULL == obj_para)
        {
            goto EXIT_OBJPARA;
        }
        ///< 此处操作蜂鸣器
        if (0 == strcmp(cJSON_GetStringValue(obj_para), "ON"))
        {
            g_app_cb.beep = 1;
            Beep_StatusSet(ON);
            printf("Beep On!\r\n");
        }
        else
        {
            g_app_cb.beep = 0;
            Beep_StatusSet(OFF);
            printf("Beep Off!\r\n");
        }
        cmdret = 0;
    }
    

EXIT_OBJPARA:
EXIT_OBJPARAS:
EXIT_CMDOBJ:
    cJSON_Delete(obj_root);
EXIT_JSONPARSE:
    ///< do the response
    cmdresp.paras = NULL;
    cmdresp.request_id = cmd->request_id;
    cmdresp.ret_code = cmdret;
    cmdresp.ret_name = NULL;
    (void)oc_mqtt_profile_cmdresp(NULL, &cmdresp);
    return;
}


static int task_main_entry(void)//这个函数主要是用于初始化网络连接和消息队列，连接到MQTT服务器，并循环处理从消息队列中接收到的命令和上报消息。
{
    app_msg_t *app_msg;
    uint32_t ret ;

    WifiConnect(CONFIG_WIFI_SSID, CONFIG_WIFI_PWD);
    dtls_al_init();
    mqtt_al_init();
    oc_mqtt_init();
    
    g_app_cb.app_msg = queue_create("queue_rcvmsg",10,1);
    if(NULL ==  g_app_cb.app_msg){
        printf("Create receive msg queue failed");
        
    }
    oc_mqtt_profile_connect_t  connect_para;
    (void) memset( &connect_para, 0, sizeof(connect_para));
    
    //配置mqtt登录数据
    connect_para.boostrap =      0;
    connect_para.device_id =     CONFIG_APP_DEVICEID;
    connect_para.device_passwd = CONFIG_APP_DEVICEPWD;
    connect_para.server_addr =   CONFIG_APP_SERVERIP;
    connect_para.server_port =   CONFIG_APP_SERVERPORT;
    connect_para.life_time =     CONFIG_APP_LIFETIME;
    connect_para.rcvfunc =       msg_rcv_callback;
    connect_para.security.type = EN_DTLS_AL_SECURITY_TYPE_NONE;
    ret = oc_mqtt_profile_connect(&connect_para);
    if((ret == (int)en_oc_mqtt_err_ok)){
        g_app_cb.connected = 1;
        printf("oc_mqtt_profile_connect succed!\r\n");
    }
    else
    {
        printf("oc_mqtt_profile_connect faild!\r\n");
    }


    while (1)
    {
        app_msg = NULL;
        (void)queue_pop(g_app_cb.app_msg,(void **)&app_msg,0xFFFFFFFF);
        if (NULL != app_msg)
        {
            switch (app_msg->msg_type)
            {
            case en_msg_cmd:
                deal_cmd_msg(&app_msg->msg.cmd);//这个函数主要是根据接收到的命令消息内容执行相应的操作，并发送命令执行结果的响应消息。
                break;
            case en_msg_report:
                deal_report_msg(&app_msg->msg.report);//这个函数主要是将报告消息的经纬度信息和蜂鸣器状态通过MQTT协议发送给服务器。
                break;
            default:
                break;
            }
            free(app_msg);
        }
    }
    return 0;
}

static const char *datas = "Hello, BearPi!\r\n";

void UART_Task(void)
    {
        
        uint32_t ret;

        
        //初始化GPIO
        GpioInit();

        //设置GPIO_12，12引脚复用功能为UART
        IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
        IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);

        //设置GPIO_11引脚为输出模式，12为输入模式
        GpioSetDir(WIFI_IOT_GPIO_IDX_11, WIFI_IOT_GPIO_DIR_OUT);
        GpioSetDir(WIFI_IOT_GPIO_IDX_12, WIFI_IOT_GPIO_DIR_IN);
        /*串口的配置*/
        WifiIotUartAttribute uart_attr = {

            //baud_rate: 9600
            .baudRate = 9600,//波特率

            //data_bits: 8bits
            .dataBits = 8,//数据位8
            .stopBits = 1,//校验位是1
            .parity = 0,
        };

        //Initialize uart driver
        ret = UartInit(WIFI_IOT_UART_IDX_2, &uart_attr, NULL);
        /*第一个参数是串口选择，第二个参数是指针指向串口配置，第三个配置是拓展配置*/
        if (ret != WIFI_IOT_SUCCESS)
        {
            printf("Failed to init uart! Err code = %d\n", ret);
            return;
        }
        printf("UART Test Start\n");
        int a;
        a=0;
        char* ptr = NULL;
        ptr = strdup("00000");
        char* kong = NULL;
        kong = strdup("00000");
        while (1)
        {
            
            printf("*************UART_example**************\r\n");
            
            //通过串口2发送数据
            UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)datas, strlen(datas));

            //通过串口2接收数据
            UartRead(WIFI_IOT_UART_IDX_2, uart_buff_ptr, UART_BUFF_SIZE);
             
             if(uart_buff[2]){
                g_app_cb.code = (char*)uart_buff_ptr;
            }

             if(*ptr!=*g_app_cb.code)
            {
                ptr=g_app_cb.code;
                a=0;
            }
            else
            {
                a=a+1;
            }
            if(a>=4)
            {
                g_app_cb.code = kong;
                a=0;
            }
            
            usleep(1000000);
        }
    }

/***** 获取电压值函数 *****/
static float GetVoltage(void)
{
unsigned int ret;
 unsigned short data;

 ret = AdcRead(WIFI_IOT_ADC_CHANNEL_6, &data, WIFI_IOT_ADC_EQU_MODEL_8, WIFI_IOT_ADC_CUR_BAIS_DEFAULT, 0xff);
 /*参数：1，channelADC通道；2，data用于存放读取数据的地址的指针；3，equModel平均算法的次数；4，curBais，模拟功率控制模式；
5，rstCnt从重置到转换开始的时间计数*/
if (ret != WIFI_IOT_SUCCESS)
{
 printf("ADC Read Fail\n");
 }

 return (float)data * 1.8 * 4 / 4096.0;
}


static int task_sensor_entry(void)//这个函数主要实现了不断从传感器读取数据，将经度和纬度值打包成上报消息，并推送到消息队列中，然后每隔3秒钟重复执行这个过程。
{
    E53_ST1_Data_TypeDef data;
    app_msg_t *app_msg;
    E53_ST1_Status_ENUM status;

    Init_E53_ST1();
    float voltage;
    while (1)
    {
        E53_ST1_Read_Data(&data);//这里读取GPS信号
        SHT3x_Read_Data(&data);
        voltage = GetVoltage();

        printf("\r\n******************************vat Value is  %.3f\r\n",voltage);
        printf("\r\n******************************Longitude Value is  %.5f\r\n", data.Longitude);
		printf("\r\n******************************Latitude Value is  %.5f\r\n", data.Latitude);
        printf("\r\n******************************Humidity Value is  %.5f\r\n", data.Humidity);
		printf("\r\n******************************Temperature Value is  %.5f\r\n", data.Temperature);
        app_msg = malloc(sizeof(app_msg_t));
        if ((NULL != app_msg) )//y& (data.Longitude != 0) & (data.Latitude != 0))
        {
            app_msg->msg_type = en_msg_report;
            sprintf(app_msg->msg.report.Longitude,"%.5f\0",data.Longitude);
            sprintf(app_msg->msg.report.Latitude,"%.5f\0",data.Latitude);
            sprintf(app_msg->msg.report.Humidity,"%.5f\0",data.Humidity);
            sprintf(app_msg->msg.report.Temperature,"%.5f\0",data.Temperature);
            if(0 != queue_push(g_app_cb.app_msg,app_msg,CONFIG_QUEUE_TIMEOUT)){
                free(app_msg);
            }
        }
        if(data.Humidity>75||data.Temperature>35)
 {
 g_app_cb.beep = 1;
 status=ON;
Beep_StatusSet(status);
 SHT30_reset();
 if(voltage>2)
 {
 GpioSetOutputVal(WIFI_IOT_GPIO_IDX_9, 1); 
 }
 else 
{
GpioSetOutputVal(WIFI_IOT_GPIO_IDX_9, 0);
 }
        }
        
        else
        {
            g_app_cb.beep = 0;
            status=OFF;
            Beep_StatusSet(status);
            if(voltage>2)
            {
                GpioSetOutputVal(WIFI_IOT_GPIO_IDX_9, 1);              
            }
            else 
            {
                GpioSetOutputVal(WIFI_IOT_GPIO_IDX_9, 0);
            }
            
        }
        sleep(3);
    }
    return 0;
}

static void get_productID(void)
{
      UART_Task();
}

static void OC_Demo(void)//总的来说，这个函数主要负责创建两个线程，分别执行task_main_entry和task_sensor_entry函数中定义的任务。
{
    osThreadAttr_t attr;

    attr.name = "task_main_entry";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = 24;

    if (osThreadNew((osThreadFunc_t)task_main_entry, NULL, &attr) == NULL)
    {
        printf("Falied to create task_main_entry!\n");
    }

    attr.stack_size = 4096;
    attr.priority = 25;
    attr.name = "task_sensor_entry";
    if (osThreadNew((osThreadFunc_t)task_sensor_entry, NULL, &attr) == NULL)
    {
        printf("Falied to create task_sensor_entry!\n");
    }
attr.stack_size = 4096;
    attr.priority = 26;
    attr.name = "get_productID";
    if (osThreadNew((osThreadFunc_t)get_productID, NULL, &attr) == NULL)
    {
        printf("Falied to create get_productID!\n");
    }
}




APP_FEATURE_INIT(OC_Demo);