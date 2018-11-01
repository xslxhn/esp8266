/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
/*
 * 徐松亮问题记录
 * 		代码结构
 * 		os_printf printf怎么区别
 * 		JSON格式的具体应用
 * 		模块全信息打印
 * 		OTA升级的makefile文件设置
 *		时钟设置
 *		flash分区设置
 *******************************************************************************
 *							Esp8266的Flash空间分配表	Begin
 *******************************************************************************
 *------------------------------------------------------------------------------
 *							512		1024	2048				4096
 *							512*2	1024*2	512*2	1024*2		512*2	1024*2
 *------------------------------------------------------------------------------
 *	master_device_key.bin	0x3E000	0x7E000	0x7E000	0xFE000		0x7E000	0xFE000
 *	esp_init_data_de...bin	0x7C000	0xFC000	0x1FC000			0x3FC000
 *	blank.bin				0x7E000	0xFE000	0x1FE000			0x3FE000
 *	boot.bin				0x00000
 *	user1.bin				0x01000
 *	user2.bin				0x41000	0x81000	0x81000	0x101000	0x81000	0x101000
 *******************************************************************************
 *							Esp8266的Flash空间分配图
 *	boot	Irom0text.bin	Flash.bin	UserPara
 *	4KB		<=940KB			<=64KB		16KB
 *			|<		User1.bin		>|
 *	|<				1024KB					>|
 *	--------------------------------------------
 *	Res		Irom0text.bin	Flash.bin	UserData
 *	4KB		<=940KB			<=64KB		16KB
 *			|<		User2.bin		>|
 *	|<				1024KB					>|
 *	--------------------------------------------
 *	UserData							System Param
 *	xxxKB								16KB
 *******************************************************************************
 *							注意
 *	Esp8266的代码允许最大量是1024K,与外置Flash的容量没有关系.
 *******************************************************************************
 *							Esp8266的Flash空间分配表	End
 *******************************************************************************
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * */
#include "esp_common.h"

#include "gpio.h"

#include "../xsl_config/includes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_libc.h"
//-----xsl-----
#include "includes.h"
#include "Module_HttpApp.h"
#include "Module_cJSON.h"
//
#include "lwip/err.h"
//
#define os_memcpy memcpy

uint32_t main_RunTimeS = 0;
// 0 未连接到路由 1-已连接到路由 2-smartconfig准备 3-smartconfig进行中
uint8_t ConnectState = 0;

extern void Module_UdpClient_Init(); //初始化
extern void si4463_port_init(void);
extern void SI4463_PeriodFun(void);
extern void SI4463_AppSend(void);
extern struct ip_addr udp_RemoteIpAddr;
extern uint8_t udp_DnsOK;

extern void Task_MQTT_Init(void);

/*
 * 函数功能: SmartConfig
 * */
void ICACHE_FLASH_ATTR smartconfig_done(sc_status status, void *pdata) {

	switch (status) {
	// 连接未开始，请勿在此阶段开始连接
	case SC_STATUS_WAIT:
		os_printf("SC_STATUS_WAIT\n");
		break;

		// 发现信道
	case SC_STATUS_FIND_CHANNEL:
		os_printf("SC_STATUS_FIND_CHANNEL\n");
		break;

		// 得到wifi名字和密码
	case SC_STATUS_GETTING_SSID_PSWD:
		os_printf("SC_STATUS_GETTING_SSID_PSWD\n");
		sc_type *type = pdata;
		if (*type == SC_TYPE_ESPTOUCH) {
			os_printf("SC_TYPE:SC_TYPE_ESPTOUCH\n");
		} else {
			os_printf("SC_TYPE:SC_TYPE_AIRKISS\n");
		}
		break;
		// 启动连接
	case SC_STATUS_LINK:
		os_printf("SC_STATUS_LINK\n");
		struct station_config *sta_conf = pdata;

		wifi_station_set_config(sta_conf);
		wifi_station_disconnect();
		wifi_station_connect();
		break;

		//成功获取到IP，连接路由完成。
	case SC_STATUS_LINK_OVER:
		os_printf("SC_STATUS_LINK_OVER \n\n");
		if (pdata != NULL) {
			uint8 phone_ip[4] = { 0 };
			os_memcpy(phone_ip, (uint8*) pdata, 4);
			os_printf("Phone ip: %d.%d.%d.%d\n", phone_ip[0], phone_ip[1],
					phone_ip[2], phone_ip[3]);
			ConnectState = 1;
		}

		//停止配置
		smartconfig_stop();
		break;
	}

}

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
 *******************************************************************************/
uint32 user_rf_cal_sector_set(void) {
	flash_size_map size_map = system_get_flash_size_map();
	uint32 rf_cal_sec = 0;

	switch (size_map) {
	case FLASH_SIZE_4M_MAP_256_256:
		rf_cal_sec = 128 - 5;
		break;

	case FLASH_SIZE_8M_MAP_512_512:
		rf_cal_sec = 256 - 5;
		break;

	case FLASH_SIZE_16M_MAP_512_512:
	case FLASH_SIZE_16M_MAP_1024_1024:
		rf_cal_sec = 512 - 5;
		break;

	case FLASH_SIZE_32M_MAP_512_512:
	case FLASH_SIZE_32M_MAP_1024_1024:
		rf_cal_sec = 1024 - 5;
		break;

	default:
		rf_cal_sec = 0;
		break;
	}

	return rf_cal_sec;
}
/******************************************************************************
 * FunctionName : smartconfig_task
 * Description  : task->smartconfig_task
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void ICACHE_FLASH_ATTR
smartconfig_task(void *pvParameters) {
	os_printf("\r\ntask create OK!!\r\n");
	for (;;) {
		//os_delay_us(1000000);
		vTaskDelay(100);
		//-----xsl-----Begin	watch dog
		//system_soft_wdt_feed();
		//-------------End
		//-----xsl-----Begin	lwip	-	DNS
#if		(HARDWARE_VER==202)
		if (1) {
			static uint8_t sfirst = 1;
			if (sfirst == 1) {
				sfirst = 0;
				os_printf("\r\n-----DNS TEST-----\r\n");
				dns_init();
				struct ip_addr addr;
				char name0[] = "www.baidu.com";
				//调用netconn_gethostbyname
				err_t err = netconn_gethostbyname((char*) name0, &addr);
				//定义字符串来接收IP
				char* ipAdress;
				if (err == ERR_OK) {
					//格式化IP
					ipAdress = ip_ntoa(&addr);
					//打印下
					printf("DNS ok , %s ip is:%s\r\n", name0, ipAdress);
				} else {
					//return;
				}
			}
		}
#elif	(HARDWARE_VER==33)
		if (1) {
			static uint8_t sfirst = 1;
			if (sfirst == 1) {
				sfirst = 0;
				os_printf("\r\n-----DNS TEST-----\r\n");
				dns_init();
				struct ip_addr addr;
				char name0[] = "dlisleep.xikang.com";
				//调用netconn_gethostbyname
				err_t err = netconn_gethostbyname((char*) name0, &addr);
				//定义字符串来接收IP
				char* ipAdress;
				if (err == ERR_OK) {
					//格式化IP
					ipAdress = ip_ntoa(&addr);
					//打印下
					printf("DNS ok , %s ip is:%s\r\n", name0, ipAdress);
					//
					memcpy((char*)&udp_RemoteIpAddr,(char*)&addr,sizeof(addr));
					udp_DnsOK=1;
				} else {
					//return;
					sfirst=1;
				}
			}
		}
#endif
		//-----xsl-----Begin	LED
#if		(HARDWARE_VER==202)
		if (1) {
			static char si = 0;
			si++;
			if (si >= 100) {
				si = 1;
			}
			if ((si % 2) != 0) {
				GPIO_OUTPUT_SET(GPIO_ID_PIN(14), 1);
				GPIO_OUTPUT_SET(GPIO_ID_PIN(4), 1);
				GPIO_OUTPUT_SET(GPIO_ID_PIN(5), 1);
				GPIO_OUTPUT_SET(GPIO_ID_PIN(0), 1);
				GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 1);
				gpio16_output_set(1);
			} else {
				GPIO_OUTPUT_SET(GPIO_ID_PIN(14), 0);
				GPIO_OUTPUT_SET(GPIO_ID_PIN(4), 0);
				GPIO_OUTPUT_SET(GPIO_ID_PIN(5), 0);
				GPIO_OUTPUT_SET(GPIO_ID_PIN(0), 0);
				GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 0);
				//
				gpio16_output_set(0);
			}
		}
#elif	((HARDWARE_VER==33)&&(HARDWARE_SUB_VER==2))
		if (ConnectState == 0) {
			if (wifi_station_get_connect_status() == STATION_GOT_IP) {
				ConnectState = 1;
			}
		} else if (ConnectState == 1) {
			if (wifi_station_get_connect_status() != STATION_GOT_IP) {
				ConnectState = 0;
			}
		} else if (ConnectState == 2) {
			ConnectState = 3;
			smartconfig_set_type(SC_TYPE_ESPTOUCH); //SC_TYPE_ESPTOUCH,SC_TYPE_AIRKISS,SC_TYPE_ESPTOUCH_AIRKISS
			wifi_set_opmode(STATION_MODE);
			smartconfig_start(smartconfig_done);
		}
#endif
		//-----XSL-----Begin	Test6	-->	Http Get/Post
#if		(HARDWARE_VER==202 || HARDWARE_VER==33)

		/*
		 {
		 static uint16 s_flag = 500;
		 if(s_flag!=0)s_flag --;
		 if (s_flag != 0 && (s_flag%5==0)) {
		 if (OK == Module_HttpApp_GetWeather()) {
		 printf("Get freeHeap: %d\n", system_get_free_heap_size());
		 vTaskDelay(100);
		 my_station_disinit();
		 }
		 }
		 }
		 */
		if(0)
		{
			static uint8 s_flag = 1;
			if (s_flag == 1) {
				if (OK
						== Module_HttpApp_GetOtaInfo("172.27.35.1",
								"xsl/info.json", 8080,
								false, true)) {
					s_flag = 0;
					printf("Get freeHeap: %d\n", system_get_free_heap_size());
				}
			}
		}
		if(0)
		{
			static uint16 s_flag = 500;
			if (s_flag != 0)
				s_flag--;
			if (s_flag != 0 && (s_flag % 5 == 0)) {
				if (OK == Module_HttpApp_GetOtaInfo("172.27.35.1",
						"xsl/info.json", 8080,
						false, true)) {
					printf("Get freeHeap: %d\n", system_get_free_heap_size());
				}
			}
		}
#endif

		//-------------End
		//-----XSL-----Begin	OTA
		//-------------End
	}
}
/******************************************************************************
 * FunctionName :
 * Description  :
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
static void Softtimer_Fun(void) {
	//-----XSL-----Begin	Test3	-->	New Soft timer
#if	(HARDWARE_VER==202)
	static uint32 duty = 0;
	uint8 y;
	if (1) {
		if (duty < 2048) {
			duty += 50;
		} else {
			duty = 0;
		}
		for (y = 0; y < 3; y++) {
			//设置 PWM 某个通道信号的占空比, duty 占空比的值, type当前要设置的PWM通道
			pwm_set_duty(abs(duty - 1024), y);
			//设置完成后，需要调用 pwm_start,PWM 开始
			pwm_start();
		}
		//printf("Timer duty=%d!\r\n",duty);
	}
#endif
	//-----XSL-----End
	//-----XSL-----SI4432
#if	((HARDWARE_VER==33))
	SI4463_PeriodFun();
#endif
	//-------------
	//-----XSL-----KEY
#if	((HARDWARE_VER==33)&&(HARDWARE_SUB_VER==2))
	{
		static uint8_t si = 0;
		static uint8_t stimer = 0;
		if (si == 0) {
			if (0 == GPIO_INPUT_GET(GPIO_ID_PIN(0))) {
				if (stimer != 0xFF)
				stimer++;
			} else if (1 == GPIO_INPUT_GET(GPIO_ID_PIN(0))) {
				si = 1;
			}
		} else if (si == 1) {
			if ((stimer >= 1) && (stimer <= 5)) {
				printf("Key Press\n");
				SI4463_AppSend();
			} else if (stimer > 30) {
				os_printf("----------begin to SmartConfig\n\n\n-----");
				ConnectState = 2;
			}
			si = 0;
			stimer = 0;
		}
		//LED
		if (1) {
			static char si = 0;
			si++;
			if (si >= 200) {
				si = 1;
			}
			//
			if (ConnectState == 1) {
				if (si == 1) {
					gpio16_output_set(0);
				} else if (si == 3) {
					gpio16_output_set(1);
				} else if (si >= 30) {
					si = 0;
				}
			} else if (ConnectState == 0) {
				if (si == 1) {
					gpio16_output_set(0);
				} else if (si == 3) {
					gpio16_output_set(1);
				} else if (si >= 10) {
					si = 0;
				}
			} else if (ConnectState == 2 || ConnectState == 3) {
				if (si == 1) {
					gpio16_output_set(0);
				} else if (si == 3) {
					gpio16_output_set(1);
				} else if (si >= 5) {
					si = 0;
				}
			}
		}
	}
#endif
}
/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void user_init(void) {
	uart_init_new();
	printf("----------Xsl-Info\r\n");
	printf("SDK version   :%s\n", system_get_sdk_version());
	printf("Boot version  :%d\n", system_get_boot_version());
	printf("User Bin Addr :0x%08X\n", system_get_userbin_addr());
	{
		uint8_t i;
		i = system_get_boot_mode();
		if (i == 0) {
			printf("Boot Mode: SYS_BOOT_ENHANCE_MODE\r\n");
		} else if (i == 1) {
			printf("Boot Mode: SYS_BOOT_NORMAL_MODE\r\n");
		}
	}
	{
		flash_size_map e_FlashSizeMap = system_get_flash_size_map();
		if (e_FlashSizeMap == FLASH_SIZE_4M_MAP_256_256) {
			printf("Flash Size Map: FLASH_SIZE_4M_MAP_256_256\r\n");
		} else if (e_FlashSizeMap == FLASH_SIZE_2M) {
			printf("Flash Size Map: FLASH_SIZE_2M\r\n");
		} else if (e_FlashSizeMap == FLASH_SIZE_8M_MAP_512_512) {
			printf("Flash Size Map: FLASH_SIZE_8M_MAP_512_512\r\n");
		} else if (e_FlashSizeMap == FLASH_SIZE_16M_MAP_512_512) {
			printf("Flash Size Map: FLASH_SIZE_16M_MAP_512_512\r\n");
		} else if (e_FlashSizeMap == FLASH_SIZE_32M_MAP_512_512) {
			printf("Flash Size Map: FLASH_SIZE_32M_MAP_512_512\r\n");
		} else if (e_FlashSizeMap == FLASH_SIZE_16M_MAP_1024_1024) {
			printf("Flash Size Map: FLASH_SIZE_16M_MAP_1024_1024\r\n");
		} else if (e_FlashSizeMap == FLASH_SIZE_32M_MAP_1024_1024) {
			printf("Flash Size Map: FLASH_SIZE_32M_MAP_1024_1024\r\n");
		}
	}
	printf("Ai-Thinker Technology Co. Ltd.\r\n%s %s\r\n", __DATE__, __TIME__);
	printf("Get freeHeap: %d\n", system_get_free_heap_size());
	printf("Chip ID: 0x%08X\n", system_get_chip_id());
	printf("Cpu Freq: %d(MHz)\r\n", system_get_cpu_freq());
	printf("Flash ID: %d\r\n", spi_flash_get_id());
	{
		uint8 macAdress[6];
		if (!wifi_get_macaddr(STATION_IF, macAdress)) {
			printf("Failed to get mac!\r\n");
		} else {
			printf("Mac Addr:  %02X:%02X:%02X:%02X:%02X:%02X\r\n", macAdress[0],
					macAdress[1], macAdress[2], macAdress[3], macAdress[4],
					macAdress[5]);
		}
	}
	{
		struct rst_info *prst_info = system_get_rst_info();
		if (prst_info->reason == REASON_DEFAULT_RST) {
			printf("Res Info: normal startup by power on\r\n");
		} else if (prst_info->reason == REASON_WDT_RST) {
			printf("Res Info: hardware watch dog reset\r\n");
		} else if (prst_info->reason == REASON_EXCEPTION_RST) {
			printf("Res Info: exception reset, GPIO status won't change\r\n");
		} else if (prst_info->reason == REASON_SOFT_WDT_RST) {
			printf(
					"Res Info: software watch dog reset, GPIO status won't change\r\n");
		} else if (prst_info->reason == REASON_SOFT_RESTART) {
			printf(
					"Res Info: software restart ,system_restart , GPIO status won't change\r\n");
		} else if (prst_info->reason == REASON_DEEP_SLEEP_AWAKE) {
			printf("Res Info: wake up from deep-sleep\r\n");
		} else if (prst_info->reason == REASON_EXT_SYS_RST) {
			printf("Res Info: external system reset\r\n");
		}
	}
	printf("Mem Info:\r\n");
	system_print_meminfo();
	if (system_upgrade_userbin_check() == 0x00) {
		printf("Fota Info: now is user1.bin\n");
	} else if (system_upgrade_userbin_check() == 0x01) {
		printf("Fota Info: now is user2.bin\n");
	} else {
		printf("Fota Info: now is not support FOTA\n");
	}
	printf("----------\r\n");
#if		(HARDWARE_VER==202)
	printf("----------Xsl-TEST-cJSON-Parse\r\n");
	Module_cJSON_ParseJson();
	printf("----------Xsl-TEST-cJSON-Create\r\n");
	Module_cJSON_CreateJson();
	//printf("----------Xsl-TEST-Flash\r\n");
	//spi_flash_erase_sector(0);
	//spi_flash_write()
	//spi_flash_read
	printf("----------\r\n");
#endif
	//
	BspLed_Init();
	//-----XSL-----Begin	Test1	-->	init GPIO
#if	((HARDWARE_VER==33) && (HARDWARE_SUB_VER==2))
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);
#elif	(HARDWARE_VER==202)
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
	gpio16_output_conf();
	gpio16_output_set(1);
#endif
	printf("GPIO Init OK!\r\n");
	//-----XSL-----End
	//-----XSL-----Begin	Test2	-->	PWM
#if	(HARDWARE_VER==202)
	if (1) {
		uint32 pwm_duty_init[3] = { 0 };
		//初始化 PWM，1000周期,pwm_duty_init占空比,3通道数
		uint32 io_info[][3] = { { PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12, 12 }\
, {
		PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13, 13 }\
, {
		PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15, 15 } };
		//开始初始化
		pwm_init(1000, pwm_duty_init, 3, io_info);
		//
		pwm_set_duty(0, 0);
		pwm_set_duty(0, 1);
		pwm_set_duty(0, 2);
		pwm_start();
	}
#endif
	//-----XSL-----End
	//-----XSL-----Begin	Test3	-->	New Soft timer
	if (1) {
		static os_timer_t os_timer;
		//timer off
		os_timer_disarm(&os_timer);
		//set fun
		os_timer_setfn(&os_timer, (os_timer_func_t*) (Softtimer_Fun), NULL);
		//timer on
		os_timer_arm(&os_timer, 100, true);
	}
	//-----XSL-----End
	//-----XSL-----Begin	Test4	-->	New Task
	if (1) {
		xTaskCreate(smartconfig_task, "smartconfig_task", 256, NULL, 2, NULL);
	}
	//-----XSL-----End
	//-----XSL-----Begin	Test5	-->	static connect AP / smartconfig
#if		(HARDWARE_VER==202)
	// 固定wifi连接
	if (1) {
		struct station_config stationConf;
		wifi_set_opmode(0x01); //设置为STATION模式
		strcpy(stationConf.ssid, "xslxhn"); //改成你自己的   路由器的用户名
		strcpy(stationConf.password, "19820824"); //改成你自己的   路由器的密码
		wifi_station_set_config(&stationConf); //设置WiFi station接口配置，并保存到 flash
		wifi_station_connect(); //连接路由器
	}
#endif
	Module_Ota_Init();
	//-----XSL-----End
	//-----XSL-----Begin	Test7	-->	8266作为TCP客户端，目标端口为6000，IP地址见串口打印，此要先获取Tcp服务端的ip地址
	//Module_TcpClient_Init();
	//-----XSL-----Begin	Test8	-->	8266作为TCP服务器，本地和目标端口为8266，IP地址见串口打印
	//Module_TcpService_Init();
	//-----XSL-----Begin	Test9	-->	8266作为UDP客户端，本地端口为2000，目标端口为8686，IP地址见串口打印
#if	((HARDWARE_VER==33))
	dns_init();
	Module_UdpClient_Init();
#endif
	//-----XSL-----Begin	Test10	-->	8266作为UDP服务器，本地和目标端口为8266，IP地址见串口打印
	//Module_UdpService_Init();
	//-----XSL-----End
	//-----XSL-----Begin	Test11	-->	SI4463
#if	((HARDWARE_VER==33))
	si4463_port_init();
#endif
	//-----XSL-----End
	//-----XSL-----Begin	Test	-->	MQTT
#if		(HARDWARE_VER==202)
	Task_MQTT_Init();
#endif
	//-----XSL-----End
}
