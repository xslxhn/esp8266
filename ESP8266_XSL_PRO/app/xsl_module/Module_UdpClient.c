#include "../xsl_config/includes.h"
#include "../xsl_module/Count.h"
#include "esp_common.h"
#include "espconn.h"//TCP连接需要的头文件

//-----XSL-----
#include "../xsl_module/MemManager.h"
#define os_memcpy memcpy
#define os_strcpy strcpy

static void UdpClient_XkapPacket(void);

uint8_t UdpClient_Cmd_TxKey = 0;
extern uint8_t si4463_Out_RxBuf[21];

struct ip_addr udp_RemoteIpAddr;
uint8_t udp_DnsOK=0;
//-------------
//extern bool wifi_set_broadcast_if(uint8 interface);

//#include "ets_sys.h"
//#include "os_type.h"
//#include "osapi.h"
//#include "mem.h"
//#include "user_interface.h"
//#include "user_json.h"
//#include "user_devicefind.h"
//-------------

static struct espconn Module_UdpClient_1_espconn;
static os_timer_t Module_UdpClient_1_Timer;

static void ICACHE_FLASH_ATTR Module_UdpClient_1_Send_Cb(void *arg)   //发送
{
	os_printf("\r\nUdpClient Send OK!\r\n");

}

static void ICACHE_FLASH_ATTR Module_UdpClient_1_Receive_Cb(void *arg,    //接收
		char *pdata, unsigned short len) {
	os_printf("nUdpClient Rx:%s", pdata);
	//每次发送数据确保参数不变
	/*
	 Module_UdpClient_1_espconn.proto.udp = (esp_udp *) os_zalloc(sizeof(esp_udp));
	 Module_UdpClient_1_espconn.type = ESPCONN_UDP;
	 Module_UdpClient_1_espconn.proto.udp->local_port = 2000;
	 Module_UdpClient_1_espconn.proto.udp->remote_port = 8080;
	 const char udp_remote_ip[4] = { 10, 10, 18, 202 }
	 os_memcpy(Module_UdpClient_1_espconn.proto.udp->remote_ip, udp_remote_ip, 4);
	 espconn_sent((struct espconn *) arg, "Rx OK!", strlen("Rx OK!"));
	 */
}

static void Module_UdpClient_TimerFun(void) {

	uint8 getState = wifi_station_get_connect_status();
	uint8 buf[11] = { 0 };
	static uint8 s_first = 1;
	static uint16 s16 = 0;
	//如果状态正确，证明已经连接
	if (getState == STATION_GOT_IP && udp_DnsOK==1) {
		if (s_first == 1) {
			s_first = 0;
			os_printf("WIFI Connect OK!");
			//os_timer_disarm(&Module_UdpClient_1_Timer);

			//wifi_set_broadcast_if(0x01);	 //设置 ESP8266 发送 UDP广播包时，从 station 接口发送
			//wifi_set_broadcast_if(STATION_MODE);
			Module_UdpClient_1_espconn.type = ESPCONN_UDP;	 		  //设置类型为UDP协议
			Module_UdpClient_1_espconn.proto.udp->local_port = 2000;	 		  //本地端口
			//-----XK-XSL调试
			/*
			 Module_UdpClient_1_espconn.proto.udp->remote_port = 8080;	 	//目标端口
			 const char udp_remote_ip[4] = { 10, 10, 18, 202 };//目标IP地址（广播）
			 */
			//-----XK-集成环境
			/*
			Module_UdpClient_1_espconn.proto.udp->remote_port = 2012;	 	//目标端口
			//const char udp_remote_ip[4] = { 10, 32, 144, 101 };//目标IP地址（广播）
			const char udp_remote_ip[4] = { 59, 46, 22, 16 };		//目标IP地址（广播）
			*/
			//
			//-----XK-XK环境
			Module_UdpClient_1_espconn.proto.udp->remote_port = 2012;	 	//目标端口
			char udp_remote_ip[4];

			udp_remote_ip[0]	=	(char)(udp_RemoteIpAddr.addr);
			udp_remote_ip[1]	=	(char)(udp_RemoteIpAddr.addr>>8);
			udp_remote_ip[2]	=	(char)(udp_RemoteIpAddr.addr>>16);
			udp_remote_ip[3]	=	(char)(udp_RemoteIpAddr.addr>>24);
			/*
			udp_remote_ip[3]	=	(char)(udp_RemoteIpAddr.addr);
			udp_remote_ip[2]	=	(char)(udp_RemoteIpAddr.addr>>8);
			udp_remote_ip[1]	=	(char)(udp_RemoteIpAddr.addr>>16);
			udp_remote_ip[0]	=	(char)(udp_RemoteIpAddr.addr>>24);
			*/
			os_printf("UDP udp_RemoteIpAddr: %03d,%03d,%03d,%03d",\
					udp_remote_ip[0],udp_remote_ip[1],udp_remote_ip[2],udp_remote_ip[3]);
			//-----
			os_memcpy(Module_UdpClient_1_espconn.proto.udp->remote_ip, udp_remote_ip, 4);

			espconn_regist_recvcb(&Module_UdpClient_1_espconn, Module_UdpClient_1_Receive_Cb);	 	//接收
			espconn_regist_sentcb(&Module_UdpClient_1_espconn, Module_UdpClient_1_Send_Cb);	 	//发送
			espconn_create(&Module_UdpClient_1_espconn);	 		  //建立 UDP 传输
		} else {
			/*
			 sprintf(buf, "Tx:%05d\r\n", s16++);
			 espconn_sent(&Module_UdpClient_1_espconn, buf, strlen(buf));
			 */
			UdpClient_XkapPacket();
		}
	}
}

void Module_UdpClient_Init() //初始化
{
	//-----XSL-----del
#if	0
	wifi_set_opmode(0x01); //设置为STATION模式
	struct station_config stationConf;
	os_strcpy(stationConf.ssid, "meizu");//改成你要连接的 路由器的用户名
	os_strcpy(stationConf.password, "12345678");//改成你要连接的路由器的密码

	wifi_station_set_config(&stationConf);//设置WiFi station接口配置，并保存到 flash
	wifi_station_connect();//连接路由器
#endif
	//-------------
	os_timer_disarm(&Module_UdpClient_1_Timer);	  //取消定时器定时
	os_timer_setfn(&Module_UdpClient_1_Timer, (os_timer_func_t *) Module_UdpClient_TimerFun,
	NULL);	  //设置定时器回调函数
	os_timer_arm(&Module_UdpClient_1_Timer, 1000, 1);	  //启动定时器，单位：毫秒
	//
	Module_UdpClient_1_espconn.proto.udp = (esp_udp *) os_zalloc(sizeof(esp_udp));//分配空间
}
//------------------------------------------------------------------------------
typedef enum GPRSAPP_XKAP_E_CMD {
	GPRSAPP_XKAP_E_CMD_NULL = 0x00,
	GPRSAPP_XKAP_E_CMD_SLEEP_DATA1 = 0x01,
	GPRSAPP_XKAP_E_CMD_SLEEP_DATA2 = 0x02,
	GPRSAPP_XKAP_E_CMD_SLEEP_DATA3 = 0x03,
	GPRSAPP_XKAP_E_CMD_SLEEP_DATA4 = 0x04,
	GPRSAPP_XKAP_E_CMD_SLEEP_DATA5 = 0x05,
	GPRSAPP_XKAP_E_CMD_GET_TIME = 0x06,
	GPRSAPP_XKAP_E_CMD_SEND_IMSI = 0x07,
	GPRSAPP_XKAP_E_CMD_PARA_R = 0x08,
	GPRSAPP_XKAP_E_CMD_PARA_W = 0x09,
	GPRSAPP_XKAP_E_CMD_SEND_MOVEPOWER = 0x0A,
	GPRSAPP_XKAP_E_CMD_PARA_UPLOAD = 0x0B,
	GPRSAPP_XKAP_E_CMD_SOS = 0x0C,
	GPRSAPP_XKAP_E_CMD_SEND_DAYMOVE = 0x0D,
	GPRSAPP_XKAP_E_CMD_SEND_INFO = 0x0E,
	GPRSAPP_XKAP_E_CMD_SEND_ERR = 0x0F,
	GPRSAPP_XKAP_E_CMD_PARA_R_1 = 0x10,
	GPRSAPP_XKAP_E_CMD_PARA_W_1 = 0x11,
	GPRSAPP_XKAP_E_CMD_IAP = 0x60,
	//
	GPRSAPP_XKAP_E_CMD_SLEEP_DATA_ALL,
	GPRSAPP_XKAP_E_CMD_SLEEP_DATA_SCAN,
} GPRSAPP_XKAP_E_CMD;
static void UdpClient_XkapPacket(void) {
	static uint8_t *pbuf;
	uint16_t i16 = 0, crc16;
	static uint8_t si = 0;
	static uint8_t first = 1;
	//
	if (first == 1) {
		first = 0;
		pbuf = MemManager_Get(E_MEM_MANAGER_TYPE_256B);
	}
	//
	if (UdpClient_Cmd_TxKey == 0)
		return;
	UdpClient_Cmd_TxKey = 0;
	//
	if (si4463_Out_RxBuf[0] != 0x55) {
		si4463_Out_RxBuf[0] = 0;
		return;
	}
	//
	pbuf[0] = Count_Sum(0, si4463_Out_RxBuf, 20);
	if (pbuf[0] != si4463_Out_RxBuf[20]) {
		si4463_Out_RxBuf[0] = 0;
		return;
	}
	//
	//功能码
	pbuf[i16++] = GPRSAPP_XKAP_E_CMD_SEND_INFO;
	//MCU-ID
	pbuf[i16++] = si4463_Out_RxBuf[2];
	pbuf[i16++] = si4463_Out_RxBuf[3];
	pbuf[i16++] = si4463_Out_RxBuf[4];
	pbuf[i16++] = si4463_Out_RxBuf[5];
	pbuf[i16++] = si4463_Out_RxBuf[6];
	pbuf[i16++] = si4463_Out_RxBuf[7];
	pbuf[i16++] = si4463_Out_RxBuf[8];
	pbuf[i16++] = si4463_Out_RxBuf[9];
	pbuf[i16++] = si4463_Out_RxBuf[10];
	pbuf[i16++] = si4463_Out_RxBuf[11];
	pbuf[i16++] = si4463_Out_RxBuf[12];
	pbuf[i16++] = si4463_Out_RxBuf[13];
	//整包序号
	pbuf[i16++] = 0;
	//数据包
	//---硬件版本号
	pbuf[i16++] = si4463_Out_RxBuf[1];
	pbuf[i16++] = 1;
	//---时间戳
	pbuf[i16++] = 0;
	pbuf[i16++] = 0;
	pbuf[i16++] = 0;
	pbuf[i16++] = 0;
	pbuf[i16++] = 0;
	pbuf[i16++] = 0;
	//---标签  SOS 平安 请回电 离开 回家 热释电
	pbuf[i16++] = 5;
	//---数据
	if (si4463_Out_RxBuf[14] == 1) {
		pbuf[i16++] = 2;	// SOS
	} else if (si4463_Out_RxBuf[15] == 1) {
		pbuf[i16++] = 3;	// 平安
	} else if (si4463_Out_RxBuf[16] == 1) {
		pbuf[i16++] = 6;	// 请回电
	} else if (si4463_Out_RxBuf[17] == 1) {
		pbuf[i16++] = 4;	// 离开
	} else if (si4463_Out_RxBuf[18] == 1) {
		pbuf[i16++] = 5;	// 回家
	} else if (si4463_Out_RxBuf[19] == 1) {
		pbuf[i16++] = 7;	// 热释电
	}
	pbuf[i16++] = 0;
	pbuf[i16++] = 0;
	pbuf[i16++] = 0;
	pbuf[i16++] = 0;
	pbuf[i16++] = 0;
	pbuf[i16++] = 0;
	pbuf[i16++] = 0;
	//校验
	crc16 = Count_CRC16(pbuf, i16);
	pbuf[i16++] = (uint8_t) (crc16);
	pbuf[i16++] = (uint8_t) (crc16 >> 8);
	//
	si4463_Out_RxBuf[0] = 0;
	//
	espconn_sent(&Module_UdpClient_1_espconn, pbuf, i16);
	//
	//MemManager_Free(E_MEM_MANAGER_TYPE_256B,pbuf);
}
