#include "esp_common.h"
#include "espconn.h"//TCP连接需要的头文件

//-----XSL-----未测试
#define os_memcpy memcpy
#define os_strcpy strcpy
//-------------
struct espconn Module_UdpService_1_espconn;

static void Module_UdpService_1_Receive_Cb(void *arg, char *pdata,
		unsigned short len) {  //接收
	printf("Rx：%s\r\n", pdata); // %s,用来输出一个字符串
	espconn_sent((struct espconn *) arg, "Rx OK!", strlen("Rx OK!"));
}
static void Module_UdpService_1_Send_Cb(void *arg) {  //发送
	os_printf("\r\nSend OK!\r\n");

}

static void Module_UdpService_1_Init(int32_t Remote_port, uint32_t Local_port) {
	Module_UdpService_1_espconn.proto.udp = (esp_udp *) os_zalloc(sizeof(esp_udp));  //分配空间
	Module_UdpService_1_espconn.type = ESPCONN_UDP;  //设置类型为UDP协议
	Module_UdpService_1_espconn.proto.udp->local_port = Local_port;  //本地端口
	Module_UdpService_1_espconn.proto.udp->remote_port = Remote_port;  //目标端口

	espconn_regist_recvcb(&Module_UdpService_1_espconn, Module_UdpService_1_Receive_Cb); //接收
	espconn_regist_sentcb(&Module_UdpService_1_espconn, Module_UdpService_1_Send_Cb);  //发送
	espconn_create(&Module_UdpService_1_espconn);  //建立UDP传输
}
/*
void WIFI_Init() {
	struct softap_config apConfig;

	wifi_set_opmode(0x02);  //设置为AP模式，并保存到 flash

	apConfig.ssid_len = 10;				        //设置ssid长度
	os_strcpy(apConfig.ssid, "meizu");	//设置ssid名字
	os_strcpy(apConfig.password, "12345678");	//设置密码
	apConfig.authmode = 3;                      //设置加密模式
	apConfig.beacon_interval = 100;            //信标间隔时槽100 ~ 60000 ms
	apConfig.channel = 1;                      //通道号1 ~ 13
	apConfig.max_connection = 4;               //最大连接数
	apConfig.ssid_hidden = 0;                  //隐藏SSID

	wifi_softap_set_config(&apConfig);		//设置 WiFi soft-AP 接口配置，并保存到 flash
}
*/
void Module_UdpService_Init()
{
	os_printf("\r\n Module_UdpService_Init ... \r\n");
	//WIFI_Init();
	Module_UdpService_1_Init(8266, 8266);
}

