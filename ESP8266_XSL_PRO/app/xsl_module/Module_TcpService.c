#include "esp_common.h"
#include "espconn.h"//TCP连接需要的头文件
//-----XSL-----
#define os_memcpy memcpy
#define os_strcpy strcpy
//-------------
struct espconn Module_TcpService_1_espconn;

void ICACHE_FLASH_ATTR Module_TcpService_1_Receive_Cb(void *arg, char *pdata,
		unsigned short len) {
	os_printf("TcpService Rx:%s", pdata);
	espconn_sent((struct espconn *) arg, "Rx OK!", strlen("Rx OK!"));

}
void ICACHE_FLASH_ATTR Module_TcpService_1_Send_Cb(void *arg) {
	os_printf("TcpService Send OK!");
}
void ICACHE_FLASH_ATTR Module_TcpService_1_Disconnect_Cb(void *arg) {
	os_printf("TcpService CloseConnect OK!");
}

void ICACHE_FLASH_ATTR Module_TcpService_1_Listen_Cb(void *arg)  //注册 TCP 连接成功建立后的回调函数
{
	struct espconn *pespconn = arg;
	//接收
	espconn_regist_recvcb(pespconn, Module_TcpService_1_Receive_Cb);
	//发送
	espconn_regist_sentcb(pespconn, Module_TcpService_1_Send_Cb);
	//断开
	espconn_regist_disconcb(pespconn, Module_TcpService_1_Disconnect_Cb);
}
//注册 TCP 连接发生异常断开时的回调函数，可以在回调函数中进行重连
void ICACHE_FLASH_ATTR Module_TcpService_1_Reconnect_Cb(void *arg, sint8 err)
{
	os_printf("TcpService Connect Err,Err code:%d\r\n", err); //%d,用来输出十进制整数
}

static void Module_TcpService_1_Init(uint32_t Local_port) {
	Module_TcpService_1_espconn.proto.tcp = (esp_tcp *) os_zalloc(sizeof(esp_tcp)); //分配空间
	Module_TcpService_1_espconn.type = ESPCONN_TCP; //设置类型为TCP协议
	Module_TcpService_1_espconn.proto.tcp->local_port = Local_port; //本地端口

	espconn_regist_connectcb(&Module_TcpService_1_espconn, Module_TcpService_1_Listen_Cb); //注册 TCP 连接成功建立后的回调函数
	espconn_regist_reconcb(&Module_TcpService_1_espconn, Module_TcpService_1_Reconnect_Cb); //注册 TCP 连接发生异常断开时的回调函数，可以在回调函数中进行重连
	espconn_accept(&Module_TcpService_1_espconn); //创建 TCP server，建立侦听
	espconn_regist_time(&Module_TcpService_1_espconn, 180, 0); //设置超时断开时间 单位：秒，最大值：7200 秒

}
/*
 void WIFI_Init() {
 struct softap_config apConfig;
 wifi_set_opmode(0x02);    //设置为AP模式，并保存到 flash
 apConfig.ssid_len = 10;						//设置ssid长度
 os_strcpy(apConfig.ssid, "xuhongLove");	    //设置ssid名字
 os_strcpy(apConfig.password, "12345678");	//设置密码
 apConfig.authmode = 3;                      //设置加密模式
 apConfig.beacon_interval = 100;            //信标间隔时槽100 ~ 60000 ms
 apConfig.channel = 1;                      //通道号1 ~ 13
 apConfig.max_connection = 4;               //最大连接数
 apConfig.ssid_hidden = 0;                  //隐藏SSID

 wifi_softap_set_config(&apConfig);		//设置 WiFi soft-AP 接口配置，并保存到 flash
 }
 */

void Module_TcpService_Init()		//初始化
{
	Module_TcpService_1_Init(8266);		//本地端口
}

