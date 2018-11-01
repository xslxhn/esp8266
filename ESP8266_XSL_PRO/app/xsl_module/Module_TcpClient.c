#include "esp_common.h"
#include "espconn.h"//TCP连接需要的头文件
//-----XSL-----
#define os_memcpy memcpy
#define os_strcpy strcpy
//-------------
static os_timer_t Module_TcpClient_1_Timer;
static struct espconn Module_TcpClient_1_espconn;

//发送
void ICACHE_FLASH_ATTR Module_TcpClient_1_Send_Cb(void *arg)
{
	os_printf("TcpClient send data OK!");
}
//断开
void ICACHE_FLASH_ATTR Module_TcpClient_1_Disconnect_Cb(void *arg)
{
	os_printf("TcpClient CloseConnect OK!");
}
//接收
void ICACHE_FLASH_ATTR Module_TcpClient_1_Receive_Cb(void *arg,
		char *pdata, unsigned short len) {

	os_printf("TcpClient Rx:%s\r\n", pdata);
	espconn_sent((struct espconn *) arg, "0", strlen("0"));

}
//注册 TCP 连接发生异常断开时的回调函数，可以在回调函数中进行重连
void ICACHE_FLASH_ATTR Module_TcpClient_1_Reconnect_Cb(void *arg, sint8 err)
{
	os_printf("Connect Err,Err Code:%d\r\n", err);
	espconn_connect((struct espconn *) arg);
}
//注册 TCP 连接成功建立后的回调函数
void ICACHE_FLASH_ATTR Module_TcpClient_1_Connect_Cb(void *arg)
{
	struct espconn *pespconn = arg;
	espconn_regist_recvcb(pespconn, Module_TcpClient_1_Receive_Cb);
	espconn_regist_sentcb(pespconn, Module_TcpClient_1_Send_Cb);
	espconn_regist_disconcb(pespconn, Module_TcpClient_1_Disconnect_Cb);
	espconn_sent(pespconn, "8226", strlen("8226"));

}

void ICACHE_FLASH_ATTR Module_TcpClient_1_Init(struct ip_addr *remote_ip,
		struct ip_addr *local_ip, int remote_port) {
	Module_TcpClient_1_espconn.proto.tcp = (esp_tcp *) os_zalloc(sizeof(esp_tcp));  //分配空间
	Module_TcpClient_1_espconn.type = ESPCONN_TCP;  //设置类型为TCP协议
	os_memcpy(Module_TcpClient_1_espconn.proto.tcp->local_ip, local_ip, 4);
	os_memcpy(Module_TcpClient_1_espconn.proto.tcp->remote_ip, remote_ip, 4);
	Module_TcpClient_1_espconn.proto.tcp->local_port = espconn_port();  //本地端口
	Module_TcpClient_1_espconn.proto.tcp->remote_port = remote_port;  //目标端口
	//注册连接成功回调函数和重新连接回调函数
	//注册 TCP 连接成功建立后的回调函数
	espconn_regist_connectcb(&Module_TcpClient_1_espconn, Module_TcpClient_1_Connect_Cb);
	//注册 TCP 连接发生异常断开时的回调函数，可以在回调函数中进行重连
	espconn_regist_reconcb(&Module_TcpClient_1_espconn, Module_TcpClient_1_Reconnect_Cb);
	//启用连接
	espconn_connect(&Module_TcpClient_1_espconn);
}

static void Module_TcpClient_TimerFun(void) {
	uint8 getState;
	getState = wifi_station_get_connect_status();
	//查询 ESP8266 WiFi station 接口连接 AP 的状态
	if (getState == STATION_GOT_IP) {
		os_printf("WIFI connect ok！\r\n");
		os_timer_disarm(&Module_TcpClient_1_Timer);
		struct ip_info info;
		//目标IP地址,必须要先从手机获取，否则连接失败.
		const char remote_ip[4] = { 10, 10, 18, 202 };
		//查询 WiFi模块的 IP 地址
		wifi_get_ip_info(STATION_IF, &info);
		//连接到目标服务器的6000端口
		Module_TcpClient_1_Init((struct ip_addr *) remote_ip, &info.ip, 8080);
	}
}

void Module_TcpClient_Init()	//初始化
{
	/*
	 wifi_set_opmode(0x01);	//设置为STATION模式

	 struct station_config stationConf;
	 os_strcpy(stationConf.ssid, "meizu");	  //改成你自己的   路由器的用户名
	 os_strcpy(stationConf.password, "12345678"); //改成你自己的   路由器的密码
	 wifi_station_set_config(&stationConf);	//设置WiFi station接口配置，并保存到 flash
	 wifi_station_connect();	//连接路由器
	 */
	os_timer_disarm(&Module_TcpClient_1_Timer);	//取消定时器定时
	os_timer_setfn(&Module_TcpClient_1_Timer,
			(os_timer_func_t *) Module_TcpClient_TimerFun,
			NULL);	//设置定时器回调函数
	os_timer_arm(&Module_TcpClient_1_Timer, 500, 1);	//启动定时器，单位：毫秒
}

