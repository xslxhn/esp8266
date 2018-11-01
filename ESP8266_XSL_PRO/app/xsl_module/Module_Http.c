/*
 * my_http.c
 *
 *  Created on: 2015年7月24日
 *      Author: Administrator
 */
#include "Module_Http.h"

#include "c_types.h"
#include "esp_libc.h"
#include "Module_HttpTcp.h"

#define uart0_sendStr printf
#define os_sprintf sprintf
//剖析URL
void ICACHE_FLASH_ATTR http_parse_request_url(char *URL, char *host,char *filename, unsigned short *port) {


	char *PA;
	char *PB;

	memset(host, 0, sizeof(host));
	memset(filename, 0, sizeof(filename));

	*port = 0;

	if (!(*URL)){
		uart0_sendStr("\r\n ----- URL return -----  \r\n");
		return;
	}
	//PA	=	https://api.seniverse.com/v3/weather/daily.json?key=rrpd2zmqkpwlsckt&location=guangzhou&language=en&unit=c&start=0&days=3
	PA = URL;

	if (!strncmp(PA, "http://", strlen("http://"))) {
		PA = URL + strlen("http://");
	} else if (!strncmp(PA, "https://", strlen("https://"))) {
		PA = URL + strlen("https://");
	}
	//PA	=	api.seniverse.com/v3/weather/daily.json?key=rrpd2zmqkpwlsckt&location=guangzhou&language=en&unit=c&start=0&days=3
	PB = strchr(PA, '/');
	//PB	=	/v3/weather/daily.json?key=rrpd2zmqkpwlsckt&location=guangzhou&language=en&unit=c&start=0&days=3
	if (PB) {
		uart0_sendStr("\r\n ----- PB=true -----  \r\n");
		//		host		=	api.seniverse.com
		memcpy(host, PA, strlen(PA) - strlen(PB));
		if (PB + 1) {
			//	filename	=	v3/weather/daily.json?key=rrpd2zmqkpwlsckt&location=guangzhou&language=en&unit=c&start=0&days=3
			memcpy(filename, PB + 1, strlen(PB - 1));
			filename[strlen(PB) - 1] = 0;
		}
		host[strlen(PA) - strlen(PB)] = 0;

		uart0_sendStr(host,strlen(host));
		uart0_sendStr("\r\n");

	} else {
		uart0_sendStr("\r\n ----- PB=false -----  \r\n");
		//		host		=	api.seniverse.com
		memcpy(host, PA, strlen(PA));
		host[strlen(PA)] = 0;
		uart0_sendStr(host,strlen(host));
		uart0_sendStr("\r\n");
	}
	//
	PA = strchr(host, ':');

	if (PA){
		*port = atoi(PA + 1);
	}else{
		*port = 80;
	}
}

//寻找DNS解析，并且配置
void ICACHE_FLASH_ATTR user_esp_dns_found(const char *name, ip_addr_t *ipaddr,void *arg) {

	struct ip_info info;
	wifi_get_ip_info(STATION_IF, &info);
	my_station_init(ipaddr, &info.ip, port);

}

//定义Get请求的实现
void ICACHE_FLASH_ATTR startHttpQuestByGET(char *URL){
	struct ip_addr addr;
	memset(buffer,0,1024);
	http_parse_request_url(URL,host,filename,&port);
	os_sprintf(buffer,GET,filename,host);

	uint8_t res;
	res=espconn_gethostbyname(&user_tcp_conn,host, &addr,
	user_esp_dns_found);
	printf("espconn_gethostbyname:%d\r\n",res);
}


//定义Post请求的实现
void ICACHE_FLASH_ATTR startHttpQuestByPOST(char *URL,char *method,char *postdata){
	struct ip_addr addr;
	memset(buffer,0,1024);
	http_parse_request_url(URL,host,filename,&port);
	os_sprintf(buffer,POST,filename,strlen(postdata),host,postdata);
	espconn_gethostbyname(&user_tcp_conn,host, &addr,
	user_esp_dns_found);
}
