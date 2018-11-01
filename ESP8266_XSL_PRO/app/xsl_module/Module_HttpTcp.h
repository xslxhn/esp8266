/*
 * client.h
 *
 *  Created on: 2015Äê7ÔÂ22ÈÕ
 *      Author: Administrator
 */

#ifndef APP_XSL_MODULE_MODULE_HTTPTCP_H_
#define APP_XSL_MODULE_MODULE_HTTPTCP_H_
#include "user_main.h"
#include "espconn.h"
char buffer[1024];
#define GET "GET /%s HTTP/1.1\r\nContent-Type: text/html;charset=utf-8\r\nAccept: */*\r\nHost: %s\r\nConnection: Keep-Alive\r\n\r\n"
#define POST "POST /%s HTTP/1.1\r\nAccept: */*\r\nContent-Length: %d\r\nContent-Type: application/json\r\nHost: %s\r\nConnection: Keep-Alive\r\n\r\n%s"
struct espconn user_tcp_conn;
void my_station_init(struct ip_addr *remote_ip,struct ip_addr *local_ip,int remote_port);

void ICACHE_FLASH_ATTR my_station_disinit(void);

#endif /* APP_XSL_MODULE_MODULE_HTTPTCP_H_ */
