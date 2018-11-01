/**
 ******************************************************************************
 * @file    Module_HttpApp.c
 * @author  徐松亮 许红宁(5387603@qq.com)
 * @version V1.0.0
 * @date    2018/01/01
 ******************************************************************************
 * @attention
 *
 * GNU General Public License (GPL)
 *
 * <h2><center>&copy; COPYRIGHT 2017 XSLXHN</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Module_HttpApp.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*
 * 函数功能: 天气预报(基于"心知天气"网站)
 * */
uint8_t Module_HttpApp_GetWeather(void) {
	uint8_t getState;
	uint8_t res = OK;
	//查询wifi
	getState = wifi_station_get_connect_status();
	//如果状态正确，证明已经成功连接到路由器
	if (getState == STATION_GOT_IP) {
		startHttpQuestByGET(
				"https://api.seniverse.com/v3/weather/daily.json?key=rrpd2zmqkpwlsckt&location=guangzhou&language=en&unit=c&start=0&days=3");
		res = OK;
	} else {
		res = FAIL;
	}
	return res;
}
/*
 * 函数功能: 要升级文件的信息
 * */
#include "esp8266_ota.h"

#include "esp_common.h"

#include "lwip/mem.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ota_config.h"

#include "esp_libc.h"

//处理过的数据计数
LOCAL uint32 totallength = 0;
//提取文件头的数据长度
LOCAL uint32 sumlength = 0;

LOCAL void upgrade_recycle(void) {
	return;
}

LOCAL void Module_HttpApp_GetOtaInfo_Parse(int sta_socket, char *pusrdata,
		unsigned short length, uint8_t *pext_buf) {

	char *ptr = NULL;
	char *ptmp2 = NULL;
	char lengthbuffer[32];

	//判断 "Accept-Length" 或者 "Context-Length"
	bool isAccept = false;
	if ((ptr = (char *) strstr(pusrdata, "Accept-Length")) != NULL)
		isAccept = true;
	else
		isAccept = false;
	// 首次进入
	if (totallength
			== 0&& (ptr = (char *)strstr(pusrdata, "\r\n\r\n")) != NULL) {
		// -----长度校正---去掉头信息
		ptr = (char *) strstr(pusrdata, "\r\n\r\n");
		// -----xsl
		printf("xsltest length: %d , ptr: %d , pusrdata:%d , totallength:%d\n",
				length, ptr, pusrdata, totallength);
		// -----
		length -= ptr - pusrdata;
		length -= 4;
		// -----
		// -----打印出调试信息
		printf("file download pusrdata:\n");
		printf(pusrdata);
		printf("\n");
		// -----
		// -----获取文件总长度
		// 返回"Accept-Length: "在pusrdata中首次出现的地址
		if (isAccept)
			ptr = (char *) strstr(pusrdata, "Accept-Length: ");
		else
			ptr = (char *) strstr(pusrdata, "Content-Length: ");

		if (ptr != NULL) {

			if (isAccept)
				ptr += 15;
			else
				ptr += 16;

			ptmp2 = (char *) strstr(ptr, "\r\n");

			if (ptmp2 != NULL) {

				//打印下收到的数据头
				//printf("ptr = %s \n", ptr);

				//-----获取到文件长度
				memset(lengthbuffer, 0, sizeof(lengthbuffer));
				memcpy(lengthbuffer, ptr, ptmp2 - ptr);
				sumlength = atoi(lengthbuffer);
				//-----
				if (sumlength > 0) {
					//-----数据处理
					//printf(pusrdata);
					ptr = (char *) strstr(pusrdata, "\r\n\r\n");
					memcpy(pext_buf, &ptr[4], length);
					//-----
					if (length != 0) {
						totallength += length;
					}
					//-----
					printf("PL: %d , download now : %d , but total: %d\n",
							length, totallength, sumlength);
					//-----
					if (totallength == sumlength) {
						close(sta_socket);
						upgrade_recycle();
					}
					return;
				}

			} else {
				system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
				goto ota_recycle;
			}

		} else {
			system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
			goto ota_recycle;
		}

	}
	// 非首次进入
	else {

		totallength += length;

		int part = sumlength / 100;
		int projress = (totallength / part);
		//-----XSL
		printf("PL: %d , download now : %d , but total: %d , projress: %d\% \n",
				length, totallength, sumlength, projress);
		pusrdata[length] = 0;
		printf("%s\n", pusrdata);
		//-----数据处理
		//
		// 校验文件长度
		if (totallength == sumlength) {
			close(sta_socket);
			upgrade_recycle();
			return;
		} else {
			return;
		}
	}
	ota_recycle: printf("go to ota recycle\n");
	close(sta_socket);
	upgrade_recycle();

}

uint8_t Module_HttpApp_GetOtaInfo(char *domainName, char* requestResource,
		int port, bool isDNS, bool isGet) {
	//
	uint8_t isSucceccful = true;

	int sta_socket;
	struct sockaddr_in remote_ip;
	int recbytes;
	int read_len;
	static char recv_buf[1460];
	//如果状态正确，证明已经成功连接到路由器
	if (wifi_station_get_connect_status() != STATION_GOT_IP) {
		isSucceccful = false;
		return isSucceccful;
	}
	//
	while (1) {
		// 1,建立socket
		sta_socket = socket(PF_INET, SOCK_STREAM, 0);
		if (-1 == sta_socket) {
			close(sta_socket);
			printf("Module_HttpApp_GetOtaInfo: creat socket fail !\r\n");
			isSucceccful = false;
			continue;
		}
		printf("Module_HttpApp_GetOtaInfo: creat socket ok!\r\n");
		// 2,域名DNS处理(域名--->IP)
		if (isDNS) {
			//dns begin
			dns_init();
			struct ip_addr addr;
			char name0[48];
			sprintf(name0, "%s", domainName);
			err_t err = netconn_gethostbyname((char*) name0, &addr);
			char* ipAdress;
			if (err == ERR_OK) {
				ipAdress = ip_ntoa(&addr);
				printf(
						"Module_HttpApp_GetOtaInfo:dns ok , net <%s> 'ip is:%s \r\n",
						name0, ipAdress);
			} else {
				isSucceccful = false;
				break;
			}
			bzero(&remote_ip, sizeof(struct sockaddr_in));
			remote_ip.sin_family = AF_INET;
			remote_ip.sin_addr.s_addr = inet_addr(ipAdress);
			remote_ip.sin_port = htons(port);
			//dns over
		} else {
			bzero(&remote_ip, sizeof(struct sockaddr_in));
			remote_ip.sin_family = AF_INET;
			remote_ip.sin_addr.s_addr = inet_addr(domainName);
			remote_ip.sin_port = htons(port);
		}
		// 3,socket连接
		if (0
				!= connect(sta_socket, (struct sockaddr * ) (&remote_ip),
						sizeof(struct sockaddr))) {
			close(sta_socket);
			printf("Module_HttpApp_GetOtaInfo:connect fail!\r\n");
			isSucceccful = false;
			//upgrade_recycle();
		}
		printf("Module_HttpApp_GetOtaInfo:connect ok!\r\n");
		// 4,POST or GET 分类处理
		char *pbuf = (char *) zalloc(1024);
		if (isGet)
			sprintf(pbuf, "GET /%s HTTP/1.0\r\nHost: \%s\r\n"pheadbuffer"",
					requestResource, domainName);
		else
			sprintf(pbuf, "POST /%s HTTP/1.0\r\nHost: \%s\r\n"pheadbuffer"",
					requestResource, domainName);
		// 信息打印
		printf("----request url begin----------------------\r\n\r\n");
		printf(pbuf);
		printf("-- -request url over----------------------\r\n\r\n");
		// 5,socket写入
		if (write(sta_socket, pbuf, strlen(pbuf) + 1) < 0) {
			close(sta_socket);
			printf("Module_HttpApp_GetOtaInfo: send fail...\n");
			isSucceccful = false;
			free(pbuf);
			break;
		}
		printf("Module_HttpApp_GetOtaInfo: send success...\n");
		free(pbuf);
		// 6,socket读取
		uint8_t *pext_buf = (uint8_t *) zalloc(1024);
		read_len = 1460;
		while ((recbytes = read(sta_socket, recv_buf, read_len)) >= 0) {
			if (recbytes != 0) {
				// 解析
				Module_HttpApp_GetOtaInfo_Parse(sta_socket, recv_buf, recbytes,
						pext_buf);
				printf("sumlength:%d totallength:%d\n", sumlength, totallength);
			}
			//-----XSL-----add
			else {
				printf("recbytes:%d sumlength:%d totallength:%d\n", recbytes,
						sumlength, totallength);
				//break;
				vTaskDelay(100);
			}
			if (sumlength == totallength) {
				break;
			} else if ((sumlength - totallength) < read_len) {
				read_len = sumlength - totallength;
			}
			//-------------
		}
		// 7,解析数据
		{
			uint8_t i = 10;
			while (i) {
				i--;
				vTaskDelay(100);
				//查看跳出条件
				if (totallength != 0) {
					printf("-----Parse data-----Begin\n");
					printf(pext_buf);

					{
						uint8_t *pi;
						int ver = 0, subver = 0;
						while (1) {
							// 硬件版本验证
							pi = strstr(pext_buf, "hardware_ver");
							if (pi != NULL) {
								ver = atoi((const char *) &pi[14]);
							} else {
								isSucceccful = false;
								break;
							}
							pi = strstr(pext_buf, "hardware_subver");
							if (pi != NULL) {
								subver = atoi((const char *) &pi[17]);
							} else {
								isSucceccful = false;
								break;
							}
							printf(
									"\nHARDWARE_VER:%d,HARDWARE_SUB_VER:%d,ver:%d,subver:%d\n",
									HARDWARE_VER, HARDWARE_SUB_VER, ver,
									subver);
							if ((ver != HARDWARE_VER)
									|| (subver != HARDWARE_SUB_VER)) {
								isSucceccful = false;
								break;
							}
							// 软件版本验证
							pi = strstr(pext_buf, "software_ver");
							if (pi != NULL) {
								ver = atoi((const char *) &pi[14]);
							} else {
								isSucceccful = false;
								break;
							}
							pi = strstr(pext_buf, "software_subver");
							if (pi != NULL) {
								subver = atoi((const char *) &pi[17]);
							} else {
								isSucceccful = false;
								break;
							}
							printf(
									"SOFTWARE_VER:%d,SOFTWARE_SUB_VER:%d,ver:%d,subver:%d\n",
									SOFTWARE_VER, SOFTWARE_SUB_VER, ver,
									subver);
							if ((ver == SOFTWARE_VER)
									&& (subver == SOFTWARE_SUB_VER)) {
								isSucceccful = false;
								break;
							} else {
								isSucceccful = true;
								break;
							}
						}
					}
					printf("-----Parse data-----End\n");
					break;
				} else {
					if (i == 0) {
						isSucceccful = false;
					}
				}
			}
		}
		free(pext_buf);
		// 8,结束
		printf("Module_HttpApp_GetOtaInfo:recbytes = %d,isSucceccful=%d\n",
				recbytes, isSucceccful);
		totallength = 0;
		sumlength = 0;
		break;
	}
	return isSucceccful;
}
/******************* (C) COPYRIGHT 2011 XSLXHN *****END OF FILE****/
