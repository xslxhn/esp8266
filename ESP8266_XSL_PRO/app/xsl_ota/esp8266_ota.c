#include "esp8266_ota.h"

#include "esp_common.h"

#include "lwip/mem.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ota_config.h"

ota_download_CallBack fun_call_back;

LOCAL uint32 totallength = 0;
LOCAL uint32 sumlength = 0;
LOCAL bool flash_erased = false;

void system_ota_register_callBack(ota_download_CallBack callBack) {
	fun_call_back = callBack;
	printf("system_ota_register_callBack !\n");
}

LOCAL void upgrade_recycle(void) {

	if (system_upgrade_flag_check() == UPGRADE_FLAG_FINISH) {
		totallength = 0;
		sumlength = 0;
		flash_erased = false;
		fun_call_back(100, OTA_SUCCEED);
	} else if (system_upgrade_flag_check() == UPGRADE_FLAG_IDLE) {
		printf("UPGRADE_FLAG_IDLE..\n");
	}
}

LOCAL void ota_start_download(int sta_socket, char *pusrdata, unsigned short length) {

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
		printf("ota_start_download:upgrade file download pusrdata:\n%s\n\n",
				pusrdata);
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
					// -----擦除
					if (false == system_upgrade(pusrdata, sumlength)) {
						system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
						goto ota_recycle;
					}
					flash_erased = true;
					// -----
					// -----写入数据
					if (length != 0) {
						ptr = (char *) strstr(pusrdata, "\r\n\r\n");
						if (false == system_upgrade(ptr + 4, length)) {
							system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
							goto ota_recycle;
						}
						totallength += length;
					}
					// -----
					//-----XSL
					printf("PL: %d , download now : %d , but total: %d\n",
							length,totallength, sumlength);
					//-----
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
				length,totallength, sumlength, projress);
		pusrdata[length]=0;
		printf("%s\n",pusrdata);
		//-----
		fun_call_back(projress, OTA_DOWNLOADING);
		if (false == system_upgrade(pusrdata, length)) {
			system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
			goto ota_recycle;
		}
		// 校验文件长度
		if (totallength == sumlength) {

			printf("upgrade file download finished.\n");
			if (upgrade_crc_check(system_get_fw_start_sec(), sumlength) != true) {
				printf("upgrade crc check failed !\n");
				system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
				goto ota_recycle;
			}

			system_upgrade_flag_set(UPGRADE_FLAG_FINISH);
			printf("Ok ota recycle\n");
			close(sta_socket);
			upgrade_recycle();

			return;

		} else {
			//下载中
			//printf("totallength != sumlength \n");
			return;
		}
	}

	ota_recycle: printf("go to ota recycle\n");
	close(sta_socket);
	upgrade_recycle();

}
/*
 * 函数功能	：	应用层调用用的启动升级程序
 * 参数		：	domainName		-->	ip or domain
 * 				requestResource	-->	file path
 * 				port			-->	port
 * 				isDNS			-->	true(IsDNS)	false(NoDNS)
 * 				isGet			-->	true/flase
 * */
bool system_ota_config_start(char *domainName, char* requestResource, int port,
bool isDNS, bool isGet) {

	bool isSucceccful = true;

	// 系统函数---设置系统升级标识
	system_upgrade_flag_set(UPGRADE_FLAG_START);
	// 用户函数---系统更新初始化-->初始化结构体 upgrade
	system_upgrade_init();

	int recbytes;
	int read_len;
	int sta_socket;
	char recv_buf[1460];
	const char* user_bin;
	struct sockaddr_in remote_ip;
	printf("system_ota_config_start: creat socket ! \r\n");
	//
	while (1) {
		// 1,建立socket
		sta_socket = socket(PF_INET, SOCK_STREAM, 0);
		if (-1 == sta_socket) {
			close(sta_socket);
			printf("system_ota_config_start: creat socket fail !\r\n");
			isSucceccful = false;
			continue;
		}
		printf("system_ota_config_start: creat socket ok!\r\n");
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
						"system_ota_config_start:dns ok , net <%s> 'ip is:%s \r\n",
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
			printf("system_ota_config_start:connect fail!\r\n");
			isSucceccful = false;
			system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
			upgrade_recycle();
		}
		printf("system_ota_config_start:connect ok!\r\n");
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
			printf("system_ota_config_start: send fail...\n");
			isSucceccful = false;
			free(pbuf);
			system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
			break;
		}
		printf("system_ota_config_start: send success...\n");
		free(pbuf);
		// 6,socket读取
		read_len=1460;
		while ((recbytes = read(sta_socket, recv_buf, read_len)) >= 0) {
			if (recbytes != 0) {
				ota_start_download(sta_socket, recv_buf, recbytes);
			}
			//-----XSL-----add
			else {
				printf("recbytes:%d\n", recbytes);
				//break;
				vTaskDelay(100);
			}
			if((sumlength-totallength)<read_len)
			{
				read_len=sumlength-totallength;
			}
			//-------------
		}
		// 7,容错,失败报错
		printf("system_ota_config_start:recbytes = %d\n", recbytes);

		if (recbytes < 0) {
			printf("system_ota_config_start: read data fail!\r\n");
			isSucceccful = false;
			close(sta_socket);
			system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
			break;
		}
	}

	return isSucceccful;
}
