#ifndef __UPGRADE_XUHONG__
#define __UPGRADE_XUHONG__



#include "esp_common.h"


#define SPI_FLASH_SEC_SIZE      4096

#define USER_BIN1               0x00
#define USER_BIN2               0x01

#define UPGRADE_FLAG_IDLE       0x00
#define UPGRADE_FLAG_START      0x01
#define UPGRADE_FLAG_FINISH     0x02

#define UPGRADE_FW_BIN1         0x00
#define UPGRADE_FW_BIN2         0x01

typedef enum {
	OTA_FAIL = 0,    //下载失败，可能是网速问题
	OTA_SUCCEED ,  //下载并且烧录成功，请重启系统
	OTA_DOWNLOADING ,  //下载中，可看进度条
	OTA_INIT_ERROR ,	 //初始化失败，网址有误。
	OTA_INIT_OK ,	 //初始化成功。
}ota_code;

//progress是进度条，code是状态码
typedef void (*ota_download_CallBack)(int progress,ota_code code);

//注册下载服务器bin文件的进度百分比回调函数：0到100% （整型）;
void system_ota_register_callBack(ota_download_CallBack callBack);

/******************************************************************************
 * FunctionName : system_ota_config_start
 * Description  : 传入网址,并且开始下载
 * Parameters   : char *domainName : 域名或ip地址
 *                char* requestResource : 请求的资源地址
 *                int port         : 端口号
 *                bool isDNS       : 是否要DNS解析
 *                bool isGet       : get请求或者post提交
 * Returns      : 是否成功
 *******************************************************************************/
bool system_ota_config_start(char *domainName, char* requestResource,int port,
		bool isDNS, bool isGet);


#endif
