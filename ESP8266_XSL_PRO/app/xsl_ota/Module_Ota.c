/**
 ******************************************************************************
 * @file    Module_Ota.c
 * @author  徐松亮 许红宁(5387603@qq.com)
 * @version V1.0.0
 * @date    2018/01/01
 ******************************************************************************
 * @attention
 * 分层说明
 * 	Module_Ota
 * 		esp8266_ota
 * GNU General Public License (GPL)
 *
 * <h2><center>&copy; COPYRIGHT 2017 XSLXHN</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Module_Ota.h"
#include "../xsl_ota/esp8266_ota.h"
#include "../xsl_ota/upgrade.h"
#include "lwip/err.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*
 * 函数功能: FOAT
 * */
void ota_call_back(int progress, ota_code code) {

	//下载中
	if (code == OTA_DOWNLOADING) {
		printf("ota_DowmLoading: %d%% \n", progress);
		//下载完毕
	} else if (code == OTA_SUCCEED) {
		printf("OTA_SUCCEED! \n");
		vTaskDelay(1000);
		// 都重启系统了，删除任务干嘛
		//vTaskDelete(ota_task_handle);
		// 看你需要，下载完毕是否重启系统
		system_upgrade_deinit();
		system_upgrade_reboot();
	}
}
void FotaBegin_tast() {
	//如果状态正确，证明已经成功连接到路由器
	while (wifi_station_get_connect_status() != STATION_GOT_IP) {
		vTaskDelay(100);
	}
	//获取信息文件
	if (true != Module_HttpApp_GetOtaInfo("172.27.35.1", "xsl/info.json", 8080,
	false, true)) {
		printf("Module_HttpApp_GetOtaInfo: false\n");
		while (1) {
			vTaskDelay(100);
		}
	}
	//注册回调函数
	system_ota_register_callBack(ota_call_back);
	//如果当前是处于 user1.bin，那么就拉取云端升级
	if (system_upgrade_userbin_check() == UPGRADE_FW_BIN1) {
		printf("FotaBegin_tast:now is user1.bin\n");
#if 	(PC_LINK==PC_XSL)
		system_ota_config_start("172.27.35.1", "xsl/user2.4096.new.4.bin", 8080,
		false, true);		//注意像这个ip地址是不用DNS解析的，所以为 false
#elif	(PC_LINK==PC_XHN)
				system_ota_config_start("lnmodern.vicp.net", "xsl/user2.4096.new.4.bin",
						8080, true, true);		//注意像这个ip地址是不用DNS解析的，所以为 false
#endif
	} else if (system_upgrade_userbin_check() == UPGRADE_FW_BIN2) {
		printf("FotaBegin_tast:now is user2.bin.\n");
#if 	(PC_LINK==PC_XSL)
		system_ota_config_start("172.27.35.1", "xsl/user1.4096.new.4.bin", 8080,
		false, true);		//注意像这个ip地址是不用DNS解析的，所以为 false
#elif	(PC_LINK==PC_XHN)
				system_ota_config_start("lnmodern.vicp.net", "xsl/user1.4096.new.4.bin",
						8080, true, true);
#endif
	} else {
		printf("FotaBegin_tast:now is not support FOTA..\n");
	}
	for (;;) {
	}
}
void Module_Ota_Init(void) {
#if	(MODULE_OTA_ENABLE==1)
	xTaskCreate(FotaBegin_tast, "fota_task", 1024, NULL, 1, NULL);
#else
#endif
}
/******************* (C) COPYRIGHT 2011 XSLXHN *****END OF FILE****/
