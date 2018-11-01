/*
 ***********************************************************************************
 *                    作    者: 徐松亮
 *                    更新时间: 2018-09-28
 * 当前问题: Http的应用
 ***********************************************************************************
 */
//--------------------------------
#ifndef __MODULE_HTTPAPP_H
#define __MODULE_HTTPAPP_H
//--------------------------------
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Exported types ------------------------------------------------------------*/
#if (defined(ESP8266))
#else
#error Please Set Project to Module_HttpApp.h
#endif
/* Exported constants --------------------------------------------------------*/
/* Transplant define ---------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
extern uint8_t Module_HttpApp_GetWeather(void);
extern uint8_t Module_HttpApp_GetOtaInfo(char *domainName,
		char* requestResource, int port, bool isDNS, bool isGet);
//-------------------------------------------------------------------------------
#endif
/************************ (C) COPYRIGHT XSLXHN *****END OF FILE****************/
