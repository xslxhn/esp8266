/**
  ******************************************************************************
  * @file    Module_Ota.h
  * @author  徐松亮 许红宁(5387603@qq.com)
  * @version V1.0.0
  * @date    2018/01/01
  * @brief   bottom-driven -->   LED.
  * @note
  * @verbatim

 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
   1,    适用芯片
   	   	 1.1	乐鑫      :  ESP8266
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * GNU General Public License (GPL)
  *
  * <h2><center>&copy; COPYRIGHT 2017 XSLXHN</center></h2>
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODULE_OTA_H
#define __MODULE_OTA_H
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Transplant define ---------------------------------------------------------*/
#define PC_XSL	1
#define PC_XHN	2
#define PC_LINK	PC_XSL

#if	(HARDWARE_VER==202)
#define	MODULE_OTA_ENABLE	1
#else
#define	MODULE_OTA_ENABLE	0
#endif
/* Application define --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
extern void Module_Ota_Init(void);

#endif
/************************ (C) COPYRIGHT XSLXHN *****END OF FILE****************/

