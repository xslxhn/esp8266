/* @file    Module_cJSON.c
 * @author  徐松亮 许红宁(5387603@qq.com)
 * @version V1.0.0
 * @date    2018/01/01
 ******************************************************************************
 * @attention
 * 1,相关官网
 * 		1.1		cJSON下载
 * 			https://github.com/DaveGamble/cJSON
 * 		1.2		JSON数据格式验证
 * 			https://www.json.cn/
 * 2,cJSON库常见的函数说明
 * 		2.1	 	cJSON_Parse( const char *value );
 * 			开始识别一个字符串是否为json格式，返回数值就是一个cJSON，
 * 			如果返回的cJSON为NULL则表示不是json格式，否则就是json格式。
 * 		2.2		cJSON_GetObjectItem( const cJSON * const object, const char * const string );
 * 			从一个json格式数据剖析一个指定的字段的数据，返回数值就是一个cJSON,
 * 			如果返回的cJSON为NULL则表示获取失败，否则获取成功！
 *		2.3		cJSON_IsString(const cJSON * const item);
 *			从一个json格式数据判断是否为字符串类型，返回数值就是一个布尔值，
 *			如果返回false则表示这个字段是字符串类型，否则不是！
 *		2.4		cJSON_IsBool()
 *			从一个json格式数据判断是否为布尔值，返回数值就是一个布尔值，
 *			如果返回false则表示这个字段是布尔值，否则不是！
 *		2.5		cJSON_IsNumber()
 *			从一个json格式数据判断是否为数值，返回数值就是一个布尔值，
 *			如果返回false则表示这个字段是数值，否则不是！
 *		2.6		cJSON_GetArraySize(const cJSON *array);
 *			从一个json格式的数组数据获取该数组的长度。
 *		2.7		cJSON_GetArrayItem(const cJSON *array, int index);
 *			从一个json格式的数组获取该数组的指定索引的元素，index是下标。
 *		2.8		cJSON_Print(const cJSON *item);
 *			返回一个json格式的数据，返回时char*类型，注意使用后要释放内存。调用cJSON_free()即可
 *		2.9		cJSON_Delete(cJSON *c);
 *			释放一个json格式的数据所占的内存，注意一般使用这个数据之后，释放根节点即可！！
 *		2.10	cJSON_AddStringToObject(object,name,s);
 *			新增一个字符串类型字段到json格式的数据！name是字段，s是数值。
 *		2.11	cJSON_AddItemToObject(cJSON *object, const char *string, cJSON *item);
 *			新增一个新的子节点cJSON到根节点！string是字段名字！
 *	3,cJSON注意事项
 *		3.1		注意每次使用（包括剖析和生成）之后，都要记得释放内存，就是调用cJSON_Delete()方法，
 *				否则一直在占据内存，像8266这样的本身内存就少，运行久了就会死机的 ！
 *		3.2		在剖析数据时候，一定要遵循规范，一定要判断是否json数据，而且是否想要的类型，
 *				比如字符串，整型，这些都是要认真核对。否则会造成esp8266重启。
 *		3.3		在处理数组时候，一定要注意不要数组越界问题！最后，祝大家养成好的编程习惯，受益终生。
 *
 * GNU General Public License (GPL)
 *
 * <h2><center>&copy; COPYRIGHT 2017 XSLXHN</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Module_cJSON.h"
#include "cJSON.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*
 * 函数功能:
 * */
void Module_cJSON_ParseJson(void) {
	/*
	 //解析以下一段json数据
	 {
	 "mac":"84:f3:eb:b3:a7:05",
	 "number":2,
	 "value":{"name":"半颗心脏",
	 "age":18 ,
	 "blog":"https://blog.csdn.net/xh870189248"
	 },
	 "hex":[51,15,63,22,96]
	 }
	 */

	u8* jsonRoot =\

			"{\"mac\":\"12:34:56:78:90:ab\","
					"\"number\":2,"
					"\"value\":{\"name\":\"xuhongv\",\"age\":18,\"blog\":\"https://blog.csdn.net/xh870189248\"},"
					"\"hexArry\":[51,15,63,22,96]}";

	//首先整体判断是否为一个json格式的数据
	cJSON *pJsonRoot = cJSON_Parse(jsonRoot);
	//如果是否json格式数据
	if (pJsonRoot == NULL) {
		os_printf("this is not a json data ... \n");
		cJSON_Delete(pJsonRoot);
		os_printf("get freeHeap: %d \n\n", system_get_free_heap_size());
	}

	char *s = cJSON_Print(pJsonRoot);
	os_printf("pJsonRoot: %s\r\n", s);

	cJSON_free((void *) s);

	//解析mac字段字符串内容
	cJSON *pMacAdress = cJSON_GetObjectItem(pJsonRoot, "mac");
	//判断mac字段是否json格式
	if (pMacAdress) {
		//判断mac字段是否string类型
		if (cJSON_IsString(pMacAdress)) {
			os_printf("get MacAdress:%s \n", pMacAdress->valuestring);
		}

	} else {
		os_printf("get MacAdress failed \n");
	}

	//解析number字段int内容
	cJSON *pNumber = cJSON_GetObjectItem(pJsonRoot, "number");
	//判断number字段是否存在
	if (pNumber) {
		if (cJSON_IsNumber(pNumber)) {
			os_printf("get Number:%d \n", pNumber->valueint);
		}
	} else {
		os_printf("get Number failed \n");
	}

	//解析value字段内容，判断是否为json
	cJSON *pValue = cJSON_GetObjectItem(pJsonRoot, "value");
	if (pValue) {
		//进一步剖析里面的name字段:注意这个根节点是 pValue
		cJSON *pName = cJSON_GetObjectItem(pValue, "name");
		if (pName) {
			if (cJSON_IsString(pName)) {
				os_printf("get value->Name : %s \n", pName->valuestring);
			}
		}

		//进一步剖析里面的age字段:注意这个根节点是 pValue
		cJSON *pAge = cJSON_GetObjectItem(pValue, "age");
		if (pAge) {
			if (cJSON_IsNumber(pAge)) {
				os_printf("get value->Age : %d \n", pAge->valueint);
			}
		}

		//进一步剖析里面的blog字段:注意这个根节点是 pValue
		cJSON *pBlog = cJSON_GetObjectItem(pValue, "blog");
		if (pBlog) {
			if (cJSON_IsString(pBlog)) {
				os_printf("get value->pBlog	 : %s \n", pBlog->valuestring);
			}
		}
	}

	//剖析
	cJSON *pArry = cJSON_GetObjectItem(pJsonRoot, "hexArry");
	if (pArry) {
		//获取数组长度
		int arryLength = cJSON_GetArraySize(pArry);
		os_printf("get arryLength : %d \n", arryLength);
		//逐个打印
		int i;
		for (i = 0; i < arryLength; i++) {
			os_printf("cJSON_GetArrayItem(pArry, %d)= %d \n", i,
					cJSON_GetArrayItem(pArry, i)->valueint);
		}
	}
	cJSON_Delete(pJsonRoot);
	os_printf("get freeHeap: %d \n\n", system_get_free_heap_size());

}

void Module_cJSON_CreateJson() {

	/*
	 {
	 "mac":"84:f3:eb:b3:a7:05",
	 "number":2,
	 "value":{"name":"xslxhn",
	 "age":18 ,
	 "blog":"https://blog.csdn.net/xh870189248"
	 },
	 "hex":[51,15,63,22,96]
	 }
	 */

	//取一下本地的station的mac地址
	uint8 tempMessage[6]={0x12,0x34,0x56,0x78,0x90,0xab};
	//wifi_get_macaddr(STATION_IF, tempMessage);
	uint8_t mac_str_buf[20];
	sprintf(mac_str_buf,"%02X:%02X:%02X:%02X:%02X:%02X"\
			,tempMessage[0],tempMessage[1],tempMessage[2]\
			,tempMessage[3],tempMessage[4],tempMessage[5]);

	cJSON *pRoot = cJSON_CreateObject();
	cJSON *pValue = cJSON_CreateObject();

	cJSON_AddStringToObject(pRoot,"mac",mac_str_buf);
	cJSON_AddNumberToObject(pRoot,"number",2);

	cJSON_AddStringToObject(pValue,"mac","xslxhn");
	cJSON_AddNumberToObject(pValue,"age",18);
	cJSON_AddStringToObject(pValue,"mac","https://blog.csdn.net/");

	cJSON_AddItemToObject(pRoot, "value",pValue);

	int hex[5]= {51,15,63,22,96};
	cJSON *pHex = cJSON_CreateIntArray(hex,5);
	cJSON_AddItemToObject(pRoot,"hex",pHex);

	char *s = cJSON_Print(pRoot);
	os_printf("\r\n creatJson : %s\r\n", s);
	cJSON_free((void *) s);

	cJSON_Delete(pRoot);
}

/******************* (C) COPYRIGHT 2011 XSLXHN *****END OF FILE****/
