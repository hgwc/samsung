/* ***************************************************************************
 *
 * Copyright 2019 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
#define OTA

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "st_dev.h"
#include "device_control.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "iot_uart_cli.h"
#include "iot_cli_cmd.h"

#include "caps_switch.h"
#include "caps_switchLevel.h"
#include "caps_colorTemperature.h"
#include "caps_activityLightingMode.h"
#include "caps_dustSensor.h"

// 센도리
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "caps_fanSpeed.h"
#include "caps_mode.h"
#include "caps_dustHealthConcern.h"
#include "caps_fineDustHealthConcern.h"
#include "caps_carbonDioxideMeasurement.h"
#include "caps_carbonDioxideHealthConcern.h"
#include "caps_tvocMeasurement.h"
#include "caps_tvocHealthConcern.h"
#include "caps_relativeHumidityMeasurement.h"
#include "caps_temperatureMeasurement.h"
#include "caps_filterStatus.h"
#include  "caps_signalStrength.h"
#include "caps_panicAlarm.h"

#ifdef OTA
#include "caps_firmwareUpdate.h"
#include "ota_util.h"
#endif

#define MAIN_MSG_LENGTH     2
static const char *TAG = "MAIN";

QueueHandle_t main_queue;
extern QueueHandle_t uart_queue;
extern     void erv_main(void);
esp_timer_handle_t comm_check_timer_handler;
//--------------------------------

// onboarding_config_start is null-terminated string
extern const uint8_t onboarding_config_start[]    asm("_binary_onboarding_config_json_start");
extern const uint8_t onboarding_config_end[]    asm("_binary_onboarding_config_json_end");

// device_info_start is null-terminated string
extern const uint8_t device_info_start[]    asm("_binary_device_info_json_start");
extern const uint8_t device_info_end[]        asm("_binary_device_info_json_end");

static iot_status_t g_iot_status = IOT_STATUS_IDLE;
static iot_stat_lv_t g_iot_stat_lv;

IOT_CTX* iot_ctx = NULL;

//#define SET_PIN_NUMBER_CONFRIM

static int noti_led_mode = LED_ANIMATION_MODE_IDLE;

static caps_switch_data_t *cap_switch_data;
static caps_switchLevel_data_t *cap_switchLevel_data;
static caps_colorTemperature_data_t *cap_colorTemp_data;
static caps_activityLightingMode_data_t *cap_lightMode_data;
static caps_dustSensor_data_t *cap_dustSensor_data;

// 센도리에 맞게 dataType 추가
static caps_dustHealthConcern_data_t *cap_dustHealthConcern_data;
static caps_fineDustHealthConcern_data_t *cap_fineDustHealthConcern_data;
static caps_fanSpeed_data_t *cap_fanSpeed_data;
static caps_mode_data_t *cap_mode_data;
static caps_carbonDioxideMeasurement_data_t *cap_carbonDioxideMeasurement_data;
static caps_carbonDioxideHealthConcern_data_t *cap_carbonDioxideHealthConcern_data;
static caps_tvocMeasurement_data_t *cap_tvocMeasurement_data;
static caps_tvocHealthConcern_data_t *cap_tvocHealthConcern_data;

static caps_relativeHumidityMeasurement_data_t *cap_relativeHumidityMeasurement_data;
static caps_temperatureMeasurement_data_t *cap_temperatureMeasurement_data;
static caps_filterStatus_data_t *cap_filterStatus_data;
static caps_signalStrength_data_t *cap_signalStrength_data;
static caps_panicAlarm_data_t *cap_panicAlarm_data;

#ifdef OTA
static caps_firmwareUpdate_data_t *cap_ota_data;
TaskHandle_t ota_task_handle = NULL;
#endif
//----------------------------------
const char *supported_mode[] = {
    "energyRecoveryVentilation",
    "bypass",
    "auto",
    "cleanAir",
    "filterAutoCleaning"
};
enum {
    MODE_VENT,                      // 전열
    MODE_BYPASS,                  // 바이패스 WARM_AIR,
    MODE_AUTO,                      // 자동 DRY_HEATING,
    MODE_CLEAN_AIR,            // 청정 DRY_COOLING,
    MODE_FILTER_AUTO          // 필터청소 DEHUMIDIFY_HEATING,
                                                        // MODE_DEHUMIDIFY_COOLING,
                                                        // MODE_AUTO
};

enum {
    FILTER_NORMAL,
    FILTER_REPLACE
};

enum{
    ERV_PANIC,
    ERV_CLEAR
};

int rssi_period_ms = 20;        //30000;
//--------

int monitor_enable = false;
int monitor_period_ms = 30000;
int erv_fault_state = 0;
int comm_check_cnt = 0;


static int get_switch_state(void)
{
    const char* switch_value = cap_switch_data->get_switch_value(cap_switch_data);
    int switch_state = SWITCH_OFF;

    if (!switch_value) {
        return -1;
    }

    if (!strcmp(switch_value, caps_helper_switch.attr_switch.value_on)) {
        switch_state = SWITCH_ON;
    } else if (!strcmp(switch_value, caps_helper_switch.attr_switch.value_off)) {
        switch_state = SWITCH_OFF;
    }
    return switch_state;
}

//-----
static int get_mode_state(void)
{
    const char* mode_value = cap_mode_data->get_mode_value(cap_mode_data);
    int mode_state = MODE_AUTO;

    if (!mode_value) {
        return -1;
    }

    if (!strncmp(mode_value, supported_mode[MODE_VENT], strlen(supported_mode[MODE_VENT]))) {
        mode_state = MODE_VENT;
    } else if (!strncmp(mode_value, supported_mode[MODE_BYPASS], strlen(supported_mode[MODE_BYPASS]))) {
        mode_state = MODE_BYPASS;
    } else if (!strncmp(mode_value, supported_mode[MODE_AUTO], strlen(supported_mode[MODE_AUTO]))) {
        mode_state = MODE_AUTO;
    } else if (!strncmp(mode_value, supported_mode[MODE_CLEAN_AIR], strlen(supported_mode[MODE_CLEAN_AIR]))) {
        mode_state = MODE_CLEAN_AIR;
    } else if (!strncmp(mode_value, supported_mode[MODE_FILTER_AUTO], strlen(supported_mode[MODE_FILTER_AUTO]))) {
        mode_state = MODE_FILTER_AUTO;
    } 
    //else if (!strncmp(mode_value, supported_mode[MODE_DEHUMIDIFY_COOLING], strlen(supported_mode[MODE_DEHUMIDIFY_COOLING]))) {
     //   mode_state = MODE_DEHUMIDIFY_COOLING;
    //} else if (!strncmp(mode_value, supported_mode[MODE_AUTO], strlen(supported_mode[MODE_AUTO]))) {
     //   mode_state = MODE_AUTO;
    //}

    return mode_state;
}

static int get_filterStatus_state(void)
{
    const char* value = cap_filterStatus_data->get_filterStatus_value(cap_filterStatus_data);
    //ESP_LOGI(TAG, "filter status_1 %s", value);
    int filter_value = cap_filterStatus_data->attr_filterStatus_str2idx(value);
    //ESP_LOGI(TAG, "filter status_2  %d", filter_value);    
    if ( filter_value == -1) {
        return -1;
    }
    //ESP_LOGI(TAG, "filter status_3  %d", filter_value);
    return filter_value;
}

static int get_panicStatus_state(void)
{
    const char* value = cap_panicAlarm_data->get_panicAlarm_value(cap_panicAlarm_data);
    //ESP_LOGI(TAG, "panic_1 %s", value);
    int panic_value = cap_panicAlarm_data->attr_panicAlarm_str2idx(value);
    //ESP_LOGI(TAG, "panic_2  %d", panic_value);    
    if ( panic_value == -1) {
        return -1;
    }
    //ESP_LOGI(TAG, "panic_3  %d", panic_value);
    return panic_value;
}

static void cap_set_mode_cmd_cb(struct caps_mode_data *caps_data)
{
    int8_t  msg[2] = {3, -1} ;

    int mode_state = get_mode_state();
    switch (mode_state)
    {
    case MODE_VENT:
        msg[1]= 1;
        printf("Do something for [Ventilation]\n");
        break;
    case MODE_BYPASS:
        msg[1] = 2;
        printf("Do something for [BYPASS]\n");
        break;        
    case MODE_AUTO:
        msg[1] = 3;
        printf("Do something for [AUTO]\n");
        break;
    case MODE_CLEAN_AIR:
        msg[1] = 4;
        printf("Do something for [CLEAN_AIR]\n");
        break;
    case MODE_FILTER_AUTO:
        msg[1] = 5;
        printf("Do something for [FILTER_AUTO]\n");
        break;
    //case MODE_DEHUMIDIFY_COOLING:
     //   printf("Do something for [Dehumidification/Drying(Cooling)]\n");
     //   break;
    //case MODE_AUTO:
     //   printf("Do something for [Ventilation Auto Mode]\n");
     //   break;
    }
    msg[0] = 3;    
    //ESP_LOGI(TAG, "cap_set_mode_cmd_cb_QUEUE %d", msg[1]);

 //   int8_t i;
  //  i = uxQueueMessagesWaiting(main_queue);
//    ESP_LOGI(TAG, "uxQueueMessagesWaiting %d", i);
    if((main_queue != NULL) && msg[1] != -1)  {
        xQueueSend(main_queue, (void *)msg, (TickType_t)0);
        //ESP_LOGI(TAG, "cap_set_mode_cmd_cb_xQueueSend: %d", msg[1]);
    }    
}
//-------
// 미세먼지, 초미세먼지, CO2, TVOC 값을 보고한다
static void cap_b6SensorData_report(uint8_t *rxBuffer)
{
    int i = 0;
    double j;
    IOT_EVENT *cap_evt[7];
    uint8_t evt_num = 7;
    int32_t sequence_no;
    iot_cap_val_t value[7];

    if(!cap_dustSensor_data || !cap_dustSensor_data-> handle){
        ESP_LOGI(TAG, "Failed to get cap_dustSensor_data handle");
        return;
    }
    if(!cap_carbonDioxideMeasurement_data || !cap_carbonDioxideMeasurement_data-> handle){
        ESP_LOGI(TAG, "Failed to get cap_carbonDioxideMeasurement_data handle");
        return;
    }
    if(!cap_tvocMeasurement_data || !cap_tvocMeasurement_data-> handle){
        ESP_LOGI(TAG, "Failed to get cap_tvocMeasurement_data handle");
        return;
    }

    cap_dustSensor_data->dustLevel_value = *(rxBuffer+3);
    cap_dustSensor_data->fineDustLevel_value = *(rxBuffer+6);

    i = *(rxBuffer+4);
    if( i != 0) i = i  * 100;
    i = i + *(rxBuffer+5);
    cap_carbonDioxideMeasurement_data->carbonDioxide_value  = i;

    i = 0;
    i = *(rxBuffer+7);
    i = i << 8;
    i = i+ *(rxBuffer+8);
    j = (double)i / 1000;
    cap_tvocMeasurement_data->tvocLevel_value = j;

    value[0].type = IOT_CAP_VAL_TYPE_NUMBER;
    value[0].number = cap_dustSensor_data->dustLevel_value;

    cap_evt[0] = st_cap_create_attr(cap_dustSensor_data->handle,
            (char *) caps_helper_dustSensor.attr_dustLevel.name,
            &value[0],
            NULL,
            NULL);

    value[1].type = IOT_CAP_VAL_TYPE_NUMBER;
    value[1].number = cap_dustSensor_data->fineDustLevel_value;

    cap_evt[1] = st_cap_create_attr(cap_dustSensor_data->handle,
            (char *) caps_helper_dustSensor.attr_fineDustLevel.name,
            &value[1],
            NULL,
            NULL);

    value[2].type = IOT_CAP_VAL_TYPE_NUMBER;
    value[2].number = cap_carbonDioxideMeasurement_data->carbonDioxide_value;

    cap_evt[2] = st_cap_create_attr(cap_carbonDioxideMeasurement_data->handle,
            (char *) caps_helper_carbonDioxideMeasurement.attr_carbonDioxide.name,
            &value[2],
            NULL,
            NULL);

    value[3].type = IOT_CAP_VAL_TYPE_NUMBER;
    value[3].number = cap_tvocMeasurement_data->tvocLevel_value;

    cap_evt[3] = st_cap_create_attr(cap_tvocMeasurement_data->handle,
            (char *) caps_helper_tvocMeasurement.attr_tvocLevel.name,
            &value[3],
            NULL,
            NULL);

//  dustHealthConcern 관련 내용
   if(cap_dustSensor_data->dustLevel_value <= 30) {
        cap_dustHealthConcern_data->set_dustHealthConcern_value(cap_dustHealthConcern_data, (char *) caps_helper_dustHealthConcern.attr_dustHealthConcern.value_good);
    }
    else if(cap_dustSensor_data->dustLevel_value  <= 80){
        cap_dustHealthConcern_data->set_dustHealthConcern_value(cap_dustHealthConcern_data, (char *) caps_helper_dustHealthConcern.attr_dustHealthConcern.value_moderate);       
    }
    //else if(cap_dustSensor_data->dustLevel_value   <= 150){
    //    cap_dustHealthConcern_data->set_dustHealthConcern_value(cap_dustHealthConcern_data, (char *) caps_helper_dustHealthConcern.attr_dustHealthConcern.value_slightlyUnhealthy);       
    //}
    else if(cap_dustSensor_data->dustLevel_value  <= 150){
       cap_dustHealthConcern_data->set_dustHealthConcern_value(cap_dustHealthConcern_data, (char *) caps_helper_dustHealthConcern.attr_dustHealthConcern.value_unhealthy);       
    }
    else if(cap_dustSensor_data->dustLevel_value   <= 300){
        cap_dustHealthConcern_data->set_dustHealthConcern_value(cap_dustHealthConcern_data, (char *) caps_helper_dustHealthConcern.attr_dustHealthConcern.value_veryUnhealthy);       
    } 
    else   cap_dustHealthConcern_data->set_dustHealthConcern_value(cap_dustHealthConcern_data, (char *) caps_helper_dustHealthConcern.attr_dustHealthConcern.value_hazardous);               

//  fineDustHealthConcern관련 내용
    if(cap_dustSensor_data->fineDustLevel_value <= 15) {
        cap_fineDustHealthConcern_data->set_fineDustHealthConcern_value(cap_fineDustHealthConcern_data, (char *) caps_helper_fineDustHealthConcern.attr_fineDustHealthConcern.value_good);
    }
    else if(cap_dustSensor_data->fineDustLevel_value  <= 35){
        cap_fineDustHealthConcern_data->set_fineDustHealthConcern_value(cap_fineDustHealthConcern_data, (char *) caps_helper_fineDustHealthConcern.attr_fineDustHealthConcern.value_moderate);       
    }
    else if(cap_dustSensor_data->fineDustLevel_value  <= 75){
        cap_fineDustHealthConcern_data->set_fineDustHealthConcern_value(cap_fineDustHealthConcern_data, (char *) caps_helper_fineDustHealthConcern.attr_fineDustHealthConcern.value_slightlyUnhealthy);       
    }
    else if(cap_dustSensor_data->fineDustLevel_value  <= 90){
        cap_fineDustHealthConcern_data->set_fineDustHealthConcern_value(cap_fineDustHealthConcern_data, (char *) caps_helper_fineDustHealthConcern.attr_fineDustHealthConcern.value_unhealthy);       
    }
    else if(cap_dustSensor_data->fineDustLevel_value  <= 180){
        cap_fineDustHealthConcern_data->set_fineDustHealthConcern_value(cap_fineDustHealthConcern_data, (char *) caps_helper_fineDustHealthConcern.attr_fineDustHealthConcern.value_veryUnhealthy);       
    } 
    else   cap_fineDustHealthConcern_data->set_fineDustHealthConcern_value(cap_fineDustHealthConcern_data, (char *) caps_helper_fineDustHealthConcern.attr_fineDustHealthConcern.value_hazardous);               

//  carbonDioxideHealthConcern관련 내용
   if(cap_carbonDioxideMeasurement_data->carbonDioxide_value <= 450) {
        cap_carbonDioxideHealthConcern_data->set_carbonDioxideHealthConcern_value(cap_carbonDioxideHealthConcern_data, (char *) caps_helper_carbonDioxideHealthConcern.attr_carbonDioxideHealthConcern.value_good);
    }
    else if(cap_carbonDioxideMeasurement_data->carbonDioxide_value  <= 1000){
        cap_carbonDioxideHealthConcern_data->set_carbonDioxideHealthConcern_value(cap_carbonDioxideHealthConcern_data, (char *) caps_helper_carbonDioxideHealthConcern.attr_carbonDioxideHealthConcern.value_moderate);       
    }
    else if(cap_carbonDioxideMeasurement_data->carbonDioxide_value <= 2000){
       cap_carbonDioxideHealthConcern_data->set_carbonDioxideHealthConcern_value(cap_carbonDioxideHealthConcern_data, (char *) caps_helper_carbonDioxideHealthConcern.attr_carbonDioxideHealthConcern.value_slightlyUnhealthy);       
    }
    else if(cap_carbonDioxideMeasurement_data->carbonDioxide_value  <= 3000){
        cap_carbonDioxideHealthConcern_data->set_carbonDioxideHealthConcern_value(cap_carbonDioxideHealthConcern_data, (char *) caps_helper_carbonDioxideHealthConcern.attr_carbonDioxideHealthConcern.value_unhealthy);       
    }
    else if(cap_carbonDioxideMeasurement_data->carbonDioxide_value <= 5000){
        cap_carbonDioxideHealthConcern_data->set_carbonDioxideHealthConcern_value(cap_carbonDioxideHealthConcern_data, (char *) caps_helper_carbonDioxideHealthConcern.attr_carbonDioxideHealthConcern.value_veryUnhealthy);       
    } 
    else   cap_carbonDioxideHealthConcern_data->set_carbonDioxideHealthConcern_value(cap_carbonDioxideHealthConcern_data, (char *) caps_helper_carbonDioxideHealthConcern.attr_carbonDioxideHealthConcern.value_hazardous);               

    value[4].type = IOT_CAP_VAL_TYPE_STRING;
    value[4].string = cap_dustHealthConcern_data->dustHealthConcern_value;
    cap_evt[4] = st_cap_create_attr(cap_dustHealthConcern_data->handle,
            (char *) caps_helper_dustHealthConcern.attr_dustHealthConcern.name,
            &value[4],
            NULL,
            NULL);

    value[5].type = IOT_CAP_VAL_TYPE_STRING;
    value[5].string = cap_fineDustHealthConcern_data->fineDustHealthConcern_value;

    cap_evt[5] = st_cap_create_attr(cap_fineDustHealthConcern_data->handle,
            (char *) caps_helper_fineDustHealthConcern.attr_fineDustHealthConcern.name,
            &value[5],
            NULL,
            NULL);

    value[6].type = IOT_CAP_VAL_TYPE_STRING;
    value[6].string = cap_carbonDioxideHealthConcern_data->carbonDioxideHealthConcern_value;

    cap_evt[6] = st_cap_create_attr(cap_carbonDioxideHealthConcern_data->handle,
            (char *) caps_helper_carbonDioxideHealthConcern.attr_carbonDioxideHealthConcern.name,
            &value[6],
            NULL,
            NULL);

    if (!cap_evt[0] || !cap_evt[1] || !cap_evt[2] || !cap_evt[3]  || !cap_evt[4] || !cap_evt[5] || !cap_evt[6]) {
        printf("fail to create cap_evt\n");
        free(cap_evt[0]);
        free(cap_evt[1]);
        free(cap_evt[2]);
        free(cap_evt[3]);        
        free(cap_evt[4]);
        free(cap_evt[5]);           
        free(cap_evt[6]);                  
        return;
    }

    sequence_no = st_cap_send_attr(cap_evt, evt_num);
    if (sequence_no < 0)
        printf("fail to send color data\n");

    printf("Sequence number return : %d\n", sequence_no);
    st_cap_free_attr(cap_evt[0]);
    st_cap_free_attr(cap_evt[1]);        
    st_cap_free_attr(cap_evt[2]);
    st_cap_free_attr(cap_evt[3]);
    st_cap_free_attr(cap_evt[4]);
    st_cap_free_attr(cap_evt[5]); 
    st_cap_free_attr(cap_evt[6]);     
}

// 습도와 온도값을 보고한다
static void cap_b7SensorData_report(uint8_t *rxBuffer)
{
    IOT_EVENT *cap_evt[2];
    uint8_t evt_num = 2;
    int32_t sequence_no;
    iot_cap_val_t value[2];

    double temp;

    if(!cap_relativeHumidityMeasurement_data || !cap_relativeHumidityMeasurement_data-> handle){
        ESP_LOGI(TAG, "Failed to get cap_relativeHumidityMeasurement_data handle");
        return;
    }
    if(!cap_temperatureMeasurement_data || !cap_temperatureMeasurement_data-> handle){
        ESP_LOGI(TAG, "Failed to get cap_temperatureMeasurement_data handle");
        return;
    }

    temp = *(rxBuffer+4);
    temp = temp- 20;
    cap_temperatureMeasurement_data->temperature_value = temp;
    
    cap_relativeHumidityMeasurement_data->humidity_value = *(rxBuffer+5);

   
    value[0].type = IOT_CAP_VAL_TYPE_NUMBER;
    value[0].number = cap_relativeHumidityMeasurement_data->humidity_value;

    cap_evt[0] = st_cap_create_attr(cap_relativeHumidityMeasurement_data->handle,
            (char *) caps_helper_relativeHumidityMeasurement.attr_humidity.name,
            &value[0],
            NULL,
            NULL);

    value[1].type = IOT_CAP_VAL_TYPE_NUMBER;
    value[1].number = cap_temperatureMeasurement_data->temperature_value;

    cap_evt[1] = st_cap_create_attr(cap_temperatureMeasurement_data->handle,
            (char *) caps_helper_temperatureMeasurement.attr_temperature.name,
            &value[1],
            "C",
            NULL);

    if (!cap_evt[0] || !cap_evt[1] ) {
        printf("fail to create cap_evt\n");
        free(cap_evt[0]);
        free(cap_evt[1]);
        return;
    }

    sequence_no = st_cap_send_attr(cap_evt, evt_num);
    if (sequence_no < 0)
        printf("fail to send temp or rPH data\n");

    printf("Sequence number return : %d\n", sequence_no);
    st_cap_free_attr(cap_evt[0]);
    st_cap_free_attr(cap_evt[1]);        

}

static void change_erv_mode(uint8_t new_mode){
    ESP_LOGI(TAG, "change_erv_mode : %d", new_mode);
    switch(new_mode){
        case MODE_VENT:
            cap_mode_data->set_mode_value(cap_mode_data, supported_mode[MODE_VENT]);
            break;
        case MODE_BYPASS:
            cap_mode_data->set_mode_value(cap_mode_data, supported_mode[MODE_BYPASS]);
            break;   
        case MODE_AUTO:
            cap_mode_data->set_mode_value(cap_mode_data, supported_mode[MODE_AUTO]);
            break;               
        case MODE_CLEAN_AIR:
            cap_mode_data->set_mode_value(cap_mode_data, supported_mode[MODE_CLEAN_AIR]);
            break;               
        case MODE_FILTER_AUTO:
            cap_mode_data->set_mode_value(cap_mode_data, supported_mode[MODE_FILTER_AUTO]);
            break;           
        default:
            ESP_LOGI(TAG, "change_erv_mode error");
            break;                                                           
    }
    cap_mode_data->attr_mode_send(cap_mode_data);           
}

// 습도와 온도값을 보고한다
static void cap_b5SensorData_report(uint8_t *rxBuffer)
{
    uint8_t i, j;

    // 전원 On/Off를 검사한다
    i = *(rxBuffer+3);
    i = i & 0x30;
    i = i >> 4;
    j = get_switch_state();
    //ESP_LOGI(TAG, "cap_b5SensorData_report: %x %x", i,  j);
    if( i != j ) {
        if( i == 0)  cap_switch_data->set_switch_value(cap_switch_data, (char *) caps_helper_switch.attr_switch.value_off);
        else  cap_switch_data->set_switch_value(cap_switch_data, (char *) caps_helper_switch.attr_switch.value_on);
        cap_switch_data->attr_switch_send(cap_switch_data);
    }
    // 팬 속도를 검사한다
    i = *(rxBuffer+3);
    i = i & 0x7;
    j = cap_fanSpeed_data->get_fanSpeed_value(cap_fanSpeed_data);
    //ESP_LOGI(TAG, "fanSpeed: %x %x", i,  j);
    if( i != j ) {
        cap_fanSpeed_data->set_fanSpeed_value(cap_fanSpeed_data, i);
        cap_fanSpeed_data->attr_fanSpeed_send(cap_fanSpeed_data);
    }
    // 모드를 검사한다
    i = get_mode_state();
    j = *(rxBuffer+4);    
    if( i == MODE_VENT &&  j !=  MODE_VENT){
        change_erv_mode(j);
    }
    else if( i == MODE_BYPASS  &&  j !=  MODE_BYPASS ){
        change_erv_mode(j);
    }    
    else if( i == MODE_AUTO &&  j !=  MODE_AUTO){
        change_erv_mode(j);         
    }   
    else if( i == MODE_CLEAN_AIR  &&  j !=  MODE_CLEAN_AIR){
        change_erv_mode(j);      
    }            
    else if( i == MODE_FILTER_AUTO   &&  j !=  MODE_FILTER_AUTO){
        change_erv_mode(j);              
    } 
    // 필터 경보를 알려준다
    i = *(rxBuffer+5);
    j = get_filterStatus_state();
    //ESP_LOGI(TAG, "filter status_3 %d %d", i, j );
    if( j  == FILTER_NORMAL){                   //  Filter_Normal
        if( i == 0){                // Filter 시간 초과
            cap_filterStatus_data->set_filterStatus_value(cap_filterStatus_data, caps_helper_filterStatus.attr_filterStatus.value_replace);
            cap_filterStatus_data->attr_filterStatus_send(cap_filterStatus_data);            
        }
    }
    else{                           //  Filter_Replace
       if( i  > 0){                // Filter 시간 남아있음
            cap_filterStatus_data->set_filterStatus_value(cap_filterStatus_data, caps_helper_filterStatus.attr_filterStatus.value_normal);
            cap_filterStatus_data->attr_filterStatus_send(cap_filterStatus_data);            
        }
    } 
    // 고장 유무를 알려준다
    i = *(rxBuffer+8);  
    erv_fault_state = i;
    j = get_panicStatus_state();     
    //ESP_LOGI(TAG, "panic status %d %d", i, j );    
    if( j == ERV_CLEAR){
             if( i & 0x13){             // filter error(BIT7) + 통신 에러(BIT4) + 급기팬 에러(BIT1) + 배기팬 에러 (BIT0)
                cap_panicAlarm_data->set_panicAlarm_value(cap_panicAlarm_data, caps_helper_panicAlarm.attr_panicAlarm.value_panic);
                cap_panicAlarm_data->attr_panicAlarm_send(cap_panicAlarm_data);        
             }
    }
    else{
        if( i == 0){
            cap_panicAlarm_data->set_panicAlarm_value(cap_panicAlarm_data, caps_helper_panicAlarm.attr_panicAlarm.value_clear);
            cap_panicAlarm_data->attr_panicAlarm_send(cap_panicAlarm_data);                   
        }
    }
}

static void erv_data_parsing(uint8_t *rxBuffer)
{
    #if 0           // 시험용
	comm_check_cnt = 0;
    if(*rxBuffer == 0xB5) cap_b5SensorData_report(rxBuffer);
    #if 1  //시험용 화면 출력이 많아서
    if(*rxBuffer == 0xB6) cap_b6SensorData_report(rxBuffer);
    #endif
    if(*rxBuffer == 0xB7) cap_b7SensorData_report(rxBuffer);
    #endif
}

static void cap_switch_cmd_cb(struct caps_switch_data *caps_data)
{
    //int8_t msg[2];
    //int switch_state = get_switch_state();
    //change_switch_state(switch_state);
    //msg[0] = 1;
    //msg[1] = switch_state;
    //if(switch_state) msg[1] = 0;
    //else msg[1] = 1;
     //if(main_queue != NULL) { 
    //    xQueueSend(main_queue, (void *)msg, (TickType_t)1);   
    //    ESP_LOGI(TAG, "cap_switch_cmd_cb_xQueueSend");
    // }
}

static void cap_switchLevel_cmd_cb(struct caps_switchLevel_data *caps_data)
{
    int switch_level = caps_data->get_level_value(caps_data);
    change_switch_level(switch_level);
}

static void cap_colorTemp_cmd_cb(struct caps_colorTemperature_data *caps_data)
{
    update_color_info(cap_colorTemp_data->get_colorTemperature_value(cap_colorTemp_data));
    change_switch_state(get_switch_state());
}

#if 0
static void cap_lightMode_cmd_cb(struct caps_activityLightingMode_data *caps_data)
{
    const char* lightMode = cap_lightMode_data->get_lightingMode_value(cap_lightMode_data);

    int colorTemp = 0;
    if (!strcmp(lightMode, caps_helper_activityLightingMode.attr_lightingMode.value_reading)) {
        colorTemp = 4000;
    } else if (!strcmp(lightMode, caps_helper_activityLightingMode.attr_lightingMode.value_writing)) {
        colorTemp = 5000;
    } else if (!strcmp(lightMode, caps_helper_activityLightingMode.attr_lightingMode.value_computer)) {
        colorTemp = 6000;
    } else if (!strcmp(lightMode, caps_helper_activityLightingMode.attr_lightingMode.value_day)) {
        colorTemp = 5500;
    } else if (!strcmp(lightMode, caps_helper_activityLightingMode.attr_lightingMode.value_night)) {
        colorTemp = 6500;
    } else if (!strcmp(lightMode, caps_helper_activityLightingMode.attr_lightingMode.value_sleepPreparation)) {
        colorTemp = 3000;
    } else if (!strcmp(lightMode, caps_helper_activityLightingMode.attr_lightingMode.value_cozy)) {
        colorTemp = 2000;
    } else if (!strcmp(lightMode, caps_helper_activityLightingMode.attr_lightingMode.value_soft)) {
        colorTemp = 2500;
    }
    cap_colorTemp_data->set_colorTemperature_value(cap_colorTemp_data, colorTemp);
    update_color_info(cap_colorTemp_data->get_colorTemperature_value(cap_colorTemp_data));
    change_switch_state(get_switch_state());
    cap_colorTemp_data->attr_colorTemperature_send(cap_colorTemp_data);
}
#endif

// 센도리
static void cap_fanSpeed_cmd_cb(struct  caps_fanSpeed_data *caps_data)
{
    int8_t msg[2];
    int fan_speed = caps_data->get_fanSpeed_value(caps_data);
    msg[0] = 2;
    msg[1] = fan_speed;
    if(main_queue != NULL)  {
        xQueueSend(main_queue, (void *)msg, (TickType_t)0);
        ESP_LOGI(TAG, "cap_fanSpeed_cmd_cb_xQueueSend");
    }
}


static void cap_mode_cmd_cb(struct  caps_mode_data *caps_data)
{
    //int8_t msg[2];
    const char *initial_mode = caps_data->get_mode_value(caps_data);
}

#ifdef OTA
static char *get_current_firmware_version(void)
{
    char *current_version = NULL;

    unsigned char *device_info = (unsigned char *) device_info_start;
    unsigned int device_info_len = device_info_end - device_info_start;

    ota_err_t err = ota_api_get_firmware_version_load(device_info, device_info_len, &current_version);
    if (err != OTA_OK) {
        printf("ota_api_get_firmware_version_load is failed : %d\n", err);
    }

    return current_version;
}

#define OTA_UPDATE_MAX_RETRY_COUNT 100

static void ota_update_task(void * pvParameter)
{
    printf("\n Starting OTA...\n");

    static int count = 0;

    while (1) {

        ota_err_t ret = ota_update_device();
        if (ret != OTA_OK) {
            printf("Firmware Upgrades Failed (%d) \n", ret);
            vTaskDelay(600 * 1000 / portTICK_PERIOD_MS);
            count++;
        } else {
            break;
        }

        if (count > OTA_UPDATE_MAX_RETRY_COUNT)
            break;
    }

    printf("Prepare to restart system!");
    ota_restart_device();
}

static void cap_update_cmd_cb(struct caps_firmwareUpdate_data *caps_data)
{
	ota_nvs_flash_init();

	xTaskCreate(&ota_update_task, "ota_update_task", 8096, NULL, 5, &ota_task_handle);
}

void ota_polling_task(void *arg)
{
    while (1) {

        vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);

        if (g_iot_status != IOT_STATUS_CONNECTING || g_iot_stat_lv != IOT_STAT_LV_DONE) {
            continue;
        }

        if (ota_task_handle != NULL) {
            printf("Device is updating.. \n");
            continue;
        }

        ota_check_for_update((void *)arg);

	/* Set polling period */
	unsigned int polling_day = ota_get_polling_period_day();
	#if 0       // 시험용 
        unsigned int task_delay_sec = polling_day * 24 * 3600;
    #else
        unsigned int task_delay_sec = polling_day * 1 * 600;
    #endif
	vTaskDelay(task_delay_sec * 1000 / portTICK_PERIOD_MS);
    }
}
#endif

static void capability_init()
{
    cap_switch_data = caps_switch_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_switch_data) {
        const char *switch_init_value = caps_helper_switch.attr_switch.value_on;
        cap_switch_data->cmd_on_usr_cb = cap_switch_cmd_cb;
        cap_switch_data->cmd_off_usr_cb = cap_switch_cmd_cb;
        cap_switch_data->set_switch_value(cap_switch_data, switch_init_value);
    }

    cap_dustSensor_data = caps_dustSensor_initialize(iot_ctx, "monitor", NULL, NULL);
    if (cap_dustSensor_data) {
        cap_dustSensor_data->set_dustLevel_value(cap_dustSensor_data, 0);
        cap_dustSensor_data->set_fineDustLevel_value(cap_dustSensor_data, 0);

        cap_dustSensor_data->set_dustLevel_unit(cap_dustSensor_data, caps_helper_dustSensor.attr_dustLevel.unit_ug_per_m3);
        cap_dustSensor_data->set_fineDustLevel_unit(cap_dustSensor_data, caps_helper_dustSensor.attr_fineDustLevel.unit_ug_per_m3);
    }

    // 센도리
    // 서버에 아직 정의되지 않음
    cap_dustHealthConcern_data = caps_dustHealthConcern_initialize(iot_ctx, "monitor", NULL, NULL);
    if (cap_dustHealthConcern_data) {
        cap_dustHealthConcern_data->set_dustHealthConcern_value(cap_dustHealthConcern_data, 
           (char *) caps_helper_dustHealthConcern.attr_dustHealthConcern.value_moderate);
    }
    cap_fineDustHealthConcern_data = caps_fineDustHealthConcern_initialize(iot_ctx, "monitor", NULL, NULL);
    if (cap_fineDustHealthConcern_data) {
        cap_fineDustHealthConcern_data->set_fineDustHealthConcern_value(cap_fineDustHealthConcern_data, 
            (char *) caps_helper_fineDustHealthConcern.attr_fineDustHealthConcern.value_moderate); 
    }

    cap_carbonDioxideMeasurement_data = caps_carbonDioxideMeasurement_initialize(iot_ctx, "monitor", NULL, NULL);
    if (cap_carbonDioxideMeasurement_data) {
        cap_carbonDioxideMeasurement_data->set_carbonDioxide_value(cap_carbonDioxideMeasurement_data, 0);
        cap_carbonDioxideMeasurement_data->set_carbonDioxide_unit(cap_carbonDioxideMeasurement_data, caps_helper_carbonDioxideMeasurement.attr_carbonDioxide.unit_ppm);
    }
   cap_carbonDioxideHealthConcern_data = caps_carbonDioxideHealthConcern_initialize(iot_ctx, "monitor", NULL, NULL);
    if (cap_carbonDioxideHealthConcern_data) {
        cap_carbonDioxideHealthConcern_data->set_carbonDioxideHealthConcern_value(cap_carbonDioxideHealthConcern_data, 
            (char *) caps_helper_carbonDioxideHealthConcern.attr_carbonDioxideHealthConcern.value_moderate); 
    }

    cap_tvocMeasurement_data = caps_tvocMeasurement_initialize(iot_ctx, "monitor", NULL, NULL);
    if (cap_tvocMeasurement_data) {
        cap_tvocMeasurement_data->set_tvocLevel_value(cap_tvocMeasurement_data, 0);
        cap_tvocMeasurement_data->set_tvocLevel_unit(cap_tvocMeasurement_data, caps_helper_tvocMeasurement.attr_tvocLevel.unit_ppm);
    }
    // 서버에 아직 정의되지 않음
#if 1
   cap_tvocHealthConcern_data = caps_tvocHealthConcern_initialize(iot_ctx, "monitor", NULL, NULL);
   if (cap_tvocHealthConcern_data) {
       cap_tvocHealthConcern_data->set_tvocHealthConcern_value(cap_tvocHealthConcern_data, 
            (char *) caps_helper_tvocHealthConcern.attr_tvocHealthConcern.value_moderate); 
   }
#endif
    cap_relativeHumidityMeasurement_data = caps_relativeHumidityMeasurement_initialize(iot_ctx, "monitor", NULL, NULL);
    if (cap_relativeHumidityMeasurement_data) {
        cap_relativeHumidityMeasurement_data->set_humidity_value(cap_relativeHumidityMeasurement_data, 0);
        cap_relativeHumidityMeasurement_data->set_humidity_unit(cap_relativeHumidityMeasurement_data, caps_helper_relativeHumidityMeasurement.attr_humidity.unit_percent);
    }
   cap_temperatureMeasurement_data = caps_temperatureMeasurement_initialize(iot_ctx, "monitor", NULL, NULL);
    if (cap_temperatureMeasurement_data) {
        cap_temperatureMeasurement_data->set_temperature_value(cap_temperatureMeasurement_data, 0);
        cap_temperatureMeasurement_data->set_temperature_unit(cap_temperatureMeasurement_data, caps_helper_temperatureMeasurement.attr_temperature.unit_C);
    }

    cap_filterStatus_data = caps_filterStatus_initialize(iot_ctx, "monitor", NULL, NULL);
    if (cap_filterStatus_data) {
        cap_filterStatus_data->set_filterStatus_value(cap_filterStatus_data, caps_helper_filterStatus.attr_filterStatus.values[CAP_ENUM_FILTERSTATUS_FILTERSTATUS_VALUE_NORMAL]);
    }

   cap_signalStrength_data = caps_signalStrength_initialize(iot_ctx, "monitor", NULL, NULL);
    if (cap_signalStrength_data) {
        cap_signalStrength_data->set_rssi_unit(cap_signalStrength_data, caps_helper_signalStrength.attr_rssi.unit_dBm);
    }
    cap_panicAlarm_data = caps_panicAlarm_initialize(iot_ctx, "monitor", NULL, NULL);
    if (cap_panicAlarm_data) {
        cap_panicAlarm_data->set_panicAlarm_value(cap_panicAlarm_data, caps_helper_panicAlarm.attr_panicAlarm.values[CAP_ENUM_PANICALARM_PANICALARM_VALUE_CLEAR]);
    }

    cap_fanSpeed_data = caps_fanSpeed_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_fanSpeed_data) {
        int fanSpeed_value = 1;
        cap_fanSpeed_data->cmd_setFanSpeed_usr_cb = cap_fanSpeed_cmd_cb;
        cap_fanSpeed_data->set_fanSpeed_value(cap_fanSpeed_data, fanSpeed_value);
    }

     cap_mode_data = caps_mode_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_mode_data) {
        const char *mode_init_value = supported_mode[MODE_AUTO];

        cap_mode_data->cmd_setMode_usr_cb = cap_set_mode_cmd_cb;

        cap_mode_data->set_supportedModes_value(cap_mode_data, supported_mode, ARRAY_SIZE(supported_mode));        
        cap_mode_data->set_mode_value(cap_mode_data, mode_init_value);
    }

    // firmwareUpdate는 전혀 준비가 되지 않음
    #ifdef OTA
    cap_ota_data = caps_firmwareUpdate_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_ota_data) {

        char *firmware_version = get_current_firmware_version();

        cap_ota_data->set_currentVersion_value(cap_ota_data, firmware_version);
        cap_ota_data->cmd_updateFirmware_usr_cb = cap_update_cmd_cb;

        free(firmware_version);
    }
    #endif
}

static void iot_status_cb(iot_status_t status,
                          iot_stat_lv_t stat_lv, void *usr_data)
{
    g_iot_status = status;
    g_iot_stat_lv = stat_lv;

    printf("status: %d, stat: %d\n", g_iot_status, g_iot_stat_lv);

    switch(status)
    {
        case IOT_STATUS_NEED_INTERACT:
            noti_led_mode = LED_ANIMATION_MODE_FAST;
            break;
        case IOT_STATUS_IDLE:
        case IOT_STATUS_CONNECTING:
            noti_led_mode = LED_ANIMATION_MODE_IDLE;
            change_switch_state(get_switch_state());
            break;
        default:
            break;
    }
}

#if defined(SET_PIN_NUMBER_CONFRIM)
void* pin_num_memcpy(void *dest, const void *src, unsigned int count)
{
    unsigned int i;
    for (i = 0; i < count; i++)
        *((char*)dest + i) = *((char*)src + i);
    return dest;
}
#endif

static void connection_start(void)
{
    iot_pin_t *pin_num = NULL;
    int err;

#if defined(SET_PIN_NUMBER_CONFRIM)
    pin_num = (iot_pin_t *) malloc(sizeof(iot_pin_t));
    if (!pin_num)
        printf("failed to malloc for iot_pin_t\n");

    // to decide the pin confirmation number(ex. "12345678"). It will use for easysetup.
    //    pin confirmation number must be 8 digit number.
    pin_num_memcpy(pin_num, "12345678", sizeof(iot_pin_t));
#endif

    // process on-boarding procedure. There is nothing more to do on the app side than call the API.
    err = st_conn_start(iot_ctx, (st_status_cb)&iot_status_cb, IOT_STATUS_ALL, NULL, pin_num);
    if (err) {
        printf("fail to start connection. err:%d\n", err);
    }
    if (pin_num) {
        free(pin_num);
    }
}

static void connection_start_task(void *arg)
{
    connection_start();
    vTaskDelete(NULL);
}

static void iot_noti_cb(iot_noti_data_t *noti_data, void *noti_usr_data)
{
    printf("Notification message received\n");

    if (noti_data->type == IOT_NOTI_TYPE_DEV_DELETED) {
        printf("[device deleted]\n");
    } else if (noti_data->type == IOT_NOTI_TYPE_RATE_LIMIT) {
        printf("[rate limit] Remaining time:%d, sequence number:%d\n",
               noti_data->raw.rate_limit.remainingTime, noti_data->raw.rate_limit.sequenceNumber);
    } else if(noti_data->type == IOT_NOTI_TYPE_PREFERENCE_UPDATED) {
		for (int i = 0; i < noti_data->raw.preferences.preferences_num; i++) {
			printf("[preference update] name : %s value : ", noti_data->raw.preferences.preferences_data[i].preference_name);
			if (noti_data->raw.preferences.preferences_data[i].preference_data.type == IOT_CAP_VAL_TYPE_NULL)
				printf("NULL\n");
			else if (noti_data->raw.preferences.preferences_data[i].preference_data.type == IOT_CAP_VAL_TYPE_STRING)
				printf("%s\n", noti_data->raw.preferences.preferences_data[i].preference_data.string);
			else if (noti_data->raw.preferences.preferences_data[i].preference_data.type == IOT_CAP_VAL_TYPE_NUMBER)
				printf("%f\n", noti_data->raw.preferences.preferences_data[i].preference_data.number);
			else if (noti_data->raw.preferences.preferences_data[i].preference_data.type == IOT_CAP_VAL_TYPE_INTEGER)
				printf("%d\n", noti_data->raw.preferences.preferences_data[i].preference_data.integer);
			else if (noti_data->raw.preferences.preferences_data[i].preference_data.type == IOT_CAP_VAL_TYPE_BOOLEAN)
				printf("%s\n", noti_data->raw.preferences.preferences_data[i].preference_data.boolean ? "true" : "false");
			else
				printf("Unknown type\n");
		}
	}
}

void button_event(IOT_CAP_HANDLE *handle, int type, int count)
{
    if (type == BUTTON_SHORT_PRESS) {
        printf("Button short press, count: %d\n", count);
        switch(count) {
            case 1:
                if (g_iot_status == IOT_STATUS_NEED_INTERACT) {
                    st_conn_ownership_confirm(iot_ctx, true);
                    noti_led_mode = LED_ANIMATION_MODE_IDLE;
                    change_switch_state(get_switch_state());
                } else {
                    if (get_switch_state() == SWITCH_ON) {
                        change_switch_state(SWITCH_OFF);
                        cap_switch_data->set_switch_value(cap_switch_data, caps_helper_switch.attr_switch.value_off);
                        cap_switch_data->attr_switch_send(cap_switch_data);
                    } else {
                        change_switch_state(SWITCH_ON);
                        cap_switch_data->set_switch_value(cap_switch_data, caps_helper_switch.attr_switch.value_on);
                        cap_switch_data->attr_switch_send(cap_switch_data);
                    }
                }
                break;
            case 2:
                monitor_enable = !monitor_enable;
                printf("change monitor mode to %d\n", monitor_enable);
                break;
            case 5:
                /* clean-up provisioning & registered data with reboot option*/
                st_conn_cleanup(iot_ctx, true);
                break;
            default:
                led_blink(get_switch_state(), 100, count);
                break;
        }
    } else if (type == BUTTON_LONG_PRESS) {
        printf("Button long press, iot_status: %d\n", g_iot_status);
        led_blink(get_switch_state(), 100, 3);
        st_conn_cleanup(iot_ctx, false);
        xTaskCreate(connection_start_task, "connection_task", 2048, NULL, 10, NULL);
    }
}

// ERV와 통신이 30초 이상 응답이 없으면 ERV가 꺼져있거나, 통신선 불량으로 에러 처리한다
// switch off 시키고, panic 보고를 한다
void comm_check_callback(void *param)
{
    const char* panic_value = cap_panicAlarm_data->get_panicAlarm_value(cap_panicAlarm_data);
    if (!panic_value) {
        return ;
     }

    if(++comm_check_cnt >= 3){
        comm_check_cnt = 3;
        if(!cap_switch_data || !cap_switch_data-> handle){
            ESP_LOGI(TAG, "Failed to get cap_switch_data handle");
            return;
        }
        if(!cap_panicAlarm_data || !cap_panicAlarm_data-> handle){
            ESP_LOGI(TAG, "Failed to get cap_panicAlarm_data handle");
            return;
        } 
         if (!strcmp(panic_value, caps_helper_panicAlarm.attr_panicAlarm.value_clear)) {
            printf("strcmp comm_check_cnt = %d\n", comm_check_cnt);
             cap_switch_data->set_switch_value(cap_switch_data, (char *) caps_helper_switch.attr_switch.value_off);
             cap_switch_data->attr_switch_send(cap_switch_data);
             cap_panicAlarm_data->set_panicAlarm_value(cap_panicAlarm_data, (char *) caps_helper_panicAlarm.attr_panicAlarm.value_panic);
             cap_panicAlarm_data->attr_panicAlarm_send(cap_panicAlarm_data);  
         }
    }
    else{
        if(erv_fault_state == 0){
            if (!strcmp(panic_value, caps_helper_panicAlarm.attr_panicAlarm.value_clear)) {
                cap_panicAlarm_data->set_panicAlarm_value(cap_panicAlarm_data, (char *) caps_helper_panicAlarm.attr_panicAlarm.value_clear);
                cap_panicAlarm_data->attr_panicAlarm_send(cap_panicAlarm_data);  
            }
        }
    }
}

static void app_main_task(void *arg)
{
    uint8_t rxBuffer[10];
    IOT_CAP_HANDLE *handle = (IOT_CAP_HANDLE *)arg;

    int button_event_type;
    int button_event_count;

    //int dustLevel_value = 0;
    //int fineDustLevel_value = 0;
    //TimeOut_t monitor_timeout;
    // The macro pdMS_TO_TICKS() can be used to convert a time specified in milliseconds into a time specified in ticks
    //TickType_t monitor_period_tick = pdMS_TO_TICKS(monitor_period_ms);  
    TimeOut_t rssi_timeout;
    TickType_t rssi_period_tick = 300;
    //vTaskSetTimeOutState(&monitor_timeout);

    // LQI가 나오지 않음으로 아래 문장은 필요없음(10.06)
    //vTaskSetTimeOutState(&rssi_timeout);

    main_queue = xQueueCreate(5, MAIN_MSG_LENGTH);
    if(!main_queue){
        ESP_LOGI(TAG, "Faiiled to create uart_queue = %p", main_queue);
    }

    const esp_timer_create_args_t comm_check_timer_args = {
        .callback = &comm_check_callback,
        .name = "CommunicationCheckTimer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&comm_check_timer_args, &comm_check_timer_handler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(comm_check_timer_handler, 10000000));         // 10초 타이머

    for (;;) {
        // 센도리
        if(xQueueReceive(uart_queue, &(rxBuffer), (TickType_t)0)){
                erv_data_parsing(rxBuffer);
        }
        //------------------

        //if (monitor_enable && (xTaskCheckForTimeOut(&monitor_timeout, &monitor_period_tick) != pdFALSE)) {
        //    vTaskSetTimeOutState(&monitor_timeout);
        //    monitor_period_tick = pdMS_TO_TICKS(monitor_period_ms);
            /* emulate sensor value for example */
            //dustLevel_value = (dustLevel_value + 1) % 300;
            //fineDustLevel_value = dustLevel_value;

            //cap_dustSensor_data->set_dustLevel_value(cap_dustSensor_data, dustLevel_value);
            //cap_dustSensor_data->attr_dustLevel_send(cap_dustSensor_data);

            //cap_dustSensor_data->set_fineDustLevel_value(cap_dustSensor_data, fineDustLevel_value);
            //cap_dustSensor_data->attr_fineDustLevel_send(cap_dustSensor_data);
        //}
        if(xTaskCheckForTimeOut(&rssi_timeout, &rssi_period_tick) ==pdTRUE){
            vTaskSetTimeOutState(&rssi_timeout);
            rssi_period_tick = 3000;             // 300 x 10msec = 3,0000msec 마다 수행됨
#if  0
            wifi_ap_record_t accessPoint;
            if(esp_wifi_sta_get_ap_info(&accessPoint) == ESP_OK){
                //ESP_LOGI(TAG, "Currently connected to %s %x", accessPoint.ssid, accessPoint.rssi);
                int i = cap_signalStrength_data->get_rssi_value(cap_signalStrength_data);
                if( i != accessPoint.rssi){
                    cap_signalStrength_data->set_rssi_value(cap_signalStrength_data, accessPoint.rssi);
                    cap_signalStrength_data->attr_rssi_send(cap_signalStrength_data);
                }
            }
#endif
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    /**
      SmartThings Device SDK(STDK) aims to make it easier to develop IoT devices by providing
      additional st_iot_core layer to the existing chip vendor SW Architecture.

      That is, you can simply develop a basic application
      by just calling the APIs provided by st_iot_core layer like below.

      // create a iot context
      1. st_conn_init();

      // create a handle to process capability
      2. st_cap_handle_init(); (called in function 'capability_init')

      // register a callback function to process capability command when it comes from the SmartThings Server.
      3. st_cap_cmd_set_cb(); (called in function 'capability_init')

      // process on-boarding procedure. There is nothing more to do on the app side than call the API.
      4. st_conn_start(); (called in function 'connection_start')
     */

    unsigned char *onboarding_config = (unsigned char *) onboarding_config_start;
    unsigned int onboarding_config_len = onboarding_config_end - onboarding_config_start;
    unsigned char *device_info = (unsigned char *) device_info_start;
    unsigned int device_info_len = device_info_end - device_info_start;

    int iot_err;

    // create a iot context
    iot_ctx = st_conn_init(onboarding_config, onboarding_config_len, device_info, device_info_len);
    if (iot_ctx != NULL) {
        iot_err = st_conn_set_noti_cb(iot_ctx, iot_noti_cb, NULL);
        if (iot_err)
            printf("fail to set notification callback function\n");
    } else {
        printf("fail to create the iot_context\n");
    }

    // create a handle to process capability and initialize capability info
    capability_init();

    //iot_gpio_init();
    register_iot_cli_cmd();
    uart_cli_main();
     erv_main();                // 센도리
    xTaskCreate(app_main_task, "app_main_task", 4096, NULL, 10, NULL);

#ifdef OTA
    xTaskCreate(ota_polling_task, "ota_polling_task", 8096, (void *)cap_ota_data, 5, NULL);
#endif

    // connect to server
    connection_start();
}
