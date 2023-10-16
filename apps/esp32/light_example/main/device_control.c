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


#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// 센도리 ----
#include "esp_log.h"
#include "device_control.h"
#include "dev_common.h"
static const char *TAG = "DEVICE";

#define OTA_DEMO    0

QueueHandle_t uart_queue;
esp_timer_handle_t uart_timer_handler;
esp_timer_handle_t control_cmd_timer_handler;

extern QueueHandle_t main_queue;

static uint8_t pwr_fansp = 1;              // 팬 스피드 기본값은 약풍
static uint8_t control_cmd_send_flag;
static uint8_t auto_flag, bypass_flag, clean_flag, dust_flag;
static uint8_t vent_flag = 1;       // 전원을 켜면 기본이 vent_mode임
// -------------------

static int rgb_color_red = 255;
static int rgb_color_green = 0;
static int rgb_color_blue = 0;

static void update_rgb_from_color_temp(int color_temp, int *red, int *green, int *blue)
{
    int ct_table[10][3] = {
            {160, 0, 0}, //0
            {220, 20, 0}, //1000
            {255, 50, 0}, //2000
            {255, 160, 0}, //3000
            {255, 230, 130}, //4000
            {255, 255, 255}, //5000
            {120, 150, 255}, //6000
            {60, 80, 240}, //7000
            {30, 70, 200}, //8000
            {10, 50, 130}, //9000
    };

    if (color_temp < 0) {
        *red = ct_table[0][0];
        *green = ct_table[0][1];
        *blue = ct_table[0][2];
        return;
    }
    if (color_temp >= 9000) {
        *red = ct_table[9][0];
        *green = ct_table[9][1];
        *blue = ct_table[9][2];
        return;
    }

    int idx = color_temp / 1000;
    int remain = color_temp % 1000;
    *red = ct_table[idx][0] + (ct_table[idx+1][0]-ct_table[idx][0])*remain/1000;
    *green = ct_table[idx][1] + (ct_table[idx+1][1]-ct_table[idx][1])*remain/1000;
    *blue = ct_table[idx][2] + (ct_table[idx+1][2]-ct_table[idx][2])*remain/1000;
}

void change_switch_state(int switch_state)
{
    if (switch_state == SWITCH_OFF) {
        ESP_LOGI(TAG, "SWITCH_OFF");
        // gpio_set_level(GPIO_OUTPUT_COLORLED_R, COLOR_LED_OFF);
        // gpio_set_level(GPIO_OUTPUT_COLORLED_G, COLOR_LED_OFF);
        // gpio_set_level(GPIO_OUTPUT_COLORLED_B, COLOR_LED_OFF);
        //uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) switch_off_msg,  MSG_LENGTH);
    } else {
        ESP_LOGI(TAG, "SWITCH_ON");
        // gpio_set_level(GPIO_OUTPUT_COLORLED_R, (rgb_color_red > 127) ? COLOR_LED_ON : COLOR_LED_OFF);
        // gpio_set_level(GPIO_OUTPUT_COLORLED_G, (rgb_color_green > 127) ? COLOR_LED_ON : COLOR_LED_OFF);
        // gpio_set_level(GPIO_OUTPUT_COLORLED_B, (rgb_color_blue > 127) ? COLOR_LED_ON : COLOR_LED_OFF);
        //uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) switch_on_msg,  MSG_LENGTH);
    }
}

void update_color_info(int color_temp)
{
    update_rgb_from_color_temp(color_temp,
                               &rgb_color_red, &rgb_color_green, &rgb_color_blue);
}

void change_switch_level(int level)
{
    printf("switch level is changed to %d", level);
    return;
}

void change_fanSpeed(int fan_speed)
{
    printf("fanSpeed is changed to %d", fan_speed);
    return;
}

int get_button_event(int* button_event_type, int* button_event_count)
{
    static uint32_t button_count = 0;
    static uint32_t button_last_state = BUTTON_GPIO_RELEASED;
    static TimeOut_t button_timeout;
    static TickType_t long_press_tick = pdMS_TO_TICKS(BUTTON_LONG_THRESHOLD_MS);
    static TickType_t button_delay_tick = pdMS_TO_TICKS(BUTTON_DELAY_MS);

    uint32_t gpio_level = 0;

    gpio_level = gpio_get_level(GPIO_INPUT_BUTTON);
    if (button_last_state != gpio_level) {
        /* wait debounce time to ignore small ripple of currunt */
        vTaskDelay( pdMS_TO_TICKS(BUTTON_DEBOUNCE_TIME_MS) );
        gpio_level = gpio_get_level(GPIO_INPUT_BUTTON);
        if (button_last_state != gpio_level) {
            printf("Button event, val: %d, tick: %u\n", gpio_level, (uint32_t)xTaskGetTickCount());
            button_last_state = gpio_level;
            if (gpio_level == BUTTON_GPIO_PRESSED) {
                button_count++;
            }
            vTaskSetTimeOutState(&button_timeout);
            button_delay_tick = pdMS_TO_TICKS(BUTTON_DELAY_MS);
            long_press_tick = pdMS_TO_TICKS(BUTTON_LONG_THRESHOLD_MS);
        }
    } else if (button_count > 0) {
        if ((gpio_level == BUTTON_GPIO_PRESSED)
                && (xTaskCheckForTimeOut(&button_timeout, &long_press_tick ) != pdFALSE)) {
            *button_event_type = BUTTON_LONG_PRESS;
            *button_event_count = 1;
            button_count = 0;
            return true;
        } else if ((gpio_level == BUTTON_GPIO_RELEASED)
                && (xTaskCheckForTimeOut(&button_timeout, &button_delay_tick ) != pdFALSE)) {
            *button_event_type = BUTTON_SHORT_PRESS;
            *button_event_count = button_count;
            button_count = 0;
            return true;
        }
    }
    return false;
}

void led_blink(int switch_state, int delay, int count)
{
    for (int i = 0; i < count; i++) {
        vTaskDelay(delay / portTICK_PERIOD_MS);
        change_switch_state(1 - switch_state);
        vTaskDelay(delay / portTICK_PERIOD_MS);
        change_switch_state(switch_state);
    }
}

void change_led_mode(int noti_led_mode)
{
    static TimeOut_t led_timeout;
    static TickType_t led_tick = -1;
    static int last_led_mode = -1;
    static int led_state = SWITCH_OFF;

    if (last_led_mode != noti_led_mode) {
        last_led_mode = noti_led_mode;
        vTaskSetTimeOutState(&led_timeout);
        led_tick = 0;
    }

    switch (noti_led_mode)
    {
        case LED_ANIMATION_MODE_IDLE:
            break;
        case LED_ANIMATION_MODE_SLOW:
            if (xTaskCheckForTimeOut(&led_timeout, &led_tick ) != pdFALSE) {
                led_state = 1 - led_state;
                change_switch_state(led_state);
                vTaskSetTimeOutState(&led_timeout);
                if (led_state == SWITCH_ON) {
                    led_tick = pdMS_TO_TICKS(200);
                } else {
                    led_tick = pdMS_TO_TICKS(800);
                }
            }
            break;
        case LED_ANIMATION_MODE_FAST:
            if (xTaskCheckForTimeOut(&led_timeout, &led_tick ) != pdFALSE) {
                led_state = 1 - led_state;
                change_switch_state(led_state);
                vTaskSetTimeOutState(&led_timeout);
                led_tick = pdMS_TO_TICKS(100);
            }
            break;
        default:
            break;
    }
}

void iot_gpio_init(void)
{
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;

    io_conf.pin_bit_mask = 1 << GPIO_OUTPUT_COLORLED_R;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = 1 << GPIO_OUTPUT_COLORLED_G;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = 1 << GPIO_OUTPUT_COLORLED_B;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = 1 << GPIO_OUTPUT_COLORLED_0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1 << GPIO_INPUT_BUTTON;
    io_conf.pull_down_en = (BUTTON_GPIO_RELEASED == 0);
    io_conf.pull_up_en = (BUTTON_GPIO_RELEASED == 1);
    gpio_config(&io_conf);

    gpio_set_intr_type(GPIO_INPUT_BUTTON, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);

    gpio_set_level(GPIO_OUTPUT_COLORLED_R, 1);
    gpio_set_level(GPIO_OUTPUT_COLORLED_G, 0);
    gpio_set_level(GPIO_OUTPUT_COLORLED_B, 0);
    gpio_set_level(GPIO_OUTPUT_COLORLED_0, 0);
}

// 센도리
void control_cmd_callback(void *param){
    control_cmd_send_flag = 0;          // 명령을 주었는데 통신에러가 발생하여 응답이 없는 경우 3초 후에 플래그를 제거하여 상태 요청을 보낸다 
    //ESP_LOGI(TAG, "control_cmd_send_flag release");
}

void uart_timer_callback(void *param)
{
    static uint8_t state_seq = 0;
    const uint8_t d5_state_msg[10] = {0xd5,  0, 0, 0, 0, 0, 0, 0, 0, 0xd5};
    const uint8_t d6_state_msg[10] = {0xd6,  0, 0, 0, 0, 0, 0, 0, 0, 0xd6};
    const uint8_t d7_state_msg[10] = {0xd7,  0, 0, 0, 0, 0, 0, 0, 0, 0xd7};
    if(!control_cmd_send_flag){
        gpio_set_level(RX_ENABLE, 1);
        if(state_seq == 0) uart_write_bytes(ERV_UART_PORT_NUM, (const char *) d5_state_msg,  MSG_LENGTH);
         if(state_seq == 1) uart_write_bytes(ERV_UART_PORT_NUM, (const char *) d6_state_msg,  MSG_LENGTH);
        if(state_seq == 2) uart_write_bytes(ERV_UART_PORT_NUM, (const char *) d7_state_msg,  MSG_LENGTH);
        //ESP_LOGI(TAG, "state_seq: %d", state_seq);
        if(++state_seq > 2)   state_seq = 0; 
        uart_wait_tx_done(ERV_UART_PORT_NUM, (20/ portTICK_PERIOD_MS));
        gpio_set_level(RX_ENABLE, 0);
    }
    else{
        //ESP_LOGI(TAG, "control_cmd_send_flag");
        esp_timer_start_once(control_cmd_timer_handler, 3000000);
    }
}

uint8_t make_crc(uint8_t *msg){
    uint8_t crc, i;
#if 1
    for(crc = i = 0; i < (MSG_LENGTH - 1); i++) {
        crc ^= *msg++;
    }
#else
    for(crc = i = 0; i < (MSG_LENGTH - 1); i++) {
        crc += *msg++;
        crc = 256 - crc;
    }
#endif
    return crc;
}

void send_cmd_to_erv(uint8_t *data)
{
    static uint8_t control_msg[10] = {0xe5,  0, 0, 0, 0, 0, 0, 0, 0, 0}; 
    static uint8_t operation_mode = 0;

    switch(*data++){
        case 1:                 // 전원 On/Off
            if(*data == 0) pwr_fansp &= 0x8f;
            else pwr_fansp |= BIT4;
            ESP_LOGI(TAG, "power_flag : %x", pwr_fansp);  
            break;
        case 2:                 // 팬 속도
            pwr_fansp = pwr_fansp & 0xf0;
            if(*data == 1) pwr_fansp |= 1;
            else if(*data == 2) pwr_fansp |= 2;
             else if(*data == 3) pwr_fansp |= 3;
             else pwr_fansp |= 1;
             break;
        case 3:                 // 동작 모드
            if(*data == 1) {
                vent_flag = 1;
                bypass_flag = auto_flag =  dust_flag = clean_flag = 0;
                operation_mode = 0x00;         //전열
            }
             else if(*data == 2) {   
                bypass_flag = 1;
                vent_flag = auto_flag =  dust_flag = clean_flag = 0;
                operation_mode = 0x01;          //바이패스
                ESP_LOGI(TAG, "bypass_flag : %x", operation_mode);             
             }               
             else if(*data == 3) {           
                ESP_LOGI(TAG, "auto_flag : %x", operation_mode);                     
                auto_flag = 1;
                vent_flag = bypass_flag =  dust_flag = clean_flag = 0; 
                operation_mode = 0x02;         //자동
                ESP_LOGI(TAG, "auto_flag : %x", operation_mode);  
             }
            else if(*data == 4) {
                clean_flag = 1;       
                vent_flag = bypass_flag =  dust_flag = auto_flag = 0;           
                operation_mode = 0x03;          // 청정
                ESP_LOGI(TAG, "clean_flag : %x", operation_mode);                
            }
             else if(*data == 5) {
                dust_flag = 1;    
                vent_flag = bypass_flag =  clean_flag = auto_flag = 0;        
                operation_mode = 0x04;          // 필터정소
                pwr_fansp = pwr_fansp & 0xf0;
                pwr_fansp |= 3;                             // 필터청소 모드는 항상 강풍             
                ESP_LOGI(TAG, "dust_flag : %x", operation_mode);                
             }               
             break;            
        default:
            ESP_LOGI(TAG, "send_cmd_to_erv_DEFAULT: %x", *data);
    }
    
    control_msg[3] = pwr_fansp;
    control_msg[4] = operation_mode;

    control_msg[MSG_LENGTH-1] = make_crc(control_msg);
    gpio_set_level(RX_ENABLE, 1);
    uart_write_bytes(ERV_UART_PORT_NUM, (const char *) control_msg,  MSG_LENGTH);
    uart_wait_tx_done(ERV_UART_PORT_NUM, (20/ portTICK_PERIOD_MS));    
    ESP_LOG_BUFFER_HEXDUMP("send_cmd_to_erv", control_msg, 10, ESP_LOG_INFO);
    gpio_set_level(RX_ENABLE, 0);    
    control_cmd_send_flag = 1;
}

void rcv_msg_parsing(uint8_t *msg){
     //ESP_LOGI(TAG, "rcv_msg_parsing: %x", *msg);
    uint8_t crc = make_crc(msg);
    if(crc != *(msg + 9)){
        ESP_LOGI(TAG, "CRC 에러");
        uart_flush(ERV_UART_PORT_NUM);        
    }
    else if(*msg == 0xB5 || *msg == 0xB6 || *msg == 0xB7){
        // 수신 메시지 데이터로 스마트씽스 관련 데이터 변경   
        //ESP_LOGI(TAG, "xQueueSend: uart_queue");
    #if OTA_DEMO
        if(*msg != 0xB5 ) *(msg + 3) = 50;
        *(msg + 4) = 50;
        *(msg + 5) = 50;
        *(msg + 6) = 50;
        *(msg + 7) = 50;
        *(msg + 8) = 50;
    #endif
        xQueueSend(uart_queue, (void *)msg, (TickType_t)0);
    }
    else {
        ESP_LOGI(TAG, "0xB5, 0xB6 , 0xB7 not found\n");
        uart_flush(ERV_UART_PORT_NUM);
    }
}

static void erv_rcv_task(void *arg)
{
    uint8_t rxBuffer[2];
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE + 1);
    if(data == NULL){
        ESP_LOGI(TAG, "erv_rcv_task malloc error");
    }
    while (1) {
        // UART로 부터 메시지를 수신합니다.
        // len은 수신된 메시지의 길이를 받습니다. 
        int len = uart_read_bytes(ERV_UART_PORT_NUM, data,  (BUF_SIZE  - 1), 20 / portTICK_PERIOD_MS);  
        if (len) {
            control_cmd_send_flag = 0;
            data[len] = '\0';
            //ESP_LOGI(TAG, "len = %d", len);
            char  *str_buf= (char *) malloc(len*10);            // String으로 저장하기 위한 메모리를 할당 받습니다.
            int temp = 0;
            uint8_t *ptr;
            ptr = data;
            for(uint8_t i = 0; i < len; i++){                                                                                                   // temp를 사용하여 수신버퍼 위치를 변경합니다.
                temp += snprintf(str_buf + temp,  30 - temp, "%x ", (unsigned int)*ptr++);     // 수신 버퍼에 있는 1 byte를 str_buf에 넣습니다.
            }
             ESP_LOGI(TAG, "Recv str_buf: %s", (char *) str_buf);
             free(str_buf);
             if(len == 10)  rcv_msg_parsing(data);
             else {
                    uart_flush(ERV_UART_PORT_NUM);
                    ESP_LOGI(TAG, "UART receiving error: %d", len);
             }
        }

         if(xQueueReceive(main_queue, &(rxBuffer), (TickType_t)0)){
            ESP_LOG_BUFFER_HEXDUMP("erv_rcv_task_xQueueReceive", rxBuffer, 2, ESP_LOG_INFO);
            send_cmd_to_erv(rxBuffer);
        }
        // 아래 delay()가 없으면 고정적으로 계속 수행이 되어 watchdog를 검사하는 IDLE task가 동작할 시간이 없다
        // 아니면 menuconfig에서 watchdog을 disable 시켜야 한다
        //vTaskDelay(10/ portTICK_PERIOD_MS);
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }
}

static void erv_uart_init() {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ERV_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ERV_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ERV_UART_PORT_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));    

    // RS-485 Tx/Rx 선택 핀 설정
    gpio_pad_select_gpio(RX_ENABLE);
    gpio_set_direction(RX_ENABLE, GPIO_MODE_OUTPUT);
}

void erv_main(void)
{
    uart_queue = xQueueCreate(5, MSG_LENGTH);
    if(!uart_queue){
        ESP_LOGI(TAG, "Faiiled to create uart_queue = %p", uart_queue);
    }
    const esp_timer_create_args_t uart_timer_args = {
        .callback = &uart_timer_callback,
        .name = "UartTimer"
    };

    const esp_timer_create_args_t control_cmd_timer_args = {
        .callback = &control_cmd_callback,
        .name = "ControlCmdTimer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&uart_timer_args, &uart_timer_handler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(uart_timer_handler, 5000000));         // 3초 타이머

    ESP_ERROR_CHECK(esp_timer_create(&control_cmd_timer_args, &control_cmd_timer_handler));

    erv_uart_init();

    xTaskCreate(erv_rcv_task, "erv_rcv_task", 4096, NULL, 11, NULL);

}
