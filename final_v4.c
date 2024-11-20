
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "driver/i2c.h"
#include "i2cdev.h"
#include "ds3231.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "ads111x.h"
#include "newlib.h"

//settings
#define EXAMPLE_ESP_WIFI_SSID      "ESPTEAMD"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
#define HOST_IP_ADDR               "192.168.6.7"
#define CONFIG_EXAMPLE_IPV4
#define PORT                        8080
#define USER_INPUTS                 5         
#define MAX_MSG_LENGTH              128      






#define SAMPLE_RATE_HZ 64   // Data rate for the ADS111X, in samples per second
#define SAMPLE_DURATION_SEC 5 // Duration in seconds (1 second)
#define SAMPLE_COUNT (SAMPLE_RATE_HZ * SAMPLE_DURATION_SEC)-3 // Total samples to collect (128 for 1 second)
int8_t binary_buffer[SAMPLE_COUNT];

// Buffer to store ADC values
int16_t adc_buffer[SAMPLE_COUNT];

static char stored_messages[USER_INPUTS][MAX_MSG_LENGTH];
static int message_index = 0;

//Struct To pass as PVParams
typedef struct {
    i2c_config_t pinconfig1;//Passing Pin config
    i2c_dev_t dev_RTC;//rtc
    i2c_dev_t dev_ADC;//adc
    int RTC_interval;//interval
    int message_frmat_choice;//file type
    int No_of_NVS_msg;//No of readings
    int NVS_interval;//How often to log

}tasks_params_t;

//System time struct to store

struct tm time_read;




/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "Final TestV3";

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}



void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    /* Setting a password implies station will connect to all security modes including WEP/WPA.
        * However these modes are deprecated and not advisable to be used. Incase your Access point
        * doesn't support WPA2, these mode can be enabled by commenting below line */

    if (strlen((char *)wifi_config.sta.password)) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}



static void Take_User_inputs(){ 
    char rx_buffer[MAX_MSG_LENGTH];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

#ifdef CONFIG_EXAMPLE_IPV4
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
    struct sockaddr_in6 destAddr;
    inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
    destAddr.sin6_family = AF_INET6;
    destAddr.sin6_port = htons(PORT);
    destAddr.sin6_scope_id = tcpip_adapter_get_netif_index(TCPIP_ADAPTER_IF_STA);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
    inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

    int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        close(sock);
        return;
    }
    ESP_LOGI(TAG, "Successfully connected");

    while (1) {
        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        
        if (len < 0) { // Error occurred during receiving
            ESP_LOGE(TAG, "recv failed: errno %d", errno);
            break;
        } else if (len > 0) { // Data received
            rx_buffer[len] = 0; // Null-terminate the received data
            ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
            ESP_LOGI(TAG, "%s", rx_buffer);

            // Store the received message
            snprintf(stored_messages[message_index], MAX_MSG_LENGTH, "%s", rx_buffer);
            message_index = (message_index + 1) % USER_INPUTS; // Cycle index to avoid overflow

            // Print stored messages
            ESP_LOGI(TAG, "Stored messages:");
            for (int i = 0; i < USER_INPUTS; i++) {
                if (stored_messages[i][0] != '\0') { // Print only non-empty messages
                    ESP_LOGI(TAG, "Message %d: %s", i, stored_messages[i]);
                }
            }

            // Check if the maximum number of messages has been reached
            if (message_index == 0) {
                ESP_LOGI(TAG, "Received User messages. Closing socket...");
                ESP_LOGI(TAG, "Shutting down socket...");
                shutdown(sock, 0);
                close(sock);
                ESP_LOGI(TAG, "Shut Down.");

               
                break;
            }
            ESP_LOGI(TAG, "Here.");

        }

    }
}



void AdcTask(void *pvParameters) {
    int sample_idx = 0;
    int16_t adc_value = 0;
    unsigned long elapsed_time = 0;
    unsigned long start_time=0;
tasks_params_t *params = (tasks_params_t *)pvParameters;

    // Access struct fields
    i2c_dev_t adc_device = params->dev_ADC;
    int interval = params->RTC_interval;
    int message_format = params->message_frmat_choice;
    i2c_config_t pinconfigx=params->pinconfig1;//Passing Pin config
    ads111x_init_desc(&adc_device,ADS111X_ADDR_GND,I2C_NUM_0,GPIO_NUM_2,GPIO_NUM_0);
 
int16_t *adc_buffer = (int16_t *)malloc(SAMPLE_COUNT * sizeof(int16_t));
int16_t *binary_buff = (int16_t *)malloc(SAMPLE_COUNT * sizeof(int16_t));

    if (adc_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for ADC buffer");
        vTaskDelete(NULL); // Terminate the task if memory allocation fails
        return;
    }

while(1){    // Dynamically allocate buffer for storing ADC values
    
    sample_idx=0;
    start_time = xTaskGetTickCount();
    elapsed_time=0;
    while (elapsed_time < pdMS_TO_TICKS(SAMPLE_DURATION_SEC * 1000)) { // Run for specified duration
        // Get ADC value
         ads111x_get_value(&adc_device, &adc_value); 
            // Store ADC value in buffer
            adc_buffer[sample_idx] = adc_value;

            // Increment sample index
            sample_idx++;

            // Log the sample
            ESP_LOGI(TAG, "1");
    

        // Update elapsed time
        elapsed_time = xTaskGetTickCount() - start_time;
    }

    for (int i = 0; i < sample_idx; i++) {
        if (adc_buffer[i] >= 13946) {
            binary_buff[i] = 1; // Store 1 if value is above or equal to threshold
        } else {
            binary_buff[i] = 0; // Store 0 if value is below threshold
        }
    }


    
    // Print collected data
    ESP_LOGI(TAG, "ADC Values:");
    for (int i = 0; i < sample_idx; i++) {
        ESP_LOGI(TAG, "Sample %d: %d", i + 1, binary_buff[i]); // Print each sample value
    }


    float transition_count = 0.0;
    float total_period = 0.0;
    int sample_count=sample_idx;
    // Iterate through the buffer and count the transitions
    for (int i = 1; i < sample_count; i++) {
        if (binary_buffer[i] != binary_buffer[i - 1]) { // A transition occurs
            transition_count++;
            // Calculate the period in terms of samples
          ; // Each transition is 1 sample apart for this simple case
        }

        
    }


        float half_trans=transition_count/2;
        float freq_hz=half_trans/SAMPLE_DURATION_SEC;
        float bpm=freq_hz*60;
        int bpm_int=int(bpm);
        float rem=bpm-bpm_int;
        
        int rem *= 100;

        int dp1=(int)rem / 10;
        int fraction_part2 = (int)fraction_part % 10;

        printf("%d . %d %d\n",bpm_int,dp1,fraction_part2);

        ESP_LOGI(TAG,"Frequency%.2f",bpm);
    // Free allocated memory
    free(binary_buff);
    free(adc_buffer);
    ESP_LOGI(TAG, "ADC Task completed");
        vTaskDelay(30000/portTICK_RATE_MS);

}
    vTaskDelete(NULL); // Terminate the task
}


void app_main(){



//Connecting to Wifi
wifi_init_sta();

// Taking 5 User Inputs 
Take_User_inputs();

//Print every 30s by default
int Interval_At_which_to_print=30;
int i=0;

//Print Values interval Selection 

        if (strstr(stored_messages[i], "a") != NULL) {
            Interval_At_which_to_print = 30;  // Assign value for Option A
        } else if (strstr(stored_messages[i], "b") != NULL) {
            Interval_At_which_to_print = 60;  // Assign value for Option B
        } else if (strstr(stored_messages[i], "c") != NULL) {
            Interval_At_which_to_print = 90;  // Assign value for Option C
        }else if (strstr(stored_messages[i], "d") != NULL) {
            Interval_At_which_to_print = 120;  // Assign value for Option C
        }

i++;

// Allowing_the user to specify how often to write to the Non Volotile Storage

    int Interval_At_which_to_log=1;

      if (strstr(stored_messages[i], "a") != NULL) {
            Interval_At_which_to_log = 1;  // Assign value for Option A
        } else if (strstr(stored_messages[i], "b") != NULL) {
            Interval_At_which_to_log = 2;  // Assign value for Option B
        } else if (strstr(stored_messages[i], "c") != NULL) {
            Interval_At_which_to_log = 3;  // Assign value for Option C
        }else if (strstr(stored_messages[i], "d") != NULL) {
            Interval_At_which_to_log = 4;  // Assign value for Option C
        }

        
// Allowing the user to specify the file format

i++;

    int File_Format=0;
    
    if (strstr(stored_messages[i], "a") != NULL) {
            File_Format = 0;  // Assign value for Option A
        } else if (strstr(stored_messages[i], "b") != NULL) {
            File_Format = 1;  // Assign value for Option B
        }

i++;

// Allowing The User To indicate No of Readings to be stored

int Reading_Numbers=5;
 
    if (strstr(stored_messages[i], "a") != NULL) {
            Reading_Numbers = 5;  // Assign value for Option A
        } else if (strstr(stored_messages[i], "b") != NULL) {
            Reading_Numbers = 10;  // Assign value for Option C
        }
          if (strstr(stored_messages[i], "c") != NULL) {
            Reading_Numbers = 15;  // Assign value for Option A
        } else if (strstr(stored_messages[i], "d") != NULL) {
            Reading_Numbers = 20;  // Assign value for Option D
        }




    // Log the selections
    printf("User Selections Based\n");
    printf("------------------------------------\n");
    printf("Interval at which to Display: %d seconds\n", Interval_At_which_to_print);
    printf("Interval at which to log: %d seconds\n", Interval_At_which_to_log*Interval_At_which_to_print);
    printf("File Format: %s\n", (File_Format == 0) ? "Format A" : "Format B");
    printf("Number of Readings to be stored: %d\n", Reading_Numbers);
    printf("------------------------------------\n");


     
    i2cdev_init();
    i2c_config_t pinconfig;


    pinconfig.mode=I2C_MODE_MASTER;

    //disabling pullups
    pinconfig.sda_pullup_en=GPIO_PULLUP_DISABLE;
    pinconfig.scl_pullup_en=GPIO_PULLUP_DISABLE;


    //Setting SCL as IO 0
    pinconfig.scl_io_num=GPIO_NUM_0;

    //Setting SDA as IO2
    pinconfig.sda_io_num=GPIO_NUM_2;

    //wait for 1000 tics
    pinconfig.clk_stretch_tick = 1000;


i2c_dev_t dev1;
    memset(&dev1, 0, sizeof(i2c_dev_t));
    dev1.addr=DS3231_ADDR;
    dev1.cfg=pinconfig;
    dev1.port=I2C_NUM_0;

i2c_dev_t dev2;
    memset(&dev2, 0, sizeof(i2c_dev_t));
    dev2.addr=ADS111X_ADDR_GND;
    dev2.cfg=pinconfig;
    dev2.port=I2C_NUM_0;

//Setting up the ADC

ads111x_init_desc(&dev2,ADS111X_ADDR_GND,I2C_NUM_0,GPIO_NUM_2,GPIO_NUM_0);
ads111x_set_gain(&dev2,ADS111X_GAIN_4V096);
ads111x_set_input_mux(&dev2,ADS111X_MUX_0_GND);
ads111x_set_mode(&dev2,ADS111X_MODE_CONTINUOUS);
ads111x_set_data_rate(&dev2,ADS111X_DATA_RATE_128);






esp_err_t installer = ds3231_init_desc(&dev2,0,GPIO_NUM_2,GPIO_NUM_0);
    if (installer==ESP_OK){

        printf("Set up Properly \n");
    }

tasks_params_t Values;
Values.dev_RTC=dev1;
Values.dev_ADC=dev2;
Values.RTC_interval=Interval_At_which_to_print;
Values.NVS_interval=Interval_At_which_to_log*Interval_At_which_to_print;
Values.message_frmat_choice=File_Format;
Values.pinconfig1=pinconfig;


    xTaskCreate(AdcTask, "AdcTask", 4096, &Values, 5, NULL);
}

