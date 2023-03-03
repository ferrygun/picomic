#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "pico/binary_info.h"
#include "ei_run_classifier.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt_priv.h"

#include <string.h>
#include <time.h>
#include <ctime>

#define WIFI_SSID ""
#define WIFI_PASSWORD ""
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

const int minutes = .3;
const int threshold = 3; //count label
const float thresh = 0.9; //prediction

int count_label_on = 0;
std::time_t start_time = std::time(0);

typedef struct MQTT_CLIENT_DATA_T {
    mqtt_client_t *mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    uint8_t data[MQTT_OUTPUT_RINGBUF_SIZE];
    uint8_t topic[100];
    uint32_t len;
} MQTT_CLIENT_DATA_T;

MQTT_CLIENT_DATA_T *mqtt;

struct mqtt_connect_client_info_t mqtt_client_info =
{
  "picow",
  "mqtt", /* user */
  "", /* pass */
  0,  /* keep alive */
  NULL, /* will_topic */
  NULL, /* will_msg */
  0,    /* will_qos */
  0     /* will_retain */
#if LWIP_ALTCP && LWIP_ALTCP_TLS
  , NULL
#endif
};

/* Called when publish is complete either with sucess or failure */
static void mqtt_pub_request_cb(void *arg, err_t result)
{
  if(result != ERR_OK) {
    printf("Publish result: %d\n", result);
  }
}



err_t do_publish(mqtt_client_t *client, void *arg)
{
  const char *pub_payload= "Picow MQTT";
  err_t err;
  u8_t qos = 2; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0; /* No don't retain such crappy payload... */
  cyw43_arch_lwip_begin();
  err = mqtt_publish(client, "picow", pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, arg);
  cyw43_arch_lwip_end();
  if(err != ERR_OK) {
    printf("Publish err: %d\n", err);
  }
  return err;
}


static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    printf("mqtt_incoming_data_cb\n");
    MQTT_CLIENT_DATA_T* mqtt_client = (MQTT_CLIENT_DATA_T*)arg;
    LWIP_UNUSED_ARG(data);
    strncpy((char*)mqtt_client->data, (char*)data, len);

    mqtt_client->len=len;
    mqtt_client->data[len]='\0'; 
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
  MQTT_CLIENT_DATA_T* mqtt_client = (MQTT_CLIENT_DATA_T*)arg;
  strcpy((char*)mqtt_client->topic, (char*)topic);

  if (strcmp((const char*)mqtt->topic, "stop") == 0) {
    printf("STOP\n");
  }
  if (strcmp((const char*)mqtt->topic, "start") == 0) {
    printf("START\n");
  }

}

static void mqtt_request_cb(void *arg, err_t err) {
  MQTT_CLIENT_DATA_T* mqtt_client = ( MQTT_CLIENT_DATA_T*)arg;

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" request cb: err %d\n", mqtt_client->mqtt_client_info.client_id, (int)err));
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
  MQTT_CLIENT_DATA_T* mqtt_client = (MQTT_CLIENT_DATA_T*)arg;
  LWIP_UNUSED_ARG(client);

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" connection cb: status %d\n", mqtt_client->mqtt_client_info.client_id, (int)status));

  if (status == MQTT_CONNECT_ACCEPTED) {
    printf("MQTT_CONNECT_ACCEPTED\n");

    //example_publish(client, arg);
    //mqtt_disconnect(client);

    mqtt_sub_unsub(client,
            "start", 0,
            mqtt_request_cb, arg,
            1);
    mqtt_sub_unsub(client,
            "stop", 0,
            mqtt_request_cb, arg,
            1);        
  }
}


/* Example code to extract analog values from a microphone using the ADC
   with accompanying Python file to plot these values
   Connections on Raspberry Pi Pico board, other boards may vary.
   GPIO 26/ADC0 (pin 31)-> AOUT or AUD on microphone board
   3.3v (pin 36) -> VCC on microphone board
   GND (pin 38)  -> GND on microphone board
*/

#define ADC_NUM 0
#define ADC_PIN (26 + ADC_NUM)
#define ADC_VREF 3.3
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))
#define NSAMP 200

float get_axis (int adc_n) {
    unsigned int axis_raw = 0;
    for (int i=0;i<NSAMP;i++){
        axis_raw = axis_raw + (adc_read());
        
    }
    axis_raw = axis_raw/NSAMP;
    float axis_g = axis_raw * ADC_CONVERT;
    return axis_g;
}

int main() {
    stdio_init_all();
    
    mqtt=(MQTT_CLIENT_DATA_T*)calloc(1, sizeof(MQTT_CLIENT_DATA_T));

    if (!mqtt) {
        printf("mqtt client instant ini error\n");
        return 0;
    }

    mqtt->mqtt_client_info = mqtt_client_info;

    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("failed to connect\n");
        return 1;
    }

    ip_addr_t addr;
    if (!ip4addr_aton("192.168.50.XX", &addr)) {
        printf("ip error\n");
        return 0;
    }

    mqtt->mqtt_client_inst = mqtt_client_new();
    mqtt_set_inpub_callback(mqtt->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, mqtt);

    err_t err = mqtt_client_connect(mqtt->mqtt_client_inst, &addr, MQTT_PORT, &mqtt_connection_cb, mqtt, &mqtt->mqtt_client_info);
    if (err != ERR_OK) {
      printf("connect error\n");
      return 0;
    }

    adc_init();
    adc_gpio_init(ADC_PIN);


    while (1) {
        ei_printf("\nStarting inferencing in 2 seconds...\n");
        //sleep_ms(2000);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        ei_printf("Sampling...\n");

        // Allocate a buffer here for the values we'll read from the IMU
        float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

        for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
            // Determine the next tick (and then sleep later)

            uint64_t next_tick = ei_read_timer_us() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
            //ei_printf("Loop: %.3f\n", next_tick - ei_read_timer_us());

            buffer[ix] = get_axis(ADC_NUM);
            sleep_us(next_tick - ei_read_timer_us());
        }

        // Turn the raw buffer in a signal which we can the classify
        signal_t signal;
        int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            return 1;
        }

        // Run the classifier
        ei_impulse_result_t result = { 0 };
 
        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            return 1;
        }

        // print the predictions
        ei_printf("Predictions ");

        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
        ei_printf(": \n");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);

            if (ix == 1 && result.classification[ix].value > thresh) {
              count_label_on++;
            } else {
              //count_label_on = 0;
            }

            if (result.classification[1].value < thresh) {
              count_label_on = 0;
            }

            ei_printf("count_label_on: %d", count_label_on);
            if (count_label_on >= threshold) {
              std::time_t current_time = std::time(0);
              if (current_time - start_time >= minutes * 60) {
                printf("**send-mqtt-**\n");
                do_publish(mqtt->mqtt_client_inst, mqtt);
                count_label_on = 0;
                start_time = std::time(0);
              }
            }

        }

        #if EI_CLASSIFIER_HAS_ANOMALY == 1
            ei_printf("    anomaly score: %.3f\n", result.anomaly);
        #endif


        if(result.classification[1].value >= 0.99) {

        }

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

        
    }

    return 0;
}
