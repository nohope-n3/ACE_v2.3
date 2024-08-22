// #define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_ESP32S3_EYE
// #define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
#define PIN_MOTOR_A_PWM 12
#define PIN_MOTOR_B_PWM 13
#elif defined(CAMERA_MODEL_ESP32S3_EYE)
#define PIN_MOTOR_A_PWM 41
#define PIN_MOTOR_B_PWM 42
#endif

#define DEBUG_MODE

#include "DCMotorControl.h"
#include "camera_pins.h"
#include "driver/uart.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <cstring>
#include <stdio.h>

const char *ssid = "P109";
const char *password = "12345678";

WiFiUDP Udp;
const char *udpAddressLaptop = "192.168.109.106";
const uint16_t udpPortLaptop = 3000;
const uint16_t udpPortCam = 3001;

// uint32_t sendTime = 0;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 10,
    .fb_count = 1,
};
#if defined(DEBUG_MODE)
void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);
}
#endif

esp_err_t camera_init() {
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        printf("Camera Init Failed\n");
        return err;
    }

    sensor_t *pSensor = esp_camera_sensor_get();

    pSensor->set_vflip(pSensor, 0);
    pSensor->set_hmirror(pSensor, 0);

    printf("Camera Init OK\n");
    return ESP_OK;
}

void wifi_init() {
    WiFi.begin(ssid, password);
    Serial.println("");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    printf("Connected to WiFi\n");
    Udp.begin(udpPortCam);
    printf("Now listening at IP %s port %d\n", WiFi.localIP().toString().c_str(), udpPortCam);
}

void sendImageTask(void *pvParameters) {
    camera_fb_t *fb = NULL;
    printf("Send image\n");
    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            printf("Camera Capture Failed\n");
            delay(100);
            continue;
        }

        // if (sendTime == 0) {
        //     sendTime = esp_timer_get_time();
        // }

        Udp.beginPacket(udpAddressLaptop, udpPortLaptop);
        Udp.write(fb->buf, fb->len);
        Udp.endPacket();
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void receiveMessageTask(void *pvParameters) {
    DCMotorControl motorControl(PIN_MOTOR_A_PWM, PIN_MOTOR_B_PWM);
    uint8_t packetSize;
    char MovementCmd[15];
    uint8_t directionLength;
    uint8_t speedLength;
    uint8_t alphaLength;

    char *directionStr;
    char *speedStr;
    char *alphaStr;
    char *spacePos1;
    char *spacePos2;

    uint8_t unDirection;
    uint8_t unSpeed;
    int8_t nAlpha;

    uint8_t len;

    printf("Listen\n");
    while (true) {
        packetSize = Udp.parsePacket();
        if (packetSize) {
            len = Udp.read(MovementCmd, 15);
            if (len > 0) {
                MovementCmd[len] = '\0';
                printf("Command: %s\n", MovementCmd);

                // Find the positions of the first and second spaces
                spacePos1 = strchr(MovementCmd, ' ');
                spacePos2 = nullptr;
                if (spacePos1 != nullptr) {
                    spacePos2 = strchr(spacePos1 + 1, ' ');
                }

                if (spacePos1 != nullptr && spacePos2 != nullptr) {
                    // Calculate lengths of substrings
                    directionLength = spacePos1 - MovementCmd;
                    speedLength = spacePos2 - (spacePos1 + 1);
                    alphaLength = len - (spacePos2 - MovementCmd) - 1;

                    // Extract substrings directly using pointers
                    directionStr = MovementCmd;
                    speedStr = spacePos1 + 1;
                    alphaStr = spacePos2 + 1;

                    // Convert direction, speed, and alpha to integer values
                    unDirection = atoi(directionStr);
                    unSpeed = atoi(speedStr);
                    nAlpha = atoi(alphaStr);

                    printf("Dir: %d, Speed: %d, Alpha: %d\n", unDirection, unSpeed, nAlpha);

                    // Control car movement based on received command
                    motorControl.CarMovementControl(unDirection, unSpeed, nAlpha);
                } else {
                    printf("Invalid command format\n");
                }
                // if (sendTime != 0) {
                //     uint32_t currentTime = esp_timer_get_time();
                //     uint32_t delayTime = currentTime - sendTime;
                //     printf("Delay time: %llu microseconds\n", delayTime);
                //     sendTime = 0;
                // }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup() {
    uart_init();
    camera_init();
    wifi_init();

    // Create tasks
    xTaskCreatePinnedToCore(
        sendImageTask,
        "SendImageTask",
        8192,
        NULL,
        2,
        NULL,
        1);

    xTaskCreatePinnedToCore(
        receiveMessageTask,
        "RecvMsgTask",
        4096,
        NULL,
        1,
        NULL,
        0);
}

void loop() {
}
