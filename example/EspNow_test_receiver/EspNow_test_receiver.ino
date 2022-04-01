/*
 * Multiple connections example
 * Touch to turn on or off the LED
 * This example code is in the public domain.
 */

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
#define NUMPIXELS   58 // Number of LEDs in strip
#define CLOCK_PIN   4
#define DATA_PIN    5
#define TOUCH_PIN   6
#define U1_TXD_PIN  18
#define U1_RXD_PIN  19

#include "driver/uart.h"
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>


/**************************** Task to handle ********************************/
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t Touch_Task_Handle = NULL;
static TaskHandle_t LED_Task_Handle = NULL;

uint32_t Blue  = 0x0000FF;      // 'On' color (starts Blue)
uint32_t Red   = 0x00FF00;      // 'On' color (starts Red)
uint32_t Green = 0xFF0000;      // 'On' color (starts Green)
uint32_t Clear = 0x000000;      // 'Off' color



Adafruit_DotStar strip(NUMPIXELS, DATA_PIN, CLOCK_PIN, DOTSTAR_BRG);

bool LED_state = false;
static const int RX_BUF_SIZE = 1024;
static const char *TX_TASK_TAG = "TX_TASK";


int Brightness = 25;

int Uart1_sendData(const char *logName, const char *data);
void Touch_Task( void *pvParameters );
void LED_Task( void *pvParameters );
bool touch();


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    Serial.printf("Message from %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3],
                  mac[4], mac[5]);
    for (size_t i = 0; i < len; ++i) {
        Serial.print(static_cast<char>(incomingData[i]));
    }
    Serial.println();
    if (incomingData[0] == '+') {
        if (Brightness < 255) {
            Brightness++;
        } else Brightness = 255;
        strip.setBrightness(Brightness);
        strip.show();
        Serial.print("setBrightness(Brightness) +");
    } else if (incomingData[0] == '-') {
        if (Brightness > 1) {
            Brightness--;
        } else Brightness = 1;
        strip.setBrightness(Brightness);
        strip.show();
        Serial.print("setBrightness(Brightness) -");

    } else {
        // digitalWrite(LED_PIN, ledState);
        LED_state = 1 - LED_state;
    }
}


void setup()
{
    Serial.begin(115200);

    //UART1 config
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, U1_TXD_PIN, U1_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);


    pinMode(TOUCH_PIN, INPUT);

    strip.begin(); // Initialize pins for output
    strip.setBrightness(Brightness);
    strip.clear();
    strip.show();  // Turn all LEDs off ASAP



    // init ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != 0) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    //Set the receive data callback function
    esp_now_register_recv_cb(OnDataRecv);


    xTaskCreatePinnedToCore(
        Task_Create
        ,  "Task_Create"   // A name just for humans
        , 4096   // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  &AppTaskCreate_Handle
        ,  ARDUINO_RUNNING_CORE);



    Serial.println("setup done");
}

void loop()
{

}

bool touch()
{
    if (digitalRead(TOUCH_PIN)) {
        delay(50);
        if (digitalRead(TOUCH_PIN)) {
            while ((digitalRead(TOUCH_PIN)));
            return true;
        }
        return false;
    }
}

//Task_Create
static void Task_Create(void *pvParameters)
{
    (void) pvParameters;

    BaseType_t xReturn = pdPASS;


    xReturn = xTaskCreatePinnedToCore(
                  Touch_Task
                  ,  "Touch_Task"
                  ,  1024  // Stack size
                  ,  NULL
                  ,  1  // Priority
                  ,  &Touch_Task_Handle
                  ,  ARDUINO_RUNNING_CORE);
    if (xReturn == pdPASS) {
        Serial.println("Create Touch_Task  succeeded!");
    }



    xReturn = xTaskCreatePinnedToCore(
                  LED_Task
                  ,  "LED_Task"
                  ,  1024  // Stack size
                  ,  NULL
                  ,  1  // Priority
                  ,  &LED_Task_Handle
                  ,  ARDUINO_RUNNING_CORE);
    if (xReturn == pdPASS) {
        Serial.println("Create LED_Task  succeeded!");
    }



    vTaskDelete(AppTaskCreate_Handle); //Delete AppTaskCreate

    for (;;) {


    }

}

//UART1 send data
int Uart1_sendData(const char *logName, const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void Touch_Task(void *pvParameters)
{
    (void) pvParameters;

    for (;;) {

        if (touch()) { //
            if (LED_state == false) {
                Serial.println("ON!");
                Uart1_sendData(TX_TASK_TAG, "H");//Uart1 send ‘H’
                LED_state = true;
            } else if (LED_state == true) {
                Serial.println("OFF!");
                Uart1_sendData(TX_TASK_TAG, "L");//Uart1 send ‘L’
                LED_state = false;
                strip.fill(Clear, 0, NUMPIXELS);
                strip.show();  // Turn all LEDs off ASAP
            }


        }
        if (Serial.available() > 0) {
            int Serial_str = Serial.read();//read uart0 receive
            vTaskDelay(2);
            Serial.println(Serial_str);
            if (Serial_str == 'H') {
                Uart1_sendData(TX_TASK_TAG, "H");//Uart1 send
                LED_state = true;//LED ON
            } else if (Serial_str == 'L') {
                Uart1_sendData(TX_TASK_TAG, "L");//Uart1 send
                LED_state = false;//LED Off
                strip.fill(Clear, 0, NUMPIXELS);
                strip.show();  // Turn all LEDs off ASAP
            }

        }


        vTaskDelay(10);
    }
}

void LED_Task(void *pvParameters)
{
    (void) pvParameters;


    for (;;) {
        if (LED_state) {
            fill(Red, 500);
            fill(Green, 500);
            fill(Blue, 500);

            theaterChase(strip.Color(  0, 255,   0), 50); // Blue,
            theaterChase(strip.Color(255,   0,   0), 50); // Green,
            theaterChase(strip.Color(  0,   0, 255), 50); // Blue,
            theaterChase(strip.Color(255, 255, 255), 50); // White,

            rainbow(10);             // Flowing rainbow cycle along the whole strip
            theaterChaseRainbow(50);

        } else {
            strip.fill(Clear, 0, NUMPIXELS);
            strip.show();  // Turn all LEDs off ASAP

        }
        vTaskDelay(10);
    }
}


// Some functions of our own for creating animated effects -----------------

void colorWipe(uint32_t color, int wait)
{
    for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
        if (!LED_state)return;
        strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
        strip.show();                          //  Update strip to match
        vTaskDelay(wait); //delay(wait);       //  Pause for a moment
    }
}

void fill(uint32_t color, int wait)
{
    if (!LED_state)return;
    strip.fill(color, 0, NUMPIXELS);
    strip.show();  // Refresh strip                       //  Update strip to match
    vTaskDelay(wait); //delay(wait);                           //  Pause for a moment

}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait)
{
    for (int a = 0; a < 10; a++) { // Repeat 10 times...
        for (int b = 0; b < 3; b++) { //  'b' counts from 0 to 2...

            strip.clear();         //   Set all pixels in RAM to 0 (off)
            // 'c' counts up from 'b' to end of strip in steps of 3...
            for (int c = b; c < strip.numPixels(); c += 3) {
                if (!LED_state)return;
                strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
            }
            if (!LED_state)return;
            strip.show(); // Update strip with new contents
            vTaskDelay(wait);//delay(wait);  // Pause for a moment
        }
    }

}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait)
{
    // Hue of first pixel runs 5 complete loops through the color wheel.
    // Color wheel has a range of 65536 but it's OK if we roll over, so
    // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
    // means we'll make 5*65536/256 = 1280 passes through this outer loop:
    for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
        for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...

            // Offset pixel hue by an amount to make one full revolution of the
            // color wheel (range of 65536) along the length of the strip
            // (strip.numPixels() steps):
            int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
            // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
            // optionally add saturation and value (brightness) (each 0 to 255).
            // Here we're using just the single-argument hue variant. The result
            // is passed through strip.gamma32() to provide 'truer' colors
            // before assigning to each pixel:
            strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
            if (!LED_state)return;
        }

        strip.show(); // Update strip with new contents
        vTaskDelay(wait); //delay(wait);  // Pause for a moment
    }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait)
{
    int firstPixelHue = 0;     // First pixel starts at red (hue 0)
    for (int a = 0; a < 30; a++) { // Repeat 30 times...
        for (int b = 0; b < 3; b++) { //  'b' counts from 0 to 2...
            strip.clear();         //   Set all pixels in RAM to 0 (off)
            // 'c' counts up from 'b' to end of strip in increments of 3...
            for (int c = b; c < strip.numPixels(); c += 3) {
                if (!LED_state)return;
                // hue of pixel 'c' is offset by an amount to make one full
                // revolution of the color wheel (range 65536) along the length
                // of the strip (strip.numPixels() steps):
                int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
                uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
                strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
            }
            strip.show();                // Update strip with new contents
            vTaskDelay(wait);//delay(wait);                 // Pause for a moment
            firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
        }
    }
}
