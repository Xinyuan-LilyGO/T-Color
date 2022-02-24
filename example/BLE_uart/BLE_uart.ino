/*
 BLE control led  example
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "driver/uart.h"
#include <Adafruit_DotStar.h>
#include <SPI.h>

#define NUMPIXELS   58 // Number of LEDs in strip
#define CLOCK_PIN   4
#define DATA_PIN    5
#define TOUCH_PIN   6
#define U1_TXD_PIN  18
#define U1_RXD_PIN  19

/**************************** Task to handle ********************************/
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t Touch_Task_Handle = NULL;
static TaskHandle_t LED_Task_Handle = NULL;
static TaskHandle_t BLEService_Handle = NULL;

static const int RX_BUF_SIZE = 1024;
static const char *TX_TASK_TAG = "TX_TASK";

uint32_t Blue  = 0x0000FF;      // 'On' color (starts Blue)
uint32_t Red   = 0x00FF00;      // 'On' color (starts Red)
uint32_t Green = 0xFF0000;      // 'On' color (starts Green)
uint32_t Clear = 0x000000;      // 'Off' color


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

Adafruit_DotStar strip(NUMPIXELS, DATA_PIN, CLOCK_PIN, DOTSTAR_BRG);

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool LED_state = false;

bool touch();
int Uart1_sendData(const char *logName, const char *data);
static void Task_Create(void *pvParameters);
void Touch_Task(void *pvParameters);
void LED_Task(void *pvParameters);

class MyServerCallbacks: public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0) {
            Serial.println("*********");
            Serial.print("Received Value: ");
            for (int i = 0; i < rxValue.length(); i++)
                Serial.print(rxValue[i]);

            Serial.println();
            Serial.println("*********");

            if (rxValue[0] == 'H') {
                LED_state = true;
                Uart1_sendData(TX_TASK_TAG, "H");//Uart1 send ‘H’
            } else  if (rxValue[0] == 'L') {
                Uart1_sendData(TX_TASK_TAG, "L");//Uart1 send ‘L’
                LED_state = false;
            }


        }
    }
};

void setup()
{
    Serial.begin(115200);
    delay(200);
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

    // Create the BLE Device
    BLEDevice::init("T-Color");

    strip.begin(); // Initialize pins for output
    strip.setBrightness(50);
    strip.clear();
    strip.show();  // Turn all LEDs off



    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID_TX,
                            BLECharacteristic::PROPERTY_NOTIFY
                        );

    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID_RX,
            BLECharacteristic::PROPERTY_WRITE
                                           );

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");


    xTaskCreatePinnedToCore(
        Task_Create
        ,  "Task_Create"   // A name just for humans
        , 4096   // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  &AppTaskCreate_Handle
        ,  ARDUINO_RUNNING_CORE);

}

void loop()
{

}

//UART1 send data
int Uart1_sendData(const char *logName, const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

bool touch()
{
    if (digitalRead(TOUCH_PIN)) {
        while ((digitalRead(TOUCH_PIN)));
        return true;
    }
    return false;
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

    xReturn = xTaskCreatePinnedToCore(
                  BLEService_Task
                  ,  "BLEService_Task"
                  ,  4096  // Stack size
                  ,  NULL
                  ,  1  // Priority
                  ,  &BLEService_Handle
                  ,  ARDUINO_RUNNING_CORE);
    if (xReturn == pdPASS) {
        Serial.println("Create BLEService_Task  succeeded!");
    }

    vTaskDelete(AppTaskCreate_Handle); //Delete AppTaskCreate

    for (;;) {


    }

}

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
            strip.show();  // Refresh strip                       //  Update strip to match
        }
        vTaskDelay(10);
    }
}

void BLEService_Task(void *pvParameters)
{
    (void) pvParameters;

    for (;;) {
        // disconnecting
        if (!deviceConnected && oldDeviceConnected) {
            delay(500); // give the bluetooth stack the chance to get things ready
            pServer->startAdvertising(); // restart advertising
            Serial.println("start advertising");
            oldDeviceConnected = deviceConnected;
        }
        // connecting
        if (deviceConnected && !oldDeviceConnected) {
            // do stuff here on connecting
            oldDeviceConnected = deviceConnected;
        }
        vTaskDelay(10);
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

