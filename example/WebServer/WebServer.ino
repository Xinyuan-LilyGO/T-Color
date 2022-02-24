#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include "driver/uart.h"

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
static TaskHandle_t WebServer_Handle = NULL;


static const int RX_BUF_SIZE = 1024;
static const char *TX_TASK_TAG = "TX_TASK";

#define WIFI_AP_MODE  true  //Uncomment select WIFI_AP mode
//#define WIFI_STA_MODE true   //Uncomment select WIFI_STA mode
#ifdef WIFI_STA_MODE
const char *ssid = "SSID"; // SSID
const char *password = ""; // Password
#elif WIFI_AP_MODE

const char *ssid = "T-Color"; // SSID
const char *password = ""; // Password
#endif

uint32_t Blue  = 0x0000FF;      // 'On' color (starts Blue)
uint32_t Red   = 0x00FF00;      // 'On' color (starts Red)
uint32_t Green = 0xFF0000;      // 'On' color (starts Green)
uint32_t Clear = 0x000000;      // 'Off' color

bool LED_state = false;
Adafruit_DotStar strip(NUMPIXELS, DATA_PIN, CLOCK_PIN, DOTSTAR_BRG);



bool LEDStatus;
// Commands sent through Web Socket
const char ON[] = "ON";
const char OFF[] = "OFF";

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name = "viewport" content = "width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
<title>T-Color WebSocket Demo</title>
<style>
"body { background-color: #808080; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }"
</style>
<script>
var websock;
function start() {
  websock = new WebSocket('ws://' + window.location.hostname + ':81/');
  websock.onopen = function(evt) { console.log('websock open'); };
  websock.onclose = function(evt) { console.log('websock close'); };
  websock.onerror = function(evt) { console.log(evt); };
  websock.onmessage = function(evt) {
    console.log(evt);
    var e = document.getElementById('ledstatus');
    if (evt.data === 'ON') {
      e.style.color = 'red';
    }
    else if (evt.data === 'OFF') {
      e.style.color = 'black';
    }
    else {
      console.log('unknown event');
    }
  };
}
function buttonclick(e) {
  websock.send(e.id);
}
</script>
</head>
<body onload="javascript:start();">
<h1>T-Color WebSocket Demo</h1>
<div id="ledstatus"><b>LED</b></div>
<button id="ON"  type="button" style="width:100px;height:60px" background-color="green" onclick="buttonclick(this);">On</button> 
<button id="OFF" type="button" style="width:100px;height:60px" background-color="red"  onclick="buttonclick(this);">Off</button>
</body>
</html>
)rawliteral";


WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

static void Task_Create(void *pvParameters);
void Task_webSocket(void *pvParameters);
void Touch_Task(void *pvParameters);
void LED_Task(void *pvParameters);
bool touch();
int Uart1_sendData(const char *logName, const char *data);

void setup() {
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


#ifdef WIFI_STA_MODE
  /* Connect WiFi */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

#elif WIFI_AP_MODE
 WiFi.softAP(ssid, password);

  if (MDNS.begin("T-Color")) {
    Serial.println("MDNS responder started");
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ws", "tcp", 81);
    }
      else {
    Serial.println("MDNS.begin failed");
  }
    Serial.println("mDNS responder started");
    Serial.println("Connect to http://T-Color.local");

#endif

  
  strip.begin(); // Initialize pins for output
  strip.setBrightness(50);
  strip.fill(Clear, 0,NUMPIXELS);
  strip.show();  // Turn all LEDs off ASAP


  server.on("/", handleRoot);

  server.onNotFound(handleNotFound);
  //here the list of headers to be recorded
  const char * headerkeys[] = {"User-Agent", "Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys) / sizeof(char*);
  //ask server to track these headers
  server.collectHeaders(headerkeys, headerkeyssize);
  server.begin();
  Serial.println("HTTP server started");

  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

    xTaskCreatePinnedToCore(
        Task_Create
        ,  "Task_Create"   // A name just for humans
        , 4096   // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  &AppTaskCreate_Handle
        ,  ARDUINO_RUNNING_CORE);

}

void loop() {
  
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
        delay(50);
        if (digitalRead(TOUCH_PIN)) {
            while ((digitalRead(TOUCH_PIN)));
            return true;
        }
        return false;
    }
}
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

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
                  WebServer_Task
                  ,  "WebServer_Task"
                  ,  4096  // Stack size
                  ,  NULL
                  ,  1  // Priority
                  ,  &WebServer_Handle
                  ,  ARDUINO_RUNNING_CORE);
    if (xReturn == pdPASS) {
        Serial.println("Create WebServer_Task  succeeded!");
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


  for (;;)
  {
    if (LED_state)
    {
      fill(Red, 500);
      fill(Green, 500);
      fill(Blue, 500);

      theaterChase(strip.Color(  0, 255,   0), 50); // Blue, 
      theaterChase(strip.Color(255,   0,   0), 50); // Green,
      theaterChase(strip.Color(  0,   0, 255), 50); // Blue,
      theaterChase(strip.Color(255, 255, 255), 50); // White,

      rainbow(10);             // Flowing rainbow cycle along the whole strip
      theaterChaseRainbow(50);

  }
  else{
    
    strip.fill(Clear, 0, NUMPIXELS);
    strip.show();  // Refresh strip                       //  Update strip to match
  }
   vTaskDelay(10);  
}
}

void WebServer_Task(void *pvParameters)  
{
  (void) pvParameters;

  for (;;)
  {
   webSocket.loop();
   server.handleClient();
   vTaskDelay(10); 
}
}


//root page can be accessed only if authentification is ok
void handleRoot() {

  server.send_P(200, "text/html", INDEX_HTML);
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
	const uint8_t* src = (const uint8_t*) mem;
	Serial1.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
	for(uint32_t i = 0; i < len; i++) {
		if(i % cols == 0) {
			Serial1.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
		}
		Serial1.printf("%02X ", *src);
		src++;
	}
	Serial1.printf("\n");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
  Serial.printf("webSocketEvent(%d, %d, ...)\r\n", num, type);
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\r\n", num);
      break;
    case WStype_CONNECTED:
  {
IPAddress ip = webSocket.remoteIP(num);
Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);
// Send the current LED status
if (LEDStatus) {
  webSocket.sendTXT(num, ON, strlen(ON));
}
else {
  webSocket.sendTXT(num, OFF, strlen(OFF));
}
  }
  break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\r\n", num, payload);

      if (strcmp(ON, (const char *)payload) == 0) {
        Uart1_sendData(TX_TASK_TAG, "H");//Uart1 send ‘H’
        LED_state=true;
      }
      else if (strcmp(OFF, (const char *)payload) == 0) {
        Uart1_sendData(TX_TASK_TAG, "L");//Uart1 send ‘L’
        LED_state=false;
        strip.fill(Clear, 0,NUMPIXELS);
        strip.show();  // Turn all LEDs off ASAP
      }
      else {
        Serial.println("Unknown command");
      }
      // send data to all connected clients
      webSocket.broadcastTXT(payload, length);
      break;
    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\r\n", num, length);
      hexdump(payload, length);

      // echo data back to browser
      webSocket.sendBIN(num, payload, length);
      break;
    default:
      Serial.printf("Invalid WStype [%d]\r\n", type);
      break;
  }
}


void fill(uint32_t color, int wait) {
  if (!LED_state)return;
    strip.fill(color, 0, NUMPIXELS);
    strip.show();  // Refresh strip                       //  Update strip to match
    vTaskDelay(wait); //delay(wait);                           //  Pause for a moment

}


// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
     
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
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
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
    for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    
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
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
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
