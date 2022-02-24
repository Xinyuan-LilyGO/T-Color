
/*
 * simple example
 * Touch to turn on or off the LED
 * This example code is in the public domain.
 */
#include <Adafruit_DotStar.h>
#include <SPI.h>

#define NUMPIXELS   58 // Number of LEDs in strip
#define CLOCK_PIN   4
#define DATA_PIN    5
#define TOUCH_PIN   6

Adafruit_DotStar strip(NUMPIXELS, DATA_PIN, CLOCK_PIN, DOTSTAR_BRG);

uint32_t Blue  = 0x0000FF;      // 'On' color (starts Blue)
uint32_t Red   = 0x00FF00;      // 'On' color (starts Red)
uint32_t Green = 0xFF0000;      // 'On' color (starts Green)
uint32_t Clear = 0x000000;      // 'Off' color

bool LED_state = false; //0 indicates that the LED is off. 1 indicates that the LED is on. Default LED off

bool touch();

void setup()
{
    Serial.begin(115200);

    pinMode(TOUCH_PIN, INPUT);

    strip.begin(); // Initialize pins for output
    strip.setBrightness(100); //set brightness
    strip.clear();
    strip.show();  // Turn all LEDs off
    Serial.println("setup done");

    strip.fill(Red, 0, NUMPIXELS);
    strip.show();  // Turn all LEDs on

}



void loop()
{
    if (touch()) { //Detection of touch

        if (LED_state) { //led on
            strip.fill(Red, 0, NUMPIXELS);
            strip.show();  // Turn all LEDs on
            Serial.println("ON");
        } else { //led off
            strip.fill(Clear, 0, NUMPIXELS);
            strip.show();  // Turn all LEDs off
            Serial.println("OFF");
        }
    }

}

bool touch()
{

    if (digitalRead(TOUCH_PIN)) {
        delay(50);
        while ((digitalRead(TOUCH_PIN)));
        LED_state = !LED_state;
        return true;
    }
    return false;

}
