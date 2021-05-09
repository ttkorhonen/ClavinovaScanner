/*

Keyboad Scanner for Yamaha Clavinova
Inspired by: Moura's Keyboard Scanner: Copyright (C) 2017 Daniel Moura <oxe@oxesoft.com>
Originally hosted at https://github.com/oxesoft/keyboardscanner

This version is extensively modified for Clavinova, with only few traces left of Moura's code.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Visualization using Non-Blocking WS2812Serial and a NeoPixel strip (144 LEDs/m)

#include <WS2812Serial.h>
#define USE_WS2812SERIAL
#include <FastLED.h>

// Number of LEDs is twice the number of keys. 
#define NUM_LEDS 176

#define LED_DATA_PIN 20


#define KEYS_NUMBER 90

#define KEY_OFF               0
#define KEY_START             1
#define KEY_ON                2
#define KEY_RELEASED          3
#define KEY_SUSTAINED         4
#define KEY_SUSTAINED_RESTART 5

#define MIN_TIME_MS   2
#define MAX_TIME_MS   50
#define MAX_TIME_MS_N (MAX_TIME_MS - MIN_TIME_MS)

#define PEDAL_LEFT     40
#define PEDAL_MID      41
#define PEDAL_RIGHT    22


/* Yamaha Clavinova 12 * 15 matrix 
 *  12 output pins, 6 for outer keys (first contact) and 6 for inner keys (final contact)
 *  15 input pins
 */
 
byte output_pins[] = {
    0, 6,
    1, 7,
    2, 8,
    3, 9,
    4, 10,
    5, 11
};
byte input_pins[] = {
    25, 26,
    27, 28,
    29, 30,
    31, 32,
    33, 34,
    35, 36,
    37, 38,
    39
};


//uncomment the next line to inspect the number of scans per seconds
//#define DEBUG_SCANS_PER_SECOND


//uncomment the next line to get text midi message at output
//#define DEBUG_MIDI_MESSAGE

byte          keys_state[KEYS_NUMBER];
unsigned long keys_time[KEYS_NUMBER];
boolean       signals[sizeof(input_pins) * sizeof(output_pins)];
boolean       pedal_enabled;
boolean      r_pedal_on = 0;
boolean      m_pedal_on = 0;
boolean      l_pedal_on = 0;

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() {
    Serial.begin(115200);

    delay(1000);
    pinMode(PEDAL_LEFT, INPUT_PULLUP);
    pinMode(PEDAL_MID, INPUT_PULLUP);
    pinMode(PEDAL_RIGHT, INPUT_PULLUP);

    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    int i;
    for (i = 0; i < KEYS_NUMBER; i++)
    {
        keys_state[i] = KEY_OFF;
        keys_time[i] = 0;
    }
    for (byte pin = 0; pin < sizeof(output_pins); pin++)
    {
        pinMode(output_pins[pin], OUTPUT);
    }
    for (byte pin = 0; pin < sizeof(input_pins); pin++)
    {
        pinMode(input_pins[pin], INPUT_PULLDOWN);
    };

  LEDS.addLeds<WS2812SERIAL,LED_DATA_PIN,BRG>(leds,NUM_LEDS);
  LEDS.setBrightness(84);
  usbMIDI.setHandleNoteOff(myNoteOff);
  usbMIDI.setHandleNoteOn(myNoteOn);
}

void send_midi_event(byte status_byte, byte key_index, unsigned long time)
{
    unsigned long t = time;
    if (t > MAX_TIME_MS)
        t = MAX_TIME_MS;
    if (t < MIN_TIME_MS)
        t = MIN_TIME_MS;
    t -= MIN_TIME_MS;
    unsigned long velocity = 127 - (t * 127 / MAX_TIME_MS_N);
    byte vel = (((velocity * velocity) >> 7) * velocity) >> 7;

    byte key = 19 + key_index;
#ifdef DEBUG_MIDI_MESSAGE
    char out[40];
    sprintf(out, "%02X %02X %03d %lu", status_byte, key, vel, time);
    Serial.println(out);
#else

    switch (status_byte)
    {
      case 0x90:
      {
        usbMIDI.sendNoteOn(key,vel,1);
        leds[key_index*2-5] = CRGB::Green;
        // Show the leds
        FastLED.show();
        break;
      }
      case 0x80:
      {
        usbMIDI.sendNoteOff(key,vel,1);
        leds[key_index*2-5] = CRGB::Black;
        // Show the leds
        FastLED.show();

      }
      
    }

#endif
}
void myNoteOn(byte channel, byte note, byte velocity) {
        leds[(note-19)*2-5] = CRGB::Green;
        // Show the leds
        FastLED.show();
 
}

void myNoteOff(byte channel, byte note, byte velocity) {
         leds[(note-19)*2-5] = CRGB::Black;
        // Show the leds
        FastLED.show();

}


void loop() {
#ifdef DEBUG_SCANS_PER_SECOND
    static unsigned long cycles = 0;
    static unsigned long start = 0;
    static unsigned long current = 0;
    cycles++;
    current = millis();
    if (current - start >= 1000)
    {
        Serial.println(cycles);
        cycles = 0;
        start = current;
    }
#endif
    usbMIDI.read();
    byte pedal = LOW;
    {
        pedal = !digitalRead(PEDAL_RIGHT);
        if(pedal!=r_pedal_on) {
          if(pedal) {
           usbMIDI.sendControlChange(64,127,1);            
          } else  {
            usbMIDI.sendControlChange(64,0,1);           //
          }
          r_pedal_on=pedal;
        }
     }
     {
        pedal = !digitalRead(PEDAL_MID);
        if(pedal!=m_pedal_on) {
          if(pedal) {
           usbMIDI.sendControlChange(66,127,1);
          } else  {
            usbMIDI.sendControlChange(66,0,1);;
          }
          m_pedal_on=pedal;
        }
     }
    {
        pedal = !digitalRead(PEDAL_LEFT);
        if(pedal!=l_pedal_on) {
          if(pedal) {
            usbMIDI.sendControlChange(67,127,1);
          } else  {
            usbMIDI.sendControlChange(67,0,1);
          }
          l_pedal_on=pedal;
        }
     }

    //Scan the matrix
    
    

    for (unsigned int o = 0; o < sizeof(output_pins)/2; o++)
    {
        // first loop to read the outer keys 
        byte output_pin = output_pins[o*2];
        digitalWrite(output_pin, HIGH);
        delayMicroseconds(30); // 30 microsec seems to be enough to avoid key chatter
        for (unsigned int i = 0; i < sizeof(input_pins); i++)
        {
            byte input_pin = input_pins[i];
            signals[(i*12)+(o*2)] = digitalReadFast(input_pin);
        }
        digitalWrite(output_pin, LOW);

        //second loop to read the inner keys
        output_pin=output_pins[o*2+1];
        digitalWrite(output_pin, HIGH);
        delayMicroseconds(30);
        for (unsigned int i = 0; i < sizeof(input_pins); i++)
        {
            byte input_pin = input_pins[i];
            signals[(i*12)+(o*2)+1] = digitalReadFast(input_pin);
        }
        digitalWrite(output_pin, LOW);
    }
    //Check key states
    for (byte key = 0; key < KEYS_NUMBER; key++)
    {
    
        switch (keys_state[key])
        {          
            case KEY_OFF:
                if (signals[key*2])
                {
                    keys_state[key] = KEY_START;
                    keys_time[key] = millis();
                }
                break;
            case KEY_START:
                if (!signals[key*2])
                {
                    keys_state[key] = KEY_OFF;
                    break;
                }
                if (signals[key*2+1])
                {
                    keys_state[key] = KEY_ON;
                    send_midi_event(0x90, key, millis() - keys_time[key]);
                }
                break;
            case KEY_ON:
                if (!signals[key*2+1])
                {
                    keys_state[key] = KEY_RELEASED;
                    keys_time[key] = millis();
                }
                break;
            case KEY_RELEASED:
                if (!signals[key*2])
                {
                    keys_state[key] = KEY_OFF;
                    send_midi_event(0x80, key, millis() - keys_time[key]);
                }
                break;
            }
        }

}
