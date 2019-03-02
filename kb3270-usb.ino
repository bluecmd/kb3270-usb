/*
 * Copyright (c) 2019 Christian Svensson <bluecmd@google.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ***************
 *
 * Note: Ensure the keyboard is using AT mode. XT mode is one-directional
 * so it means that the host device has no control over leds.
 * Set Switch 1 under the keyboard to off to enable AT mode.
 *
 * Pinout:
 *  A0: AT clock
 *  A1: AT data
 *
 * The clock is only 12 kHz so instead of using a pin based interrupt
 * we sample it every 62.5 kHz instead.
 *
 * This program uses https://github.com/NicoHood/HID for enhanced
 * HID support.
*/

#define VERSION "1.0rc0 compiled at " __DATE__ " " __TIME__
#include "HID-Project.h"
#include <util/parity.h>

const int pinLed = LED_BUILTIN;
const int pinAtClk = A0;
const int pinAtData = A1;

#define AT_CMD_KB3270_EXTRA        0xDA
#define AT_CMD_SELECT_SCANCODE_SET 0x41
#define AT_CMD_WRITE_LEDS          0xED
#define AT_CMD_RESET               0xFF
#define AT_RESP_SELF_TEST_OK       0xAA
#define AT_RESP_BREAK              0xF0

// Used to track the falling edge of the AT clock
volatile bool isNewCycle = true;
// Whether we or the keyboard is controlling the bus
volatile bool isAtSender = false;
// Indicate that a full new byte has been read
volatile bool newAtByte = false;
// This is the byte read
volatile uint8_t atReadByte;
// This is the byte to send
volatile uint8_t atOutgoingByte;
volatile bool atOutgoingParity;
// Request to start send AT command
volatile bool requestSend = false;
// Pending byte being clocked in
volatile uint8_t atPendingByte = 0;
#define AT_READ_STATE_START  0
#define AT_READ_STATE_PARITY AT_READ_STATE_START + 9
#define AT_READ_STATE_STOP   AT_READ_STATE_PARITY + 1
volatile int atReadState = 0;
#define AT_WRITE_CLOCK_CYCLES    16
#define AT_WRITE_STATE_INIT_CLK  0
#define AT_WRITE_STATE_INIT_DATA AT_WRITE_STATE_INIT_CLK + AT_WRITE_CLOCK_CYCLES
#define AT_WRITE_STATE_RELEASE   AT_WRITE_STATE_INIT_DATA + AT_WRITE_CLOCK_CYCLES
#define AT_WRITE_STATE_PARITY    AT_WRITE_STATE_RELEASE + 9
#define AT_WRITE_STATE_STOP      AT_WRITE_STATE_PARITY + 1
#define AT_WRITE_STATE_ACK       AT_WRITE_STATE_STOP + 1
volatile int atWriteState = 0;
// The last byte was not processed in time
volatile bool atOverrun = false;
// The last AT frame was not correct
volatile bool atFrameError = false;
// Trace AT code in output
bool debugTrace = false;
// Last known keyboard leds
volatile uint8_t kbLeds = 0;
// AT command queue
uint8_t atCmdQueue[256];
uint8_t atCmdQueueEnd = 0;
uint8_t atCmdQueueStart = 0;
// Bootup complete
bool kbBooted = false;
// Previous command was Break
bool kbIsBreaking = false;

const uint8_t scancodes[] = {
  0x00, 0x42, 0x00, 0x3e, 0x3c, 0x3a, 0x3b, 0x75,
  0x68, 0x43, 0x41, 0x3f, 0x3d, 0x2b, 0x35, 0x9c,
  0x69, 0xe2, 0xe1, 0x64, 0xe0, 0x14, 0x1e, 0xa4,
  0x6a, 0x9b, 0x1d, 0x16, 0x04, 0x1a, 0x1f, 0x9a,
  0x6b, 0x06, 0x1b, 0x07, 0x08, 0x21, 0x20, 0xa3,
  0x6c, 0x2c, 0x19, 0x09, 0x17, 0x15, 0x22, 0x99,
  0x6d, 0x11, 0x05, 0x0b, 0x0a, 0x1c, 0x23, 0x00,
  0x6e, 0x00, 0x10, 0x0d, 0x18, 0x24, 0x25, 0x7c,
  0x6f, 0x36, 0x0e, 0x0c, 0x12, 0x27, 0x26, 0x00,
  0x70, 0x37, 0x38, 0x0f, 0x33, 0x13, 0x2d, 0x00,
  0x71, 0x00, 0x34, 0x00, 0x2f, 0x2e, 0x44, 0x72,
  0x39, 0xe5, 0x9e, 0x30, 0x28, 0x31, 0x45, 0x73,
  0x51, 0x50, 0x4a, 0x52, 0x4c, 0x49, 0x2a, 0x4d,
  0xba, 0x59, 0x4f, 0x5c, 0x5f, 0x77, 0x4e, 0x4b,
  0x62, 0x63, 0x5a, 0x5d, 0x5e, 0x60, 0x29, 0x53,
  0x48, 0x58, 0x5b, 0x56, 0xa0, 0x61, 0x47, 0x00,
  0x00, 0x00, 0x00, 0x40, 0x46 };

#define HID_NUM_LOCK    1
#define HID_CAPS_LOCK   2
#define HID_SCROLL_LOCK 4

void setup() {
  pinMode(pinLed, OUTPUT);
  pinMode(pinAtClk, INPUT_PULLUP);
  pinMode(pinAtData, INPUT_PULLUP);

  cli();
  TCCR3A = 0;
  TCNT3 = 0;
  OCR3A = 1;
  TCCR3B = (1<<CS32); // Select 16 Mhz / 256 = 62.5 kHz
  TIMSK3 |= (1<<OCIE3A);
  sei();

  // Sends a clean report to the host
  BootKeyboard.begin();

  Serial.begin(115200);
}

ISR(TIMER3_COMPA_vect)
{
  TCNT3 = 0;
  if (atReadState == atWriteState && atWriteState == 0 && requestSend) {
    isAtSender = true;
    requestSend = false;
  }
  if (isAtSender) {
    atWriteState = isrAtWrite(atWriteState);
  } else {
    atReadState = isrAtRead(atReadState);
  }
}

inline int isrAtWrite(int s) {
  if (s <= AT_WRITE_STATE_RELEASE) {
    // States where we drive the clock
    if (s == AT_WRITE_STATE_INIT_CLK) {
      isNewCycle = true;
      digitalWrite(pinAtClk, LOW);
      pinMode(pinAtClk, OUTPUT);
    } else if (s == AT_WRITE_STATE_INIT_DATA) {
      digitalWrite(pinAtData, LOW);
      pinMode(pinAtData, OUTPUT);
    } else if (s == AT_WRITE_STATE_RELEASE) {
      pinMode(pinAtClk, INPUT_PULLUP);
    }
    return s + 1;
  } else {
    // States where the keyboard drives the lock
    int clkState = digitalRead(pinAtClk);
    int dataVal = digitalRead(pinAtData);
    if (clkState == 0 && isNewCycle) {
      isNewCycle = false;
      if (s < AT_WRITE_STATE_PARITY) {
        bool b = atOutgoingByte & 0x1;
        digitalWrite(pinAtData, b ? HIGH : LOW);
        atOutgoingByte >>= 1;
      } else if (s == AT_WRITE_STATE_PARITY) {
        digitalWrite(pinAtData, atOutgoingParity ? HIGH : LOW);
      } else if (s == AT_WRITE_STATE_STOP) {
        pinMode(pinAtData, INPUT_PULLUP);
      } else if (s == AT_WRITE_STATE_ACK) {
        // Wait for low data for ACK
        if (dataVal == 0) {
          isAtSender = false;
          return 0;
        }
      }
      s++;
      if (s > AT_WRITE_STATE_ACK) {
        s = 0;
        isAtSender = false;
      }
    } else if (clkState == 1) {
      isNewCycle = true;
    }
    return s;
  }
}

inline int isrAtRead(int s) {
  int clkState = digitalRead(pinAtClk);
  int dataVal = digitalRead(pinAtData);
  if (clkState == 0 && isNewCycle) {
    if (s == AT_READ_STATE_START) {
      // TODO(bluecmd): Verify and set atFrameError if invalid
      atPendingByte = 0;
    } else if (s < AT_READ_STATE_PARITY) {
      atPendingByte = dataVal << 7 | atPendingByte >> 1;
    } else if (s == AT_READ_STATE_PARITY) {
      // TODO(bluecmd): Verify and set atFrameError if invalid
    } else if (s == AT_READ_STATE_STOP) {
      // TODO(bluecmd): Verify and set atFrameError if invalid
      atReadByte = atPendingByte;
      if (newAtByte) {
        atOverrun = false;
      }
      newAtByte = true;
    }
    s++;
    if (s > AT_READ_STATE_STOP) {
      s = 0;
    }
    isNewCycle = false;
  } else if (clkState == 1) {
    isNewCycle = true;
  }
  return s;
}

void atCmd(uint8_t cmd) {
  atCmdQueue[atCmdQueueEnd++] = cmd;
}

void atSetLeds(bool num, bool caps, bool scroll) {
  atCmd(AT_CMD_WRITE_LEDS);
  atCmd(caps << 2 | num << 1 | scroll);
}

void debugLine(const char *line) {
  unsigned long t = millis();
  unsigned long secs = t / 1000;
  unsigned long ms = t % 1000;

  Serial.print("[");
  Serial.print(secs);
  Serial.print(".");
  Serial.print(ms);
  if (ms < 10) {
    Serial.print("00");
  } else if (ms < 100) {
    Serial.print("0");
  }
  Serial.print("] ");
  Serial.print(line);
}

void atSendCmd(uint8_t v) {
  atOutgoingByte = v;
  atOutgoingParity = !parity_even_bit(v);
  if (debugTrace) {
    debugLine("Queued AT byte: ");
    Serial.println(v, HEX);
  }
  requestSend = true;
}

void debugCommand(char cmd) {
  if (cmd == 't') {
    debugTrace = !debugTrace;
    debugLine("AT code tracing ");
    Serial.print(debugTrace ? "enabled" : "disabled");
  } else if (cmd == 's' ) {
    debugLine("AT read state ");
    Serial.println(atReadState);
    debugLine("AT pending byte ");
    Serial.println(atPendingByte, HEX);
    debugLine("AT write state ");
    Serial.print(atWriteState);
  } else if (cmd == '?' ) {
    debugLine("Keyboard self-test: ");
    Serial.print(kbBooted ? "OK" : "not OK");
  } else if (cmd == 'r' ) {
    atSendCmd(AT_CMD_RESET);
    debugLine("Forced keyboard reset");
  } else if (cmd == 'l' ) {
    debugLine("Enabling leds ");
    atSetLeds(true, true, true);
  } else {
    debugLine("Unknown command");
  }
  Serial.println("");
}

void loop() {
  bool console = false;

  // Give the keyboard some time to wake up
  delay(1000);

  atSendCmd(AT_CMD_RESET);

  while(true) {

    // Serial debug console activates when a character is read
    if (Serial.available()) {
      if (!console) {
        while (Serial.read() != -1);
        Serial.println("=================================================");
        Serial.println("KB3270-USB conversion made by bluecmd 2019");
        Serial.println("Code at https://github.com/bluecmd/kb3270-usb");
        Serial.println("Version: " VERSION);
        Serial.println("=================================================");
        Serial.println("");
        debugLine("Debug console outputs activated");
        Serial.println("");
        console = true;
      } else {
        char cmd;
        while ((cmd = Serial.read()) != -1) {
          if (cmd == '\n' || cmd == '\r') {
            continue;
          }
          debugCommand(cmd);
        }
      }
    }

    // AT command receive processor
    if (newAtByte) {
      int b = atReadByte;
      newAtByte = 0;
      if (b == AT_RESP_SELF_TEST_OK) {
        kbBooted = true;
        if (console) {
          debugLine("Keyboard self-test OK");
          Serial.println("");
        }
        // Set native scancode set
        atCmd(AT_CMD_KB3270_EXTRA);
        atCmd(AT_CMD_SELECT_SCANCODE_SET);
        atCmd(0x01);
      } else if (b == AT_RESP_BREAK) {
        kbIsBreaking = true;
      } else if (b < sizeof(scancodes)) {
        uint8_t code = scancodes[b];
        if (code != 0) {
          if (kbIsBreaking) {
            BootKeyboard.release(KeyboardKeycode(code));
          } else {
            BootKeyboard.press(KeyboardKeycode(code));
          }
        }
        kbIsBreaking = false;
      }
      if (debugTrace && console) {
        debugLine("Read AT byte: ");
        Serial.println(b, HEX);
      }
    }

    // Only proceed with the actual keyboard functions when the keyboard
    // has booted.
    if (!kbBooted) {
      continue;
    }

    if (kbLeds != BootKeyboard.getLeds()) {
      uint8_t newLeds = BootKeyboard.getLeds();
      bool capsLock = newLeds & HID_CAPS_LOCK;
      bool numLock = newLeds & HID_NUM_LOCK;
      bool scrollLock = newLeds & HID_SCROLL_LOCK;
      atSetLeds(numLock, capsLock, scrollLock);
      kbLeds = newLeds;
      if (console) {
        debugLine("Keyboard leds: ");
        if (numLock) {
          Serial.print("NUM_LOCK ");
        }
        if (capsLock) {
          Serial.print("CAPS_LOCK ");
        }
        if (scrollLock) {
          Serial.print("SCROLL_LOCK ");
        }
        Serial.println("");
      }
    }

    if (atOverrun) {
      // TODO(bluecmd): Beep?
      atOverrun = false;
      if (console) {
        debugLine("AT receive buffer dropped byte");
        Serial.println();
      }
    }

    // AT outgoing command de-queue
    if (atCmdQueueStart != atCmdQueueEnd && !requestSend && !isAtSender) {
      uint8_t v = atCmdQueue[atCmdQueueStart++];
      atSendCmd(v);
    }
  }
}
