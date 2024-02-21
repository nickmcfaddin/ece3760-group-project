// === INLCUDE STATEMENTS =========================================================================
#include "WiFi.h"
#include "esp_now.h"
#include <Adafruit_NeoPixel.h>

// === DEFINE STATEMENTS ==========================================================================
#define BAUD_RATE 115200
#define SYS_DELAY 100

#define SKIP  0
#define SWEEP 1

#define LEFT  1
#define RIGHT 2

#define FALLING_EDGE 1
#define RISING_EDGE  2

#define PB_1_PIN 36
#define PB_2_PIN 39
#define PB_3_PIN 32

#define SW_1_PIN 25
#define SW_2_PIN 26

#define JOYSTICK_X_PIN 33
#define JOYSTICK_Y_PIN 32
#define JOYSTICK_SW_PIN 4

#define RGB1_R_PIN 25
#define RGB1_G_PIN 26
#define RGB1_B_PIN 27
#define RGB2_R_PIN 14
#define RGB2_G_PIN 12
#define RGB2_B_PIN 13

#define RGB1_R_CHANNEL 0
#define RGB1_G_CHANNEL 1
#define RGB1_B_CHANNEL 2
#define RGB2_R_CHANNEL 3
#define RGB2_G_CHANNEL 4
#define RGB2_B_CHANNEL 5

#define LED_STRIP_PIN  1
#define LED_NUM_PIXELS 12

#define PWM_FREQUENCY 5000  // 5000Hz frequency
#define PWM_RESOLUTION   8  // 8 bits of resolution (0-255)

// Pre-defined Colors
#define RED       0xFF0000
#define YELLOW    0xFFBB00
#define GREEN     0x00FF00
#define TURQUIOSE 0x00FFFF
#define BLUE      0x0000FF
#define PURPLE    0xFF00FF
#define WHITE     0xFFFFFF

#define LOWER_THRESHOLD      1200
#define UPPER_THRESHOLD      2800
#define CLEAR_THRESHOLD      900
#define SWITCH_OFF_THRESHOLD 100


// === STRUCTS & ENUMS ============================================================================

typedef struct esp_now_packet_struct {
  int leftCmd;   // 0: clear, 1: low, 2: mid, 3: high
  int rightCmd;  // 0: clear, 1: low, 2: mid, 3: high
} esp_now_packet;


// === GLOBAL VARIABLES ===========================================================================

int deviceType = SWEEP;  // Used to determine if the device should act like a [SKIP] or [SWEEP] device

int skipInputPins[] = {PB_1_PIN, PB_2_PIN, JOYSTICK_X_PIN, JOYSTICK_X_PIN, JOYSTICK_X_PIN};  //     [SKIP]
int sweepInputPins[] = {PB_1_PIN, SW_1_PIN, SW_2_PIN};  //                                          [SWEEP]

int jsValues[] = {0, 0, 0};  // Joystick (JS) last EMA values {x, y, sw}                            [SKIP]

int swPosition = 0;  // Switch (SW) current position                                                [SWEEP]

bool pbLastValue[] = {false,    false};  // Push Button (PB) last read state                        [SKIP/SWEEP]
bool pbStates[] =    {false,    false};  // Push Button (PB) logical value                          [SKIP/SWEEP]

int rgb1Pins[] =     {RGB1_R_PIN,     RGB1_G_PIN,     RGB1_B_PIN    };  // RGB LED 1 PWM pins       [SKIP]
int rgb2Pins[] =     {RGB2_R_PIN,     RGB2_G_PIN,     RGB2_B_PIN    };  // RGB LED 2 PWM pins       [SKIP]
int rgb1Channels[] = {RGB1_R_CHANNEL, RGB1_G_CHANNEL, RGB1_B_CHANNEL};  // RGB LED 1 PWM channels   [SKIP]
int rgb2Channels[] = {RGB2_R_CHANNEL, RGB2_G_CHANNEL, RGB2_B_CHANNEL};  // RGB LED 2 PWM channels   [SKIP]

int pbLeft =  0;  // Index for aliasing push button (PB) left                                       [SKIP]
int pbRight = 1;  // Index for aliasing push button (PB) right                                      [SKIP]
int jsX =  0;     // Index for aliasing joystick (JS) x position                                    [SKIP]
int jsY =  1;     // Index for aliasing joystick (JS) y position                                    [SKIP]
int jsSW = 2;     // Index for aliasing joystick (JS) switch (sw)                                   [SKIP]
int* leftLED = rgb1Channels;   // Aliase for RGB LED left PWM channels                              [SKIP]
int* rightLED = rgb2Channels;  // Aliase for RGB LED right PWM channels                             [SKIP]

int pbPower = 0;  // Index for aliasing push button (PB) power                                      [SWEEP]

Adafruit_NeoPixel pixels(LED_NUM_PIXELS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);  // LED Control      [SWEEP]

esp_now_packet packet;  // Instance of ESP-NOW packet to send to connected clients                  [SKIP/SWEEP]
esp_now_peer_info_t peerInfo;  // Information about connected peers                                 [SKIP]

uint8_t hostAddress[] =   {0x40, 0x22, 0xD8, 0xEA, 0x76, 0x30};  // TODO: determine dynamically     [SWEEP]
uint8_t clientAddress[] = {0x40, 0x22, 0xD8, 0xEE, 0x6D, 0xE0};  // TODO: determine dynamically     [SKIP]


// === MAIN PROGRAM ===============================================================================

void setup() {
  // Configure Serial Communication
  configureSerial();

  // Configure Wifi
  WiFi.mode(WIFI_MODE_STA);
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());  // TODO: fix bug on SWEEP devices (MAC gets printed as non-sense)

  if (deviceType == SKIP) {
  
    // Configure input GPIOs
    configureGPIOs(skipInputPins, INPUT, 5);  // configure all input GPIOs

    // Configure PWM channels
    configurePWM(rgb1Pins, rgb1Channels, 3);  // configure PWM for RGB LED 1
    configurePWM(rgb2Pins, rgb2Channels, 3);  // configure PWM for RGB LED 2

    // Configure ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_register_send_cb(sent);

    // Register peer (TODO: setup mulitple peers)
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Register first peer (TODO: have 'pairing' process for dynamic connection)
    memcpy(peerInfo.peer_addr, clientAddress, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }

    delay(200);  // Extra delay to ensure ESP-NOW has enough time to configure

    // Initialize LED colors
    setRGBA(rgb1Channels, BLUE, 0.25);
    setRGBA(rgb2Channels, BLUE, 0.25);

  }

  if (deviceType == SWEEP) {
    
    // Configure input GPIOs
    configureGPIOs(sweepInputPins, INPUT, 3);  // configure all input GPIOs

    // Configure NeoPixel
    pixels.begin();

    // Configure ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_register_recv_cb(recieve);

    // Initialize LED colors
    setStripRGBA(BLUE, 0.10);
  }
}

void loop() {

  if (deviceType == SKIP) {
    // Update push button (PB) states
    bool currentValues[] = {digitalRead(PB_1_PIN), digitalRead(PB_2_PIN)};
    updateButtonStates(currentValues, 2);
    // Update joystick (JS) values
    updateJoystickValues();
    // Perform Skip Logic
    skipLogic();
  }

  if (deviceType == SWEEP) {
    // Update push button (PB) states
    bool currentValues[] = {digitalRead(PB_1_PIN)};
    updateButtonStates(currentValues, 1);
    // Update switch (SW) position
    updateSwitchPosition();
    // Perform Sweep Logic
    sweepLogic();
  }
  
  delay(SYS_DELAY);  // System delay on every iteration
}


// === GPIO FUNCTIONS =============================================================================

void configureGPIOs(int* pins, int direction, int count) {
  for (int i = 0; i < count; i++) {
    pinMode(pins[i], direction);
  }
}

void configurePWM(int* pin, int* channel, int count) {
  for (int i = 0; i < count; i++) {
    ledcSetup(channel[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(pin[i], channel[i]);
  }
}

// Set the Red (R) Blue (B) Green (G) and Alpha (A) values for a PWM controlled RGB LED
void setRGBA(int* channels, int rgb, float alpha) {
  ledcWrite(channels[0], (int)(((rgb >> 16) & 0xFF) * alpha));
  ledcWrite(channels[1], (int)(((rgb >> 8 ) & 0xFF) * alpha));
  ledcWrite(channels[2], (int)(((rgb >> 0 ) & 0xFF) * alpha));
}

void setStripRGBA(int rgb, float alpha) {
  pixels.clear();
  for (int i = 0; i < LED_NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(
      (int)(((rgb >> 16) & 0xFF) * alpha),
      (int)(((rgb >> 8 ) & 0xFF) * alpha),
      (int)(((rgb >> 0 ) & 0xFF) * alpha)
    ));
  }
  pixels.show();
}

void pulseRGB(int* channel, int rgb, int count, int step, int time) {
  for (int i = 0; i < count; i++) {
    // Fade In
    for (int fade = 1; fade <= step; fade++) {
      setRGBA(channel, rgb, (float)fade / step);
      delay(time);
    }
    // Fade Out
    for (int fade = step; fade >= 1; fade--) {
      setRGBA(channel, rgb, (float)fade / step);
      delay(time);
    }
  }
}

bool detectEdgeTransition(bool lastState, bool currentState, int edge) {
  bool result = false;
  switch (edge) {
    case FALLING_EDGE:
      if (lastState && !currentState) {
        result = true;
      }
      break;
    case RISING_EDGE:
      if (!lastState && currentState) {
        result = true;
      }
      break;
    default:
      break;
  }
  return result;
}

void updateButtonStates(bool* currentValues, int numButtons) {
  for (int i = 0; i < numButtons; i++) {
    if (detectEdgeTransition(pbLastValue[i], currentValues[i], RISING_EDGE)) {
      pbStates[i] = !pbStates[i];
    }
    pbLastValue[i] = currentValues[i];
  }
}

void updateJoystickValues() {
  jsValues[0] = analogRead(JOYSTICK_X_PIN);
  jsValues[1] = analogRead(JOYSTICK_Y_PIN);
  jsValues[2] = analogRead(JOYSTICK_SW_PIN);

  serialPlot(jsValues[0], "x", false);
  serialPlot(jsValues[1], "y", false);
  serialPlot(jsValues[2], "sw", true);
}

void updateSwitchPosition() {
  if (digitalRead(SW_1_PIN)) {
    swPosition = 1;
  } else if (digitalRead(SW_2_PIN)) {
    swPosition = 2;
  }
}


// === SERIAL FUNCTIONS ===========================================================================

void configureSerial() {
  Serial.begin(BAUD_RATE);
}

void serialPlot(int value, char* label, bool endLine) {
  Serial.printf("%s:%d,", label, value);
  if (endLine) Serial.print("\n");
}


// === AVERAGING FUNCTIONS ========================================================================

// void updateEMA(int* currentValues, int* lastValues, int numValues, float k) {
//   // Update exponential moving average (EMA)
//   for (int i = 0; i < numValues; i++) {
//     lastValues[i] = (int)((currentValues[i] * k) + (lastValues[i] * (1 - k)));
//   }
// }


// === LOGIC FUNCTIONS ============================================================================

void skipLogic() {
  if (jsValues[jsX] < LOWER_THRESHOLD - CLEAR_THRESHOLD) {

    // Joystick LEFT: clear left LED
    setRGBA(leftLED, BLUE, 0.25);

    // Update client
    packet.leftCmd = 0;
    transmit();

  } else if (jsValues[jsX] > UPPER_THRESHOLD + CLEAR_THRESHOLD) {

    // Joystick RIGHT: clear right LED
    setRGBA(rightLED, BLUE, 0.25);

    // Update client
    packet.rightCmd = 0;
    transmit();

  } else {

    // Push Button LEFT
    if (pbStates[pbLeft]) {
      if (jsValues[1] < LOWER_THRESHOLD) {
        // Joystick DOWN: left LED (red)
        setRGBA(leftLED, RED, 0.8);
        packet.leftCmd = 1;
      } else if (jsValues[1] > UPPER_THRESHOLD) {
        // Joystick UP: left LED (green)
        setRGBA(leftLED, GREEN, 0.8);
        packet.leftCmd = 3;
      } else {
        // Joystick MID: left LED (yellow)
        setRGBA(leftLED, YELLOW, 0.8);
        packet.leftCmd = 2;
      }

      pbStates[pbLeft] = false;  // acknowledge LED set -> button state FALSE
      transmit(); // udate client
    }

    // Push Button RIGHT
    if (pbStates[pbRight]) {
      if (jsValues[1] < LOWER_THRESHOLD) {
        // Joystick DOWN: right LED (red)
        setRGBA(rightLED, RED, 0.8);
        packet.rightCmd = 1;
      } else if (jsValues[1] > UPPER_THRESHOLD) {
        // Joystick UP: right LED (green)
        setRGBA(rightLED, GREEN, 0.8);
        packet.rightCmd = 3;
      } else {
        // Joystick MID: left LED (yellow)
        setRGBA(rightLED, YELLOW, 0.8);
        packet.rightCmd = 2;
      }

      pbStates[pbRight] = false;  // acknowledge LED set -> button state FALSE
      transmit(); // udate client
    }
  }
}

void sweepLogic() {
  // TODO: any logic required by the sweep during runtime (ex. timeout)
}

void sweepProcessCommand(int command) {
  switch (command) {
    case 0:
      setStripRGBA(BLUE, 0.10);
      break;
    case 1:
      setStripRGBA(RED, 0.10);
      break;
    case 2:
      setStripRGBA(YELLOW, 0.10);
      break;
    case 3:
      setStripRGBA(GREEN, 0.10);
      break;
  }
}


// === ESP-NOW FUNCTIONS ==========================================================================

void transmit() {
  // Broadcast packet
  esp_err_t result = esp_now_send(0, (uint8_t *) &packet, sizeof(esp_now_packet));
  Serial.printf("Transmit: %s\n", result == ESP_OK ? "SUCCESS" : "FAIL");
}

// Callback function when data is recieved
void recieve(const uint8_t* macAddress, const uint8_t* data, int length) {
  char macAddressString[18];  // string for holding the sender's MAC address
  printMAC(macAddress, macAddressString);  // 'cast' MAC address to string

  memcpy(&packet, data, sizeof(packet));  // copy contents of packet into memory

  Serial.printf("Packet from '%s' recieved with length %d\n", macAddressString, length);
  Serial.printf(" left: %d\n right: %d\n", (int)packet.leftCmd, (int)packet.rightCmd);

  // Process the latest skip command
  if (swPosition == LEFT) {
    sweepProcessCommand(packet.leftCmd);
  } else if (swPosition == RIGHT) {
    sweepProcessCommand(packet.rightCmd);
  }
}

// Callback function when data is sent
void sent(const uint8_t* macAddress, esp_now_send_status_t status) {
  char macAddressString[18];  // string for holding the sender's MAC address
  printMAC(macAddress, macAddressString);  // 'cast' MAC address to string
  
  Serial.printf("Packet to '%s' send status: %s\n", macAddressString, status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void printMAC(const uint8_t* macAddress, char* macString) {
  // Copy MAC address to string
  snprintf(macString, sizeof(macString), "%02x:%02x:%02x:%02x:%02x:%02x", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
}
