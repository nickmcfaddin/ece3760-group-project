// === COMPILATION "FLAGS" ==========================================================================================================================

// #define SKIP_DEVICE
#define DEBUG


// === INLCUDE STATEMENTS ===========================================================================================================================

#include "WiFi.h"
#include "esp_now.h"

#ifndef SKIP_DEVICE
#include <Adafruit_NeoPixel.h>
#endif


// === DEFINE STATEMENTS ============================================================================================================================

#define BAUD_RATE 115200
#define SYS_DELAY 100

#define LEFT  1
#define RIGHT 2

#define PB_1_PIN 36
#define PB_2_PIN 39

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

#define LED_STRIP_PIN  22
#define LED_NUM_PIXELS 8

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

#define LOWER_THRESHOLD 1200
#define UPPER_THRESHOLD 2800
#define CLEAR_THRESHOLD 900


// === STRUCTS & ENUMS ============================================================================

typedef enum state_enum {
  IDLE,
  CLEAR,
  SWEEP_LIGHT,
  SWEEP_HARD
} state_t;

typedef struct esp_now_packet_struct {
  int leftState;
  int rightState;
} esp_now_packet_t;


// === GLOBAL VARIABLES ===========================================================================

float ledBrightness = 0.25;

#ifdef SKIP_DEVICE

bool pbLeft =  false;  // value of 'left'  push button (PB) [digital]
bool pbRight = false;  // value of 'right' push button (PB) [digital]

int jsX =  0;          // value of 'x' joystick (JS) position [analog]
int jsY =  0;          // value of 'y' joystick (JS) position [analog]
int jsSW = 0;          // value of joystick (JS) 'switch'     [analog]

int rgb1Pins[] =     {RGB1_R_PIN,     RGB1_G_PIN,     RGB1_B_PIN    };  // RGB LED 1 PWM pins
int rgb2Pins[] =     {RGB2_R_PIN,     RGB2_G_PIN,     RGB2_B_PIN    };  // RGB LED 2 PWM pins
int rgb1Channels[] = {RGB1_R_CHANNEL, RGB1_G_CHANNEL, RGB1_B_CHANNEL};  // RGB LED 1 PWM channels
int rgb2Channels[] = {RGB2_R_CHANNEL, RGB2_G_CHANNEL, RGB2_B_CHANNEL};  // RGB LED 2 PWM channels

int* leftLED = rgb1Channels;   // Aliase for RGB LED left PWM channels
int* rightLED = rgb2Channels;  // Aliase for RGB LED right PWM channels

#else

bool pbPower =  false;  // value of 'power' push button (PB) [digital]
int swPosition = 0;     // value of 'position' switch (SW) [LEFT/RIGHT]

Adafruit_NeoPixel pixels(LED_NUM_PIXELS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

#endif

esp_now_packet_t packet;       // Instance of ESP-NOW packet struct for stored sent/recieved data
esp_now_peer_info_t peerInfo;  // Information about connected peers

uint8_t hostAddress[] =   {0x40, 0x22, 0xD8, 0xEA, 0x76, 0x30};  // TODO: determine dynamically
uint8_t clientAddress[] = {0x40, 0x22, 0xD8, 0xEE, 0x6D, 0xE0};  // TODO: determine dynamically


// === MAIN PROGRAM ===============================================================================

void setup() {

  #ifdef DEBUG
  // Configure Serial Communication
  Serial.begin(BAUD_RATE);
  #endif

  // Configure Wifi
  WiFi.mode(WIFI_MODE_STA);
  #ifdef DEBUG
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());  // TODO: fix bug on SWEEP devices (MAC gets printed as non-sense)
  #endif

  // Configure ESP-NOW
  if (esp_now_init() != ESP_OK) {
    #ifdef DEBUG
    Serial.println("Error initializing ESP-NOW");
    #endif
    return;
  }

#ifdef SKIP_DEVICE
  
    // Configure input GPIOs
    pinMode(PB_1_PIN, INPUT);
    pinMode(PB_2_PIN, INPUT);
    pinMode(JOYSTICK_X_PIN, INPUT);
    pinMode(JOYSTICK_Y_PIN, INPUT);
    pinMode(JOYSTICK_SW_PIN, INPUT);

    // Configure PWM channels
    configurePWM(rgb1Pins, rgb1Channels, 3);  // configure PWM for RGB LED 1
    configurePWM(rgb2Pins, rgb2Channels, 3);  // configure PWM for RGB LED 2

    // Register callback function for 'on sent' event
    esp_now_register_send_cb(onSent);

    // Register peer (TODO: setup mulitple peers)
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Register first peer (TODO: have 'pairing' process for dynamic connection)
    memcpy(peerInfo.peer_addr, clientAddress, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      #ifdef DEBUG
      Serial.println("Failed to add peer");
      #endif
      return;
    }

    delay(200);  // Extra delay to ensure ESP-NOW has enough time to configure

    // Initialize LED colors
    setPwmRGBA(rgb1Channels, BLUE, ledBrightness);
    setPwmRGBA(rgb2Channels, BLUE, ledBrightness);

#else
    
    // Configure input GPIOs
    pinMode(PB_1_PIN, INPUT);
    pinMode(SW_1_PIN, INPUT);
    pinMode(SW_2_PIN, INPUT);

    // Configure NeoPixel
    pixels.begin();

    // Register callback function for 'on recieve' event
    esp_now_register_recv_cb(onRecieve);

    // Initialize LED colors
    setStripRGBA(BLUE, ledBrightness);

#endif

}

void loop() {

#ifdef SKIP_DEVICE

    // 1. Update push button (PB) values
    pbLeft = digitalRead(PB_1_PIN);
    pbRight = digitalRead(PB_2_PIN);
    
    // 2. Update joystick (JS) values
    jsX = analogRead(JOYSTICK_X_PIN);
    jsY = analogRead(JOYSTICK_Y_PIN);
    jsSW = analogRead(JOYSTICK_SW_PIN);

    // 3. Perform Skip Logic
    skipLogic();

#else

    // 1. Update push button (PB) values
    pbPower = digitalRead(PB_1_PIN);

    // 2. Update switch (SW) value
    swPosition = digitalRead(SW_1_PIN) ? LEFT : RIGHT;

    // 3. Perform Sweep Logic
    sweepLogic();

#endif
  
  delay(SYS_DELAY);  // System delay on every iteration
}


// === GPIO FUNCTIONS ===============================================================================================================================

void configurePWM(int* pin, int* channel, int count) {
  for (int i = 0; i < count; i++) {
    ledcSetup(channel[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(pin[i], channel[i]);
  }
}

// Set the Red (R) Blue (B) Green (G) and Alpha (A) values for a PWM controlled RGB LED
void setPwmRGBA(int* channels, int rgb, float alpha) {
  ledcWrite(channels[0], (int)(((rgb >> 16) & 0xFF) * alpha));
  ledcWrite(channels[1], (int)(((rgb >> 8 ) & 0xFF) * alpha));
  ledcWrite(channels[2], (int)(((rgb >> 0 ) & 0xFF) * alpha));
}

void setStripRGBA(int rgb, float alpha) {
#ifndef SKIP_DEVICE
  pixels.clear();
  for (int i = 0; i < LED_NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(
      (int)(((rgb >> 16) & 0xFF) * alpha),
      (int)(((rgb >> 8 ) & 0xFF) * alpha),
      (int)(((rgb >> 0 ) & 0xFF) * alpha)
    ));
  }
  pixels.show();
#endif
}


// === LOGIC FUNCTIONS ==============================================================================================================================

#ifdef SKIP_DEVICE

void skipLogic() {

  // 1. Check for LEFT joystick position
  if (jsX < LOWER_THRESHOLD - CLEAR_THRESHOLD) {
    setPwmRGBA(leftLED, BLUE, ledBrightness);       // set left LED 'blue'
    packet.leftState = IDLE;                        // update left state

  // 2. Check for RIGHT joystick position
  } else if (jsX > UPPER_THRESHOLD + CLEAR_THRESHOLD) {
    setPwmRGBA(rightLED, BLUE, ledBrightness);      // set left LED 'blue'
    packet.rightState = IDLE;                       // update right state

  } else {

    // 3. Check for LEFT button press
    if (pbLeft) {
      if (jsY < LOWER_THRESHOLD) {
        setPwmRGBA(leftLED, RED, ledBrightness);     // set left LED 'red'
        packet.leftState = SWEEP_LIGHT;              // update left state
      } else if (jsY > UPPER_THRESHOLD) {
        setPwmRGBA(leftLED, GREEN, ledBrightness);   // set left LED 'green'
        packet.leftState = SWEEP_HARD;               // update left state
      } else {
        setPwmRGBA(leftLED, BLUE, ledBrightness);    // set left LED 'blue'
        packet.leftState = CLEAR;                    // update left state
      }
    }

    // 4. Check for RIGHT button press
    if (pbRight) {
      if (jsY < LOWER_THRESHOLD) {
        setPwmRGBA(rightLED, RED, ledBrightness);    // set right LED 'red'
        packet.rightState = SWEEP_LIGHT;             // update right state
      } else if (jsY > UPPER_THRESHOLD) {
        setPwmRGBA(rightLED, GREEN, ledBrightness);  // set right LED 'green'
        packet.rightState = SWEEP_HARD;              // update right state
      } else {
        setPwmRGBA(rightLED, BLUE, ledBrightness);   // set right LED 'blue'
        packet.rightState = CLEAR;                   // update right state
      }
    }
  }

  transmit();  // update client
}

#else

void sweepLogic() {

  // 1. Determine which 'side' to listen
  int currentState = (swPosition == LEFT) ? packet.leftState : packet.rightState;

  // 2. Determine which 'state' to set the LEDs
  switch (currentState) {
    case IDLE:
      setStripRGBA(BLUE, ledBrightness);   // set LEDs 'blue'
      break;
    case CLEAR:
      setStripRGBA(BLUE, ledBrightness);   // set LEDs 'blue'
      break;
    case SWEEP_LIGHT:
      setStripRGBA(RED, ledBrightness);    // set LEDs 'red'
      break;
    case SWEEP_HARD:
      setStripRGBA(GREEN, ledBrightness);  // set LEDs 'green'
      break;
  }
}

#endif


// === ESP-NOW FUNCTIONS ==========================================================================

void transmit() {
  esp_err_t result = esp_now_send(0, (uint8_t *) &packet, sizeof(esp_now_packet_t));
  #ifdef DEBUG
  Serial.printf("Transmit: %s\n", result == ESP_OK ? "SUCCESS" : "FAIL");
  #endif
}

void onRecieve(const uint8_t* macAddress, const uint8_t* data, int length) {
  #ifdef DEBUG
  char macAddressString[18];               // string for holding the sender's MAC address
  printMAC(macAddress, macAddressString);  // 'cast' MAC address to string
  Serial.printf("Packet from '%s' recieved with length %d\n", macAddressString, length);
  Serial.printf(" left: %d\n right: %d\n", (int)packet.leftState, (int)packet.rightState);
  #endif

  memcpy(&packet, data, sizeof(packet));  // copy contents of packet into memory
}

void onSent(const uint8_t* macAddress, esp_now_send_status_t status) {
  #ifdef DEBUG
  char macAddressString[18];               // string for holding the sender's MAC address
  printMAC(macAddress, macAddressString);  // 'cast' MAC address to string
  Serial.printf("Packet to '%s' send status: %s\n", macAddressString, status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
  #endif
}

// Copy MAC address to string
void printMAC(const uint8_t* macAddress, char* macString) {
  snprintf(macString, sizeof(macString), "%02x:%02x:%02x:%02x:%02x:%02x", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
}
