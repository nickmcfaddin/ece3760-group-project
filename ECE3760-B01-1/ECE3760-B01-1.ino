// === COMPILATION "FLAGS" ==========================================================================================================================

#define SKIP_DEVICE
#define DEBUG


// === INLCUDE STATEMENTS ===========================================================================================================================

// WiFi and ESP-NOW
#include "WiFi.h"
#include "esp_now.h"
#include "esp_wifi.h"
// Adafruit NeoPixel LEDs
#include <Adafruit_NeoPixel.h>


// === DEFINE STATEMENTS ============================================================================================================================

#define BAUD_RATE 115200
#define SYS_DELAY 100

// TODO: This could be an ENUM
#define LEFT  1
#define RIGHT 2

#define PB_PAIRING_PIN  14

#define SW_1_PIN        25
#define SW_2_PIN        26

#define JOYSTICK_X_PIN  33
#define JOYSTICK_Y_PIN  32
#define JOYSTICK_SW_PIN 27

#define LED_LEFT_PIN    25
#define LED_RIGHT_PIN   26
#define LED_STRIP_PIN   32

#define LED_STRIP_NUM_PIXELS 8

#define TEAM_ID 123          // Unique identifier for WiFi-based pairing
#define WIFI_CHANNEL 0       // Wifi channel used for ESP-NOW
#define DEFAULT_ID   1       // Default ID for unpaired sweeper devices
#define MAX_PEERS    3       // Maximum number of allowed peer devices
#define MAC_BYTES    6       // Number of bytes in a MAC address

// Pre-defined Colors
#define RED       0xFF0000
#define YELLOW    0xFFBB00
#define GREEN     0x00FF00
#define TURQUIOSE 0x00FFFF
#define BLUE      0x0000FF
#define PURPLE    0xFF00FF
#define WHITE     0xFFFFFF
#define OFF       0x000000

#define JS_CENTER 2000
#define THRESHOLD 800


// === STRUCTS & ENUMS ============================================================================

typedef enum state_enum {
  STOP,
  LIGHT,
  HARD
} state_t;

typedef enum quadrant_enum {
  BOTTOM_LEFT,
  BOTTOM_CENTER,
  BOTTOM_RIGHT,
  MIDDLE_LEFT,
  MIDDLE_CENTER,
  MIDDLE_RIGHT,
  TOP_LEFT,
  TOP_CENTER,
  TOP_RIGHT
} quadrant_t;

typedef enum message_type_enum {
  PAIRING,
  DATA
} message_t;

typedef struct esp_now_message_packet_struct {
  uint8_t team;
  uint8_t type;
  uint8_t id;
  uint8_t leftState;
  uint8_t rightState;
} data_packet_t;

typedef struct esp_now_paring_packet_struct {
  uint8_t team;
  uint8_t type;
  uint8_t id;
  uint8_t macAddr[MAC_BYTES];
  uint8_t channel;
} pairing_packet_t;


// === GLOBAL VARIABLES ===========================================================================

// volatile variables (shared between the 2 processor threads)
volatile bool enable = true;
volatile bool paired = false;

// Configurable parameters
float ledBrightness =    0.100;    // [%] must be between 0.0 - 1.0
float debouncePeroid =   0.050;    // [s] decimals after the millisecond position will be ignored
float longPressTimeout = 2.000;    // [s] decimals after the millisecond position will be ignored
float doublePressTimeout = 0.750;  // [s] decimals after the millisecond position will be ignored

bool pbPairingValue = false;        // value of 'pairing' push button (PB) [digital]
bool pbPairingState = false;        // value of 'pairing' push button (PB) after debounce
bool pbPairingDoublePress = false;  // logical value of 'pairing' push button (PB) double press state
bool pbPairingLongPress = false;    // logical value of 'pairing' push button (PB) long press state
int pairingLastDebounce = 0;        // Last time of debounce occurance for pairing push button (PB)

int doublePressPeriod = 0;
int longPressPeriod = 0;

#ifdef SKIP_DEVICE

#ifdef SKIP_DEVICE

bool pbLeft =  false;  // value of 'left'  push button (PB) [digital]
bool pbRight = false;  // value of 'right' push button (PB) [digital]

int jsX =  0;          // value of 'x' joystick (JS) position [analog]
int jsY =  0;          // value of 'y' joystick (JS) position [analog]
int jsSW = 0;          // value of joystick (JS) 'switch'     [analog]

int prev_jsX = 0;
int prev_jsY = 0;
int lastChangedTime = 0;
int sleepTime = 10000;
int posBuffer = 100;

Adafruit_NeoPixel pixelLeft(1, LED_LEFT_PIN, NEO_GRB + NEO_KHZ800), pixelRight(1, LED_RIGHT_PIN, NEO_GRB + NEO_KHZ800);

#else

int swPosition = 0;     // value of 'position' switch (SW) [LEFT/RIGHT]

Adafruit_NeoPixel pixels(LED_STRIP_NUM_PIXELS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

#endif

data_packet_t    dataPacket;     // ESP-NOW packet for sent/recieved data messages
pairing_packet_t pairingPacket;  // ESP-NOW packet for device pairing messages
esp_now_peer_info_t peerInfo;    // Information about paired/connected peers

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t serverMAC[] =    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t boardMAC[] =     {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t peerMACs[MAX_PEERS][MAC_BYTES] = {};
int numPeers = 0;


// === MAIN PROGRAM ===============================================================================

void setup() {

  // Configure ULP wakeup condition
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14, 0);

  // Configure Serial Communication
  Serial.begin(BAUD_RATE);

  // Configure Wifi
  WiFi.mode(WIFI_MODE_STA);
  #ifdef DEBUG
  WiFi.macAddress(boardMAC);
  Serial.print("Mac Address: ");
  printMAC(boardMAC);
  Serial.println();
  #endif

  // Configure ESP-NOW
  if (esp_now_init() != ESP_OK) {
    #ifdef DEBUG
    Serial.println("Error initializing ESP-NOW");
    #endif
  }

  // Register callback functions for ESP-NOW
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecieve); 

  // Configure packet structs
  dataPacket.team = TEAM_ID;
  dataPacket.type = DATA;
  pairingPacket.team = TEAM_ID;
  pairingPacket.type = PAIRING;

  // Configure sleep timer (NOTE: time measured in us not ms)
  if (esp_sleep_enable_timer_wakeup(SYS_DELAY * 1000) != ESP_OK) {
    #ifdef DEBUG
    Serial.println("Error enabling wakeup timer");
    #endif
  }

  print_wakeup_reason();

  #ifdef SKIP_DEVICE
  
  // Configure input GPIOs
  pinMode(PB_PAIRING_PIN, INPUT);
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(JOYSTICK_SW_PIN, INPUT);  // TODO: sort out how to read this value

  // Configure NeoPixel
  pixelLeft.begin();
  pixelRight.begin();

  // Initialize LED colors
  setSingleRGBA(LEFT, RED, ledBrightness);
  setSingleRGBA(RIGHT, RED, ledBrightness);

  #else

  // Add 'broadcast' peer
  addPeer(broadcastMAC);
    
  // Configure input GPIOs
  pinMode(PB_PAIRING_PIN, INPUT);
  pinMode(SW_1_PIN, INPUT);
  pinMode(SW_2_PIN, INPUT);

  // Configure NeoPixel
  pixels.begin();

  // Initialize LED colors
  setStripRGBA(RED, ledBrightness);

  #endif
}

void loop() {
  if (enable) {

    #ifdef SKIP_DEVICE

    // 1. Update push button (PB) values
    bool pbPairingValueNext = digitalRead(PB_PAIRING_PIN);
    bool pbPairingStateNext = debounce(pbPairingValue, pbPairingValueNext, pbPairingState, &pairingLastDebounce);

    pbPairingDoublePress = doublePress(pbPairingState, pbPairingStateNext);
    pbPairingLongPress = longPress(pbPairingState, pbPairingStateNext);

    #ifdef DEBUG
    Serial.printf("pbPairingDoublePress: %d\n", pbPairingDoublePress);
    Serial.printf("pbPairingLongPress:   %d\n", pbPairingLongPress);
    Serial.printf("doublePressPeriod:   %d\n", doublePressPeriod);
    Serial.printf("longPressPeriod:   %d\n", longPressPeriod);
    #endif

    pbPairingValue = pbPairingValueNext;
    pbPairingState = pbPairingStateNext;
    
    // 2. Update joystick (JS) values
    jsX = analogRead(JOYSTICK_X_PIN);
    jsY = analogRead(JOYSTICK_Y_PIN);

    // 3. Determine if inactive & need to enter deep sleep
    if (prev_jsX >= jsX + posBuffer || prev_jsX <= jsX - posBuffer || prev_jsY >= jsY + posBuffer || prev_jsY <= jsY - posBuffer) {
      lastChangedTime = millis();
    }
    
    if (millis() - lastChangedTime > sleepTime){
      goToSleep();
    }

    prev_jsX = jsX;
    prev_jsY = jsY;

    // jsSW = analogRead(JOYSTICK_SW_PIN);  // TODO: not enough range to read digitally, switch to analog pin

    #ifdef DEBUG
    serialPlot(jsX, "X", false);
    serialPlot(jsY, "Y", true);
    #endif

    // 4. Perform Skip Logic
    skipLogic();

    #else

    // 1. Update push button (PB) values
    bool pbPairingValueNext = digitalRead(PB_PAIRING_PIN);
    bool pbPairingStateNext = debounce(pbPairingValue, pbPairingValueNext, pbPairingState, &pairingLastDebounce);
    
    pbPairingDoublePress = doublePress(pbPairingState, pbPairingStateNext);
    pbPairingLongPress = longPress(pbPairingState, pbPairingStateNext);

    #ifdef DEBUG
    Serial.printf("pbPairingDoublePress: %d\n", pbPairingDoublePress);
    Serial.printf("pbPairingLongPress:   %d\n", pbPairingLongPress);
    Serial.printf("doublePressPeriod:   %d\n", doublePressPeriod);
    Serial.printf("longPressPeriod:   %d\n", longPressPeriod);
    #endif

    pbPairingValue = pbPairingValueNext;
    pbPairingState = pbPairingStateNext;

    // 2. Update switch (SW) value
    swPosition = digitalRead(SW_1_PIN) ? LEFT : RIGHT;  // ASSUMPTION: anything not 'left' is 'RIGHT'

    // 3. Perform Sweep Logic
    sweepLogic();

    #endif
  }
}


// === GPIO FUNCTIONS ===============================================================================================================================

void setStripRGBA(int rgb, float alpha) {
#ifndef SKIP_DEVICE
  pixels.clear();
  for (int i = 0; i < LED_STRIP_NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(
      (int)(((rgb >> 16) & 0xFF) * alpha),
      (int)(((rgb >> 8 ) & 0xFF) * alpha),
      (int)(((rgb >> 0 ) & 0xFF) * alpha)
    ));
  }
  pixels.show();
#endif
}

void setSingleRGBA(int position, int rgb, float alpha) {
#ifdef SKIP_DEVICE
  if (position == LEFT) {
    pixelLeft.clear();
    pixelLeft.setPixelColor(0, pixelLeft.Color(
      (int)(((rgb >> 16) & 0xFF) * alpha),
      (int)(((rgb >> 8 ) & 0xFF) * alpha),
      (int)(((rgb >> 0 ) & 0xFF) * alpha)
    ));
    pixelLeft.show();
  }
  else if (position == RIGHT) {
    pixelRight.clear();
    pixelRight.setPixelColor(0, pixelRight.Color(
      (int)(((rgb >> 16) & 0xFF) * alpha),
      (int)(((rgb >> 8 ) & 0xFF) * alpha),
      (int)(((rgb >> 0 ) & 0xFF) * alpha)
    ));
    pixelRight.show();
  }
#endif
}

bool debounce(bool last, bool current, bool state, int* lastDebounce) {

  // Check for 'noise'
  if (current != last) {
    // Reset debounce timer
    *lastDebounce = millis();
  }

  // Check for debounce timeout
  if ((millis() - *lastDebounce) > (int)(debouncePeroid * 1000)) {
    if (state != current)
    return current;
  } else {
    return state;
  }
}

bool doublePress(bool previous, bool current) {
  // Rising Edge
  if (current && !previous) {
    // Check for double press timeout
    if ((millis() - doublePressPeriod) < (int)(doublePressTimeout * 1000)) {
      return true;
    }
  }
  // Falling Edge
  else if (!current && previous) {
    doublePressPeriod = millis();
  }
  return false;
}

bool longPress(bool previous, bool current) {
  // Rising Edge
  if (current && !previous) {
    longPressPeriod = millis();
  }
  // Falling Edge
  else if (!current && previous) {
    // Check for long press timeout
    if ((millis() - longPressPeriod) > (int)(longPressTimeout * 1000)) {
      return true;
    }
  }
  return false;
}

int parseQuadrant(int xPos, int yPos) {
  int quadrant = -1;

  // Determine row (yPos)
  if (yPos > (JS_CENTER + THRESHOLD)) {
    // Top Row
    quadrant = 6;
  }  
  else if (yPos < (JS_CENTER - THRESHOLD)) {
    // Bottom Row
    quadrant = 0;
  }
  else {
    // Middle Row
    quadrant = 3;
  }

  // Determine column (xPos)
  if (xPos > (JS_CENTER + THRESHOLD)) {
    // Left Column
    quadrant += 2;
  }
  else if (xPos < (JS_CENTER - THRESHOLD)) {
    // Right Column
    quadrant += 0;
  }
  else {
    // Center Column
    quadrant += 1;
  }

  return quadrant;
}

void serialPlot(int value, char* label, bool end) {
  Serial.printf("%s:%d", label, value);
  Serial.print(",");
  if (end) {
    Serial.print("\n");
  }
}

void print_wakeup_reason(){
  #ifdef DEBUG
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
  #endif
}


// === LOGIC FUNCTIONS ==============================================================================================================================

#ifdef SKIP_DEVICE

void skipLogic() {

  // Check if there are any connected peers
  if (numPeers > 0) {

    // 1. Determine joystick quadrant
    int quadrant = parseQuadrant(jsX, jsY);

    #ifdef DEBUG
    Serial.printf("Joystick Position: %d\n", quadrant);
    #endif

    // 2. Execute joystick command
    switch (quadrant) {
      case BOTTOM_LEFT:
        setSingleRGBA(LEFT, BLUE, ledBrightness);    // set left LED 'blue'
        setSingleRGBA(RIGHT, RED, ledBrightness);    // set right LED 'red'
        dataPacket.leftState = LIGHT;                // update left state
        dataPacket.rightState = STOP;                // update right state
        break;
      case BOTTOM_CENTER:
        setSingleRGBA(LEFT, BLUE, ledBrightness);    // set left LED 'blue'
        setSingleRGBA(RIGHT, BLUE, ledBrightness);   // set right LED 'blue'
        dataPacket.leftState = LIGHT;                // update left state
        dataPacket.rightState = LIGHT;               // update right state
        break;
      case BOTTOM_RIGHT:
        setSingleRGBA(LEFT, RED, ledBrightness);     // set left LED 'red'
        setSingleRGBA(RIGHT, BLUE, ledBrightness);   // set right LED 'blue'
        dataPacket.leftState = STOP;                 // update left state
        dataPacket.rightState = LIGHT;               // update right state
        break;
      case MIDDLE_LEFT:
        setSingleRGBA(LEFT, RED, ledBrightness);     // set left LED 'red'
        setSingleRGBA(RIGHT, RED, ledBrightness);    // set right LED 'red'
        dataPacket.leftState = STOP;                 // update left state
        dataPacket.rightState = STOP;                // update right state
        break;
      case MIDDLE_CENTER:
        setSingleRGBA(LEFT, RED, ledBrightness);     // set left LED 'red'
        setSingleRGBA(RIGHT, RED, ledBrightness);    // set right LED 'red'
        dataPacket.leftState = STOP;                 // update left state
        dataPacket.rightState = STOP;                // update right state
        break;
      case MIDDLE_RIGHT:
        setSingleRGBA(LEFT, RED, ledBrightness);     // set left LED 'red'
        setSingleRGBA(RIGHT, RED, ledBrightness);    // set right LED 'red'
        dataPacket.leftState = STOP;                 // update left state
        dataPacket.rightState = STOP;                // update right state
        break;
      case TOP_LEFT:
        setSingleRGBA(LEFT, GREEN, ledBrightness);   // set left LED 'green'
        setSingleRGBA(RIGHT, RED, ledBrightness);    // set right LED 'red'
        dataPacket.leftState = HARD;                 // update left state
        dataPacket.rightState = STOP;                // update right state
        break;
      case TOP_CENTER:
        setSingleRGBA(LEFT, GREEN, ledBrightness);   // set left LED 'green'
        setSingleRGBA(RIGHT, GREEN, ledBrightness);  // set right LED 'green'
        dataPacket.leftState = HARD;                 // update left state
        dataPacket.rightState = HARD;                // update right state
        break;
      case TOP_RIGHT:
        setSingleRGBA(LEFT, RED, ledBrightness);     // set left LED 'red'
        setSingleRGBA(RIGHT, GREEN, ledBrightness);  // set right LED 'green'
        dataPacket.leftState = STOP;                 // update left state
        dataPacket.rightState = HARD;                // update right state
        break;
    }

    // 3. Check for pairing push button (PB)
    if (pbPairingDoublePress) {
      // Set both LEDs to 'white'
      setSingleRGBA(LEFT, WHITE, ledBrightness);
      setSingleRGBA(RIGHT, WHITE, ledBrightness);

      // Enter pairing mode
      for (int i = 0; i < numPeers; i++) {
        // Remove registered peers
        if (esp_now_del_peer(peerMACs[i]) != ESP_OK) {
          #ifdef DEBUG
          Serial.println("Error removing ESP-NOW peer");
          #endif
        }
      }
      numPeers = 0;   // reset number of peers

      return;
    }

    enable = false;  // disable main thread sampling until ACK is recieved
    transmit();      // update client
  }

  else {
    #ifdef DEBUG
    Serial.println("Waiting for peers to connect");
    #endif

    setSingleRGBA(LEFT, WHITE, ledBrightness);
    setSingleRGBA(RIGHT, WHITE, ledBrightness);

    // Delay to reduce the workload/power consumption
    delay(SYS_DELAY);  // TODO: replace with timeout timer
  }

  // Check for power push button (PB)
  if (pbPairingLongPress) {
    goToSleep();
  }
}

#else

void sweepLogic() {

  // Check if device has been paired
  if (paired) {

    // 1. Determine which 'side' to listen
    int currentState = (swPosition == LEFT) ? dataPacket.leftState : dataPacket.rightState;

    // 2. Determine which 'state' to set the LEDs
    switch (currentState) {
      case STOP:
        setStripRGBA(RED, ledBrightness);    // set LEDs 'red'
        break;
      case LIGHT:
        setStripRGBA(BLUE, ledBrightness);   // set LEDs 'blue'
        break;
      case HARD:
        setStripRGBA(GREEN, ledBrightness);  // set LEDs 'green'
        break;
    }

    // 3. Check for pairing button pushed
    if (pbPairingDoublePress) {
      // Clear last recieved state
      dataPacket.leftState = STOP;
      dataPacket.rightState = STOP;
      // Enter 'pairing' mode
      paired = false;
    }
  }

  else {
    // Set LEDs 'white'
    setStripRGBA(WHITE, ledBrightness);

    // Reset 'server' MAC address to be the broadcast address
    memcpy(&serverMAC, broadcastMAC, sizeof(uint8_t[MAC_BYTES]));

    // Set up pairing packet structure
    pairingPacket.team = TEAM_ID;
    pairingPacket.type = PAIRING;
    pairingPacket.id = DEFAULT_ID;
    pairingPacket.channel = WIFI_CHANNEL;
    memcpy(pairingPacket.macAddr, boardMAC, sizeof(uint8_t[MAC_BYTES]));

    // Attempt to pair with skip device
    esp_err_t result = esp_now_send(serverMAC, (uint8_t *) &pairingPacket, sizeof(pairing_packet_t));

    #ifdef DEBUG
    printMAC(serverMAC);
    Serial.println(" (transmit)");
    Serial.printf(" status: %s\n", result == ESP_OK ? "SUCCESS" : "FAIL");
    #endif
  }

  // Delay to reduce the workload/power consumption
  delay(SYS_DELAY);  // TODO: replace with timeout timer

  // Check for power push button (PB)
  if (pbPairingLongPress) {
    goToSleep();
  }
}

#endif


// === ESP-NOW FUNCTIONS ==========================================================================

void transmit() {
  esp_err_t result = esp_now_send(0, (uint8_t *) &dataPacket, sizeof(data_packet_t));

  #ifdef DEBUG
  Serial.printf("Data Transmit: %s\n", result == ESP_OK ? "SUCCESS" : "FAIL");
  #endif
}

void onRecieve(const uint8_t* macAddress, const uint8_t* data, int length) {
  uint8_t team = data[0];  // first byte in message is the 'team'

  if (team == TEAM_ID) {
    uint8_t type = data[1];  // second byte in message is the 'type'

    #ifdef DEBUG
    printMAC(macAddress);
    Serial.println(" (recieved)");
    Serial.printf(" length: %d\n type: %s\n", length, type == PAIRING ? "PAIRING" : "DATA");
    #endif

    // Process packet based on type message type
    switch (type) {

    // Message is of type: PAIRING
    case PAIRING:
      // Copy packet information to memory
      memcpy(&pairingPacket, data, sizeof(pairingPacket));

      #ifdef DEBUG
      Serial.printf(" id: %d\n channel: %d\n MAC: ", pairingPacket.id, pairingPacket.channel);
      printMAC(pairingPacket.macAddr);
      Serial.println();
      #endif

      #ifdef SKIP_DEVICE

        // Do not reply to message with the SKIP ID
        if (pairingPacket.id > 0) {

          // Configure pairingPacket structure
          pairingPacket.type = PAIRING;
          pairingPacket.id = 0;  // Skip ID: 0
          pairingPacket.channel = WIFI_CHANNEL;
          memcpy(pairingPacket.macAddr, boardMAC, sizeof(uint8_t[MAC_BYTES]));

          // Add new peer
          if (addPeer(macAddress)) {
            // Respond to pairing device
            esp_err_t result = esp_now_send(macAddress, (uint8_t *) &pairingPacket, sizeof(pairingPacket));

            #ifdef DEBUG
            printMAC(macAddress);
            Serial.println(" (transmit)");
            Serial.printf(" status: %s\n", result == ESP_OK ? "SUCCESS" : "FAIL");
            #endif
          }
        }

        #else

        if (pairingPacket.id == 0) {
          // Set the skip MAC address
          memcpy(serverMAC, pairingPacket.macAddr, sizeof(uint8_t[MAC_BYTES]));
          
          paired = true;
        }
        
        #endif

        break;

      // Message is a type: DATA
      case DATA:
        // Copy packet information to memory
        memcpy(&dataPacket, data, sizeof(dataPacket));

        #ifdef DEBUG
        Serial.printf(" id: %d\n left: %d\n right: %d\n", dataPacket.id, dataPacket.leftState, dataPacket.rightState);
        #endif

        break;
    }
  }

  else {
    #ifdef DEBUG
    Serial.printf("Ignoring message with team ID: %d\n", team);
    #endif
  }
}

void onSent(const uint8_t* macAddress, esp_now_send_status_t status) {
  #ifdef DEBUG
  printMAC(macAddress);
  Serial.println(" (sent)");
  Serial.printf(" status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
  #endif

  #ifdef SKIP_DEVICE

  lightSleep();   // put processor into light sleep

  enable = true;  // enable to resume main thread sampling

  #endif
}

void printMAC(const uint8_t * macAddress){
  #ifdef DEBUG
  char macString[18];
  snprintf(macString, sizeof(macString), "%02x:%02x:%02x:%02x:%02x:%02x", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
  Serial.print(macString);
  #endif
}

bool addPeer(const uint8_t *peerAddress) {
  if (numPeers < MAX_PEERS) {
    // Clear peerInfo struct
    memset(&peerInfo, 0, sizeof(peerInfo));
    // Setup new peerInfo struct
    const esp_now_peer_info_t *peer = &peerInfo;
    memcpy(peerInfo.peer_addr, peerAddress, sizeof(uint8_t[MAC_BYTES]));
    
    peerInfo.channel = WIFI_CHANNEL;  // set channel
    peerInfo.encrypt = false;         // set encryption

    // Check if the peer already exists
    if (esp_now_is_peer_exist(peerInfo.peer_addr)) {
      #ifdef DEBUG
      Serial.println("Add Peer: ALREADY ADDED");
      #endif
      return true;
    }

    // Add the new peer to configuration
    else {
      if (esp_now_add_peer(peer) == ESP_OK) {
        #ifdef DEBUG
        Serial.println("Add Peer: SUCCESS");
        #endif

        // If first connected peer -> toggle LEDs
        if (numPeers == 0) {
          // Clear last recieved state
          dataPacket.leftState = STOP;
          dataPacket.rightState = STOP;
          #ifdef SKIP_DEVICE
          setSingleRGBA(LEFT,  RED, ledBrightness);
          setSingleRGBA(RIGHT, RED, ledBrightness);
          #endif
        }

        memcpy(peerMACs[numPeers], peerAddress, sizeof(uint8_t[MAC_BYTES]));
        numPeers++;
        return true;
      }
      else {
        #ifdef DEBUG
        Serial.println("Add Peer: FAILURE");
        #endif
        return false;
      }
    }
  }

  // Maximum number of peers reached
  else {
    #ifdef DEBUG
    Serial.println("Add Peer: MAX REACHED");
    #endif
    return false;
  }
}


// === PROCESSOR FUNCTIONS ========================================================================

void lightSleep() {
  // Turn off Wifi before entering 'light' sleep
  if (esp_wifi_stop() != ESP_OK) {
    #ifdef DEBUG
    Serial.println("Error stopping Wifi");
    #endif
  }

  // Enter 'light' sleep for SYS_DELAY duration (ms)
  if (esp_light_sleep_start() != ESP_OK){
    #ifdef DEBUG
    Serial.println("Error starting light sleep");
    #endif 
  }

  // Restart Wifi after leaving 'light' sleep
  if (esp_wifi_start() != ESP_OK){
    #ifdef DEBUG
    Serial.println("Error starting Wifi");
    #endif
  }

  // Double check ESP-NOW is configured (repeated for redundency)
  if (esp_now_init() != ESP_OK) {
    #ifdef DEBUG
    Serial.println("Error initializing ESP-NOW");
    #endif
  }
}

void goToSleep() {
  #ifdef SKIP_DEVICE
  // turn off LEDS
  setSingleRGBA(LEFT, OFF, ledBrightness);
  setSingleRGBA(RIGHT, OFF, ledBrightness);
  #else
  // Initialize LED colors
  setStripRGBA(OFF, ledBrightness);
  #endif


  // Disable wakeup source (ie. the timer used for light sleep)
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  // Enable GPIO wakeup source
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14, 1);

  #ifdef DEBUG
  Serial.println("Going to sleep now");
  #endif

  esp_deep_sleep_start();
}
