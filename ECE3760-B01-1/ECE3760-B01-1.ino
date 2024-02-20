// === INLCUDE STATEMENTS =========================================================================


// === DEFINE STATEMENTS ==========================================================================
#define BAUD_RATE 115200
#define SYS_DELAY 0

#define FALLING_EDGE 1
#define RISING_EDGE  2

#define PB_1_PIN 32
#define PB_2_PIN 33

#define JOYSTICK_X_PIN 15
#define JOYSTICK_Y_PIN 4
#define JOYSTICK_SW_PIN 2

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



// === GLOBAL VARIABLES ===========================================================================

int pbPins[] =    {PB_1_PIN, PB_2_PIN};  // Push Button (PB) pins
int pbDirs[] =    {INPUT,    INPUT};     // Push Button (PB) directions

int jsValues[] = {0, 0, 0};  // Joystick (JS) last EMA values {x, y, sw}

bool pbLastValue[] = {false,    false};  // Push Button (PB) last read state
bool pbStates[] =    {false,    false};  // Push Button (PB) logical value

int rgb1Pins[] =     {RGB1_R_PIN,     RGB1_G_PIN,     RGB1_B_PIN    };  // RGB LED 1 PWM pins
int rgb2Pins[] =     {RGB2_R_PIN,     RGB2_G_PIN,     RGB2_B_PIN    };  // RGB LED 2 PWM pins
int rgb1Channels[] = {RGB1_R_CHANNEL, RGB1_G_CHANNEL, RGB1_B_CHANNEL};  // RGB LED 1 PWM channels
int rgb2Channels[] = {RGB2_R_CHANNEL, RGB2_G_CHANNEL, RGB2_B_CHANNEL};  // RGB LED 2 PWM channels


int* leftLED = rgb1Channels;
int* rightLED = rgb2Channels;



// === MAIN PROGRAM ===============================================================================

void setup() {
  // Configure Serial Communication
  configureSerial();
  
  // Configure input GPIOs
  // configureGPIO(pbPins, pbDirs, 2);
  pinMode(PB_1_PIN, INPUT);
  pinMode(PB_2_PIN, INPUT);
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(JOYSTICK_SW_PIN, INPUT);

  // Configure PWM channels
  configurePWM(rgb1Pins, rgb1Channels, 3);  // configure PWM for RGB LED 1
  configurePWM(rgb2Pins, rgb2Channels, 3);  // configure PWM for RGB LED 2

  // Configure Wifi (...)


  // Initialize LED colors
  setRGBA(rgb1Channels, BLUE, 0.25);
  setRGBA(rgb2Channels, BLUE, 0.25);
}

void loop() {
  // int colors[] = {RED, PURPLE, BLUE, TURQUIOSE, GREEN, YELLOW};
  // for (int i = 0; i < (sizeof(colors) / 4); i++) {
  //   pulseRGB(rgb1Channels, colors[i], 1, 5, 256);
  //   pulseRGB(rgb2Channels, colors[i], 1, 5, 256);
  // }

  // Update push button (PB) states
  bool currentStates[] = {analogRead(PB_1_PIN), analogRead(PB_2_PIN)};
  updateButtonStates(pbLastValue, pbStates, currentStates, 2);

  readJoystickValues();

  if (jsValues[0] < LOWER_THRESHOLD - CLEAR_THRESHOLD) {
      setRGBA(leftLED, BLUE, 0.25);
    } else if (jsValues[0] > UPPER_THRESHOLD + CLEAR_THRESHOLD) {
      setRGBA(rightLED, BLUE, 0.25);
    } else {
      // Push Button 2 (L)
      if (pbStates[1]) {
        if (jsValues[1] < LOWER_THRESHOLD) {
          setRGBA(leftLED, RED, 0.8);
        } else if (jsValues[1] > UPPER_THRESHOLD) {
          setRGBA(leftLED, GREEN, 0.8);
        } else {
          setRGBA(leftLED, YELLOW, 0.8);
        }
        pbStates[1] = false;
      }
      // Push Button 1 (R)
      if (pbStates[0]) {
        if (jsValues[1] < LOWER_THRESHOLD) {
          setRGBA(rightLED, RED, 0.8);
        } else if (jsValues[1] > UPPER_THRESHOLD) {
          setRGBA(rightLED, GREEN, 0.8);
        } else {
          setRGBA(rightLED, YELLOW, 0.8);
        }
        pbStates[0] = false;
      }
    }

  // if (pbStates[1]) {
  //   setRGBA(rgb2Channels, RED, 0.5);
  // } else {
  //   setRGBA(rgb2Channels, RED, 0.0);
  // }

  delay(SYS_DELAY);
}


// === GPIO FUNCTIONS =============================================================================

void configureGPIO(int* pins, int* directions, int count) {
  for (int i = 0; i < count; i++) {
    pinMode(pins[i], directions[i]);
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

void updateButtonStates(bool* lastValue, bool* state, bool* currentValue, int numButtons) {
  for (int i = 0; i < numButtons; i++) {
    if (detectEdgeTransition(lastValue[i], currentValue[i], RISING_EDGE)) {
      state[i] = !state[i];
    }
    lastValue[i] = currentValue[i];
  }
}

void readJoystickValues() {
  jsValues[0] = analogRead(JOYSTICK_X_PIN);
  jsValues[1] = analogRead(JOYSTICK_Y_PIN);
  jsValues[2] = analogRead(JOYSTICK_SW_PIN);

  serialPlot(jsValues[0], "x", false);
  serialPlot(jsValues[1], "y", false);
  serialPlot(jsValues[2], "sw", true);
}


// === SERIAL FUNCTIONS ===========================================================================

void configureSerial() {
  Serial.begin(BAUD_RATE);
}

void serialPlot(int value, char* label, bool endLine) {
  Serial.printf("%s:%d,", label, value);
  if (endLine) Serial.print("\n");
}


// === AVERAGING FUNCTIONS ===========================================================================

// void updateEMA(int* currentValues, int* lastValues, int numValues, float k) {
//   // Update exponential moving average (EMA)
//   for (int i = 0; i < numValues; i++) {
//     lastValues[i] = (int)((currentValues[i] * k) + (lastValues[i] * (1 - k)));
//   }
// }
