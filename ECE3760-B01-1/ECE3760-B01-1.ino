// === INLCUDE STATEMENTS =========================================================================


// === DEFINE STATEMENTS ==========================================================================
#define BAUD_RATE 115200
#define SYS_DELAY 500

#define PB_1_PIN 5
#define PB_2_PIN 32

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
#define PURPLE    0xFFFF00
#define BLUE      0x00FF00
#define TURQUIOSE 0x00FFFF
#define GREEN     0x0000FF
#define YELLOW    0xFF00FF
#define WHITE     0xFFFFFF


// === GLOBAL VARIABLES ===========================================================================

int pbPins[] = {PB_1_PIN, PB_2_PIN};  // Push Button (PB) pins
int pbDirs[] = {INPUT,    INPUT};     // Push Button (PB) directions

int rgb1Pins[] =     {RGB1_R_PIN,     RGB1_G_PIN,     RGB1_B_PIN    };  // RGB LED 1 PWM pins
int rgb2Pins[] =     {RGB2_R_PIN,     RGB2_G_PIN,     RGB2_B_PIN    };  // RGB LED 2 PWM pins
int rgb1Channels[] = {RGB1_R_CHANNEL, RGB1_G_CHANNEL, RGB1_B_CHANNEL};  // RGB LED 1 PWM channels
int rgb2Channels[] = {RGB2_R_CHANNEL, RGB2_G_CHANNEL, RGB2_B_CHANNEL};  // RGB LED 2 PWM channels


// === MAIN PROGRAM ===============================================================================

void setup() {
  // Configure Serial Communication
  configureSerial();
  
  // Configure input GPIOs
  // configureGPIO(pbPins, pbDirs, 2);
  pinMode(PB_1_PIN, INPUT);
  pinMode(PB_2_PIN, INPUT);

  // Configure PWM channels
  configurePWM(rgb1Pins, rgb1Channels, 3);  // configure PWM for RGB LED 1
  configurePWM(rgb2Pins, rgb2Channels, 3);  // configure PWM for RGB LED 2

  // Configure Wifi (...)
}

void loop() {
  int colors[] = {RED, PURPLE, BLUE, TURQUIOSE, GREEN, YELLOW};
  for (int i = 0; i < (sizeof(colors) / 4); i++) {
    pulseRGB(rgb1Channels, colors[i], 1, 5, 256);
    pulseRGB(rgb2Channels, colors[i], 1, 5, 256);
  }
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

// === SERIAL FUNCTIONS ===========================================================================

void configureSerial() {
  Serial.begin(BAUD_RATE);
}

void serialPlot(int value, char* label, bool endLine) {
  Serial.printf("%s:%d,", label, value);
  if (endLine) Serial.print("\n");
}
