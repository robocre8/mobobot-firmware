#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// CONSTRUCTOR SELECTION
// For 1.3" OLEDs, SH1106 is the standard. 
// Rotation is set to U8G2_R0 (landscape, no rotation).
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// IF your 1.3" board happens to use the SSD1306 driver, uncomment the line below and comment out the one above:
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

int counter = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing OLED...");

  // Initialize the display using ESP32's default hardware I2C (SDA=21, SCL=22)
  if (!u8g2.begin()) {
    Serial.println("OLED initialization failed!");
    while (1); // Halt if display not found
  }
  
  Serial.println("OLED Initialized Successfully.");
}

void loop() {
  // Clear the internal memory buffer
  u8g2.clearBuffer();					

  // Draw a border frame around the screen
  u8g2.drawFrame(0, 0, 128, 64);

  // Set font for the title
  u8g2.setFont(u8g2_font_ncenB08_tr);	
  u8g2.drawStr(10, 20, "ESP32 & 1.3\" OLED");	

  // // Draw a separating line
  // u8g2.drawHorizontalLine(10, 26, 108);

  // Set a different font for the counter
  u8g2.setFont(u8g2_font_7x14_tf);
  
  // Format the dynamic string
  char countString[16];
  sprintf(countString, "Count: %d", counter);
  u8g2.drawStr(10, 48, countString);	

  // Send buffer to the actual display hardware
  u8g2.sendBuffer();					

  // Increment counter and log to serial
  Serial.printf("Loop iteration: %d\n", counter);
  counter++;

  delay(1000);
}



// #include <SPI.h>
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SH110X.h>

// /* 1.3" OLED usually uses I2C Address 0x3C */
// #define i2c_Address 0x3c 

// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 64
// #define OLED_RESET -1   

// Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// void setup() {
//   Serial.begin(115200);
  
//   // Initialize the display
//   display.begin(i2c_Address, true);
//   display.display();
//   display.clearDisplay();
  
//   // Text settings
//   display.setTextSize(1);
//   display.setTextColor(SH110X_WHITE);
//   display.setCursor(0, 0);
//   display.println("Hello ESP32!");
//   display.display();
// }

// void loop() {
//   // Main loop logic
// }