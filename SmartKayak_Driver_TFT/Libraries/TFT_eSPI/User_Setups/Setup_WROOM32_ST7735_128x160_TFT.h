// Setup_WROOM32_ST7735_128x160_TFT.h #################################################
//
// Section 1. Call up the right driver file and any options for it
//
// ################################################

#define ST7735_DRIVER // Define additional parameters below for this display
#define TFT_RGB_ORDER TFT_RGB // Colour order Red-Green-Blue

// ################################################
//
// Section 2. Define the pins that are used to interface with the display here
//
// ################################################

#define TFT_BL 32 // LED back-light control pin
#define TFT_BACKLIGHT_ON HIGH // Level to turn ON back-light (HIGH or LOW)

#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS 15 // Chip select control pin
#define TFT_DC 2 // Data Command control pin
#define TFT_RST 4 // Reset pin (could connect to RST pin)

// ################################################
//
// Section 3. Define the fonts that are to be used here
//
// ################################################

#define LOAD_GLCD // Font 1. Original Adafruit 8 pixel font
#define LOAD_FONT2 // Font 2. Small 16 pixel high font
#define LOAD_FONT4 // Font 4. Medium 26 pixel high font
#define LOAD_FONT6 // Font 6. Large 48 pixel font
#define LOAD_FONT7 // Font 7. 7 segment 48 pixel font
#define LOAD_FONT8 // Font 8. Large 75 pixel font
#define LOAD_GFXFF // FreeFonts.
#define SMOOTH_FONT

// ################################################
//
// Section 4. Other options
//
// ################################################

#define SPI_FREQUENCY 27000000
#define SPI_READ_FREQUENCY 20000000
#define SPI_TOUCH_FREQUENCY 2500000