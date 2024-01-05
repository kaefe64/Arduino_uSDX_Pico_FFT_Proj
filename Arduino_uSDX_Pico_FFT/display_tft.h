#ifndef __DISPLAY_TFT_H__
#define __DISPLAY_TFT_H__

#ifdef __cplusplus
extern "C" {
#endif



#define ROTATION_SETUP  2   // 0, 1, 2 or 3


#if ROTATION_SETUP == 0 || ROTATION_SETUP == 2
//ILI9341
#define display_WIDTH  TFT_WIDTH   //TFT_HEIGHT=240
#define display_HEIGHT TFT_HEIGHT    //TFT_WIDTH=320
#endif

#if ROTATION_SETUP == 1 || ROTATION_SETUP == 3
//ILI9341
#define display_WIDTH  TFT_HEIGHT    //TFT_WIDTH=320
#define display_HEIGHT TFT_WIDTH   //TFT_HEIGHT=240
#endif

/*
#define FONT &FreeMonoBold9pt7b
#define X_CHAR  11
#define Y_CHAR  16
*/
#define FONT3 &FreeMonoBold24pt7b
//#define FONT &FreeMono24pt7b
//#define FONT &FreeSans24pt7b
//#define FONT &FreeSerif24pt7b
#define X_CHAR3  28
#define Y_CHAR3  42
#define SIZE3    1

#define FONT2 &FreeMonoBold18pt7b
//#define FONT2 &FreeMono18pt7b
#define X_CHAR2  21
#define Y_CHAR2  32
#define SIZE2    1

#define FONT1 &FreeMonoBold12pt7b
#define X_CHAR1  14
#define Y_CHAR1  22
#define SIZE1    1





// Default color definitions
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xD69A      /* 211, 211, 211 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFE19      /* 255, 192, 203 */    
#define TFT_BROWN       0x9A60      /* 150,  75,   0 */
#define TFT_GOLD        0xFEA0      /* 255, 215,   0 */
#define TFT_SILVER      0xC618      /* 192, 192, 192 */
#define TFT_SKYBLUE     0x867D      /* 135, 206, 235 */
#define TFT_VIOLET      0x915C      /* 180,  46, 226 */

/*
uint16_t red =    tft.color565(255, 0, 0);
uint16_t green =  tft.color565(0, 255, 0);
uint16_t blue =   tft.color565(0, 0, 255);
uint16_t yellow = tft.color565(255, 255, 0);
*/

// scope graph
//#define AUD_GRAPH_NUM_COLS  100
#define AUD_GRAPH_MIN       -25
#define AUD_GRAPH_MAX       25
#define X_MIN_AUD_GRAPH     215
#define Y_MIN_AUD_GRAPH     30
#define Y_MAX_AUD_GRAPH     (Y_MIN_AUD_GRAPH + AUD_GRAPH_MAX - AUD_GRAPH_MIN)



// waterfall = FFT graph
#define GRAPH_NUM_LINES   (48u)
#define GRAPH_NUM_COLS    (FFT_NSAMP)
#define Y_MIN_DRAW   (display_HEIGHT - GRAPH_NUM_LINES)


extern uint8_t vet_graf_fft[GRAPH_NUM_LINES][GRAPH_NUM_COLS];    // [NL][NCOL]
//extern uint16_t vet_graf_fft_pos;





//void tft_setup(void);
//void tft_writexy(uint8_t x, uint8_t y, uint8_t *s);
void tft_writexy_(uint16_t font, uint16_t color, uint16_t color_back, uint16_t x, uint16_t y, uint8_t *s);
void tft_writexy_plus(uint16_t font, uint16_t color, uint16_t color_back, uint16_t x, uint16_t x_plus, uint16_t y, uint16_t y_plus, uint8_t *s);
void tft_cursor(uint16_t font, uint16_t color, uint8_t x, uint8_t y);
void tft_cursor_plus(uint16_t font, uint16_t color, uint8_t x, uint8_t x_plus, uint8_t y, uint8_t y_plus);
uint16_t tft_color565(uint16_t r, uint16_t g, uint16_t b);


void display_fft_graf(void);
void display_fft_graf_top(void);
void display_tft_setup0(void);
void display_tft_setup(void);
void display_tft_countdown(bool show, uint16_t val);
void display_tft_loop(void);

void display_aud_graf(void);



#ifdef __cplusplus
}
#endif
#endif
