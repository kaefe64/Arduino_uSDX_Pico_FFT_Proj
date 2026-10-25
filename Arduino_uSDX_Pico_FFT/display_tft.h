#ifndef __DISPLAY_TFT_H__
#define __DISPLAY_TFT_H__

#ifdef __cplusplus
extern "C" {
#endif



#define ROTATION_SETUP  1   // 0, 1, 2 or 3


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
#if 0
#define TFT_BLACK       0x0000u      /*   0,   0,   0 */
#define TFT_NAVY        0x000Fu      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0u      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EFu      /*   0, 128, 128 */
#define TFT_MAROON      0x7800u      /* 128,   0,   0 */
#define TFT_PURPLE      0x780Fu      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0u      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xD69Au      /* 211, 211, 211 */
#define TFT_DARKGREY    0x7BEFu      /* 128, 128, 128 */
#define TFT_BLUE        0x001Fu      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0u      /*   0, 255,   0 */
#define TFT_CYAN        0x07FFu      /*   0, 255, 255 */
#define TFT_RED         0xF800u      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81Fu      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0u      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFFu      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0u      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0u      /* 180, 255,   0 */
#define TFT_PINK        0xFE19u      /* 255, 192, 203 */    
#define TFT_BROWN       0x9A60u      /* 150,  75,   0 */
#define TFT_GOLD        0xFEA0u      /* 255, 215,   0 */
#define TFT_SILVER      0xC618u      /* 192, 192, 192 */
#define TFT_SKYBLUE     0x867Du      /* 135, 206, 235 */
#define TFT_VIOLET      0x915Cu      /* 180,  46, 226 */
#endif

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

#define WATERFALL_IN_BLOCK   1   // all lines in the waterfall move with the freq change (not only the new line)

/* waterfall colors
 *
 * vet_graf_fft[] holds the log level of each bin (see mag_level() in dsp.cpp), one level
 * step is 1.5 dB. The color is not taken from the level itself but from the difference
 * between the level and the noise floor of the band, so the meaning of a given color
 * does not change when the AGC, fft_gain or propagation make the whole band weaker or
 * stronger.
 *
 * Why not from the level: an absolute scale saturates as soon as the band gets strong.
 * Why not from a per column floor: a carrier that stays on can not be told apart from a
 * band that got stronger, and any floor that follows a single column ends up swallowing
 * the carrier. The floor is therefore taken over the whole band.
 * Why not from the average or the minimum of the plotted points: a strong signal entering
 * the band would move them. The median does not, because a signal is narrow, it only moves
 * a handful of the 320 columns.
 *
 * WF_ABOVE_FLOOR biases the reference below the measured floor, raise it to hide more
 * of the noise, lower it to show weaker signals.
 * WF_FLOOR_RISE_DIV is how many waterfall lines the floor takes to step up once, so the
 * band re centers gradually instead of jumping when a quiet bin happens to appear.
 */
#define WF_COLOR_NUM         (64u)      //palette entries, also the max color index stored in vet_graf_fft
#define WF_ABOVE_FLOOR       (0u)       //reference this many steps below the measured floor (any value above floor will be white)
#define WF_WINDOW            (34u)      //color window above the reference, in steps (~51 dB)
#define WF_FLOOR_RISE_DIV    (8u)       //lines the floor takes to step up once

//set to 1 to print on serial, every 32 lines, the level range of the band and how many
//bins are over the reference. Use it once to confirm the levels on real hardware.
#define WF_DEBUG_LEVELS      (0)


extern uint8_t vet_graf_fft[GRAPH_NUM_LINES][GRAPH_NUM_COLS];    // [NL][NCOL]
//extern uint16_t vet_graf_fft_pos;

// Use hardware SPI
extern TFT_eSPI tft;



//void tft_setup(void);
//void tft_writexy(uint8_t x, uint8_t y, uint8_t *s);
void tft_writexy_(uint16_t font, uint16_t color, uint16_t color_back, uint16_t x, uint16_t y, uint8_t *s);
void tft_writexy_plus(uint16_t font, uint16_t color, uint16_t color_back, uint16_t x, uint16_t x_plus, uint16_t y, uint16_t y_plus, uint8_t *s);
void tft_cursor(uint16_t font, uint16_t color, uint8_t x, uint8_t y);
void tft_cursor_plus(uint16_t font, uint16_t color, uint8_t x, uint8_t x_plus, uint8_t y, uint8_t y_plus);
//uint16_t tft_color565(uint16_t r, uint16_t g, uint16_t b);

// Smeter barr graph definitions
#define MAX_Smeter_table  11   // S1, S2..   S9, S9+  S9++  = 11 steps
void Smeter_bargraph(int16_t index_new); 
void TxPower_bargraph(int16_t index_new);

void display_fft_graf(uint16_t freq);
void display_fft_graf_top(void);
void display_tft_setup0(void);
void display_tft_setup0_num_mem_ok(uint8_t num);   //show num of eeprom memories read, on the setup0 display
void display_tft_setup(void);
void display_tft_countdown(bool show, uint16_t val);
void display_tft_loop(void);

void display_aud_graf(void);



#ifdef __cplusplus
}
#endif
#endif
