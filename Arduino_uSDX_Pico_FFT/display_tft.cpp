/*
 * display.cpp
 * 
 * Created: May 2022
 * Author: Klaus Fensterseifer
 * https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj
 * 
*/

#include "Arduino.h"
#include "SPI.h"
#include "uSDR.h"  //Serialx
#include "hmi.h"
#include "dsp.h"
#include "TFT_eSPI.h"
#include "display_tft.h"





// Use hardware SPI
TFT_eSPI tft = TFT_eSPI();
char vet_char[50];


uint16_t font_last = 0;
uint16_t color_last = 0;
uint16_t color_back_last = 0;
uint16_t size_last = 0;
uint16_t x_char_last = 0;
uint16_t y_char_last = 0;


void tft_writexy_(uint16_t font, uint16_t color, uint16_t color_back, uint16_t x, uint16_t y, uint8_t *s)
{
  if(font != font_last)
  {
    if(font == 3)
    {
      tft.setFreeFont(FONT3);                 // Select the font
      tft.setTextSize(SIZE3);  
      x_char_last = X_CHAR3;
      y_char_last = Y_CHAR3;
      size_last = SIZE3;
    }
    else if (font == 2)
    {
      tft.setFreeFont(FONT2);                 // Select the font
      tft.setTextSize(SIZE2);      
      x_char_last = X_CHAR2;
      y_char_last = Y_CHAR2;
      size_last = SIZE2;
    }
    else
    {
      tft.setFreeFont(FONT1);                 // Select the font
      tft.setTextSize(SIZE1);  //size 1 = 10 pixels, size 2 =20 pixels, and so on
      x_char_last = X_CHAR1;
      y_char_last = Y_CHAR1;
      size_last = SIZE1;
    }
    font_last = font;
  }

  if((color != color_last) || (color_back != color_back_last))
    {
    tft.setTextColor(color, color_back);
    color_last = color;
    color_back_last = color_back;
    }
    
  
  tft.drawString((const char *)s, x * x_char_last * size_last, y * y_char_last * size_last, 1);// Print the string name of the font
}




/* write text to display at line column plus a delta x and y */
void tft_writexy_plus(uint16_t font, uint16_t color, uint16_t color_back, uint16_t x, uint16_t x_plus, uint16_t y, uint16_t y_plus, uint8_t *s)
{
  if(font != font_last)
  {
    if(font == 3)
    {
      tft.setFreeFont(FONT3);                 // Select the font
      tft.setTextSize(SIZE3);  
      x_char_last = X_CHAR3;
      y_char_last = Y_CHAR3;
      size_last = SIZE3;
    }
    else if (font == 2)
    {
      tft.setFreeFont(FONT2);                 // Select the font
      tft.setTextSize(SIZE2);      
      x_char_last = X_CHAR2;
      y_char_last = Y_CHAR2;
      size_last = SIZE2;
    }
    else
    {
      tft.setFreeFont(FONT1);                 // Select the font
      tft.setTextSize(SIZE1);  //size 1 = 10 pixels, size 2 =20 pixels, and so on
      x_char_last = X_CHAR1;
      y_char_last = Y_CHAR1;
      size_last = SIZE1;
    }
    font_last = font;
  }

  if((color != color_last) || (color_back != color_back_last))
    {
    tft.setTextColor(color, color_back);
    color_last = color;
    color_back_last = color_back;
    }
    
  
  tft.drawString((const char *)s, (x * x_char_last * size_last)+x_plus, (y * y_char_last * size_last)+y_plus, 1);// Print the string name of the font
}






void tft_cursor(uint16_t font, uint16_t color, uint8_t x, uint8_t y)
{
    if(font == 3)
    {
      for(uint16_t i=1; i<((Y_CHAR3 * SIZE3)/8); i++)
      {
      tft.drawFastHLine (x * X_CHAR3 * SIZE3, ((y+1) * Y_CHAR3 * SIZE3) - i , ((X_CHAR3 * SIZE3)*9)/10, color);
      }
    }
    else if (font == 2)
    {
      for(uint16_t i=1; i<((Y_CHAR2 * SIZE2)/8); i++)
      {
      tft.drawFastHLine (x * X_CHAR2 * SIZE2, ((y+1) * Y_CHAR2 * SIZE2) - i , X_CHAR2 * SIZE2, color);
      }
    }
    else
    {
      for(uint16_t i=1; i<((Y_CHAR1 * SIZE1)/8); i++)
      {
      tft.drawFastHLine (x * X_CHAR1 * SIZE1, ((y+1) * Y_CHAR1 * SIZE1) - i , X_CHAR1 * SIZE1, color);
      }
    }
}


void tft_cursor_plus(uint16_t font, uint16_t color, uint8_t x, uint8_t x_plus, uint8_t y, uint8_t y_plus)
{
  static uint16_t x_old = 0;
  
    if(font == 3)
    {
      for(uint16_t i=1; i<((Y_CHAR3 * SIZE3)/8); i++)
      {
        tft.drawFastHLine (x_old, (((y+1) * Y_CHAR3 * SIZE3) - i)+y_plus , ((X_CHAR3 * SIZE3)*9)/10, TFT_BLACK);
        tft.drawFastHLine ((x * X_CHAR3 * SIZE3)+x_plus, (((y+1) * Y_CHAR3 * SIZE3) - i)+y_plus , ((X_CHAR3 * SIZE3)*9)/10, color);
      }
      x_old = (x * X_CHAR3 * SIZE3)+x_plus;
    }
    else if (font == 2)
    {
      for(uint16_t i=1; i<((Y_CHAR2 * SIZE2)/8); i++)
      {
        tft.drawFastHLine (x_old, (((y+1) * Y_CHAR2 * SIZE2) - i)+y_plus , X_CHAR2 * SIZE2, TFT_BLACK);
        tft.drawFastHLine ((x * X_CHAR2 * SIZE2)+x_plus, (((y+1) * Y_CHAR2 * SIZE2) - i)+y_plus , X_CHAR2 * SIZE2, color);
      }
      x_old = (x * X_CHAR2 * SIZE2)+x_plus;
    }
    else
    {
      for(uint16_t i=1; i<((Y_CHAR1 * SIZE1)/8); i++)
      {
        tft.drawFastHLine (x_old, (((y+1) * Y_CHAR1 * SIZE1) - i)+y_plus , X_CHAR1 * SIZE1, TFT_BLACK);
        tft.drawFastHLine ((x * X_CHAR1 * SIZE1)+x_plus, (((y+1) * Y_CHAR1 * SIZE1) - i)+y_plus , X_CHAR1 * SIZE1, color);
      }
      x_old = (x * X_CHAR1 * SIZE1)+x_plus;
    }
}


#if 0
/* used to allow calling from other modules, concentrate the use of tft variable locally */
uint16_t tft_color565(uint16_t r, uint16_t g, uint16_t b)
{
  return tft.color565(r, g, b);
}
#endif





#define bargraph_Y   (35+Y_CHAR2-3-8)   //line of display
#define bargraph_dY  6    //block high
#define bargraph_X   10     //initial column 
#define bargraph_dX  6    //block wide
#define bargraph_dX_space  1  //space between blocks
int Smeter_table_color[MAX_Smeter_table] = {  TFT_GREEN, TFT_GREEN, TFT_GREEN, TFT_GREEN, TFT_YELLOW, TFT_YELLOW, 
                                                  TFT_YELLOW, TFT_YELLOW, TFT_RED, TFT_RED, TFT_RED };

/*
    Smeter_bargraph
*/
void Smeter_bargraph(int16_t index_new)
{
  static int16_t index_old = 0;  //smeter table index = number of blocks to draw on smeter bar graph
  int16_t i;

#ifdef TST_MAX_SMETER_SWR
index_new = 10;
index_old = 0;
#endif

  if(index_new >= MAX_Smeter_table)
  {
    index_new = MAX_Smeter_table - 1;
  }

  if(tx_enable_changed == true)  //if changed tx-rx = display clear
  {
    index_old = 0;  //print all bargraph
  }

  if(index_old == 0) 
  {
    //draw Smeter first block = fixed one = min one block
    tft.fillRect((bargraph_X + ((bargraph_dX + bargraph_dX_space) * 0)), bargraph_Y, bargraph_dX, bargraph_dY, Smeter_table_color[0]);
  }

  if(index_new > index_old)
  {
    //bigger signal
    for(i=index_old+1; i<=index_new; i++)
    {
      //draw blocks from actual to the new index
      tft.fillRect((bargraph_X + ((bargraph_dX + bargraph_dX_space) * i)), bargraph_Y, bargraph_dX, bargraph_dY, Smeter_table_color[i]);
    }
  }
  else if(index_new < index_old)
  {
    //smaller signal
    for(i=index_new+1; i<=index_old; i++)
    {
      //erase blocks from actual to the new index
      tft.fillRect((bargraph_X + ((bargraph_dX + bargraph_dX_space) * i)), bargraph_Y, bargraph_dX, bargraph_dY, TFT_BLACK);
    }
  }

  index_old = index_new;

}




//int16_t TxPower_table_color[MAX_Smeter_table] = {  TFT_RED, TFT_RED, TFT_RED, TFT_RED, TFT_MAROON, TFT_MAROON, TFT_MAROON, TFT_MAROON, TFT_ORANGE, TFT_ORANGE, TFT_ORANGE };
int TxPower_table_color[MAX_Smeter_table] = {  TFT_MAROON, TFT_MAROON, TFT_MAROON, TFT_MAROON, TFT_RED, TFT_RED, TFT_RED, TFT_RED, TFT_MAGENTA, TFT_MAGENTA, TFT_MAGENTA };


/*
    TxPower_bargraph
*/
void TxPower_bargraph(int16_t index_new)
{
  static int16_t index_old = 0;  //smeter table index = number of blocks to draw on smeter bar graph
  int16_t i;

#ifdef TST_MAX_SMETER_SWR
index_new = 10;
index_old = 0;
#endif

  if(index_new >= MAX_Smeter_table)
  {
    index_new = MAX_Smeter_table - 1;
  }

  if(tx_enable_changed == true)  //if changed tx-rx = display clear
  {
    index_old = 0;  //print all bargraph
  }

  if(index_old == 0)
  {
    //draw Smeter first block = fixed one = min one block
    tft.fillRect((bargraph_X + ((bargraph_dX + bargraph_dX_space) * 0)), bargraph_Y, bargraph_dX, bargraph_dY, TxPower_table_color[0]);
  }

  if(index_new > index_old)
  {
    //bigger signal
    for(i=index_old+1; i<=index_new; i++)
    {
      //draw blocks from actual to the new index
      tft.fillRect((bargraph_X + ((bargraph_dX + bargraph_dX_space) * i)), bargraph_Y, bargraph_dX, bargraph_dY, TxPower_table_color[i]);
    }
  }
  else if(index_new < index_old)
  {
    //smaller signal
    for(i=index_new+1; i<=index_old; i++)
    {
      //erase blocks from actual to the new index
      tft.fillRect((bargraph_X + ((bargraph_dX + bargraph_dX_space) * i)), bargraph_Y, bargraph_dX, bargraph_dY, TFT_BLACK);
    }
  }

  index_old = index_new;
}









#define ABOVE_SCALE   12
#define TRIANG_TOP   (ABOVE_SCALE + 6)
#define TRIANG_WIDTH    8

int16_t triang_x_min = (display_WIDTH/2), triang_x_max = (display_WIDTH/2)+TRIANG_WIDTH;

/*********************************************************
  
*********************************************************/
void display_fft_graf_top(void) 
{
  int16_t siz, j, x;  //i, y
  uint32_t freq_graf_ini;
  uint32_t freq_graf_fim;



    //graph min freq
    freq_graf_ini = (hmi_freq - ((FFT_NSAMP/2)*FRES) )/1000;
  
    //graph max freq
    freq_graf_fim = (hmi_freq + ((FFT_NSAMP/2)*FRES) )/1000;

   
    //little triangle indicating the center freq
    switch(dsp_getmode())  //{"USB","LSB","AM","CW"}
    {
      case 0:  //USB
        triang_x_min = (display_WIDTH/2);
        triang_x_max = (display_WIDTH/2)+TRIANG_WIDTH;
        tft.fillTriangle(display_WIDTH/2, Y_MIN_DRAW - ABOVE_SCALE, 
                        display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, 
                        (display_WIDTH/2)+TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, TFT_YELLOW);
        tft.fillTriangle((display_WIDTH/2)-1, Y_MIN_DRAW - ABOVE_SCALE, 
                         display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, 
                         (display_WIDTH/2)-TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, TFT_BLACK);
        break;
      case 1:  //LSB
        triang_x_min = (display_WIDTH/2)-TRIANG_WIDTH;
        triang_x_max = (display_WIDTH/2);
        tft.fillTriangle(display_WIDTH/2, Y_MIN_DRAW - ABOVE_SCALE, 
                         display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, (display_WIDTH/2)-TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, 
                         TFT_YELLOW);
        tft.fillTriangle((display_WIDTH/2)+1, Y_MIN_DRAW - ABOVE_SCALE, 
                          display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, (display_WIDTH/2)+TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, 
                          TFT_BLACK);
        break;
      case 2:  //AM
        triang_x_min = (display_WIDTH/2)-TRIANG_WIDTH;
        triang_x_max = (display_WIDTH/2)+TRIANG_WIDTH;
        tft.fillTriangle((display_WIDTH/2)-TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, 
                         (display_WIDTH/2)+TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, TFT_YELLOW);
        break;
      case 3:  //CW = LSB
        triang_x_min = (display_WIDTH/2)-(TRIANG_WIDTH*2/4);
        triang_x_max = (display_WIDTH/2);   //-(TRIANG_WIDTH*1/4);
        tft.fillTriangle(display_WIDTH/2, Y_MIN_DRAW - ABOVE_SCALE, 
                         display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, 
                         (display_WIDTH/2)-TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, TFT_YELLOW);
        tft.fillTriangle((display_WIDTH/2)+1, Y_MIN_DRAW - ABOVE_SCALE, 
                          display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, 
                          (display_WIDTH/2)+TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, TFT_BLACK);
        break;
    }

    //erase old freqs on top of scale
    tft.fillRect(0, Y_MIN_DRAW - TRIANG_TOP - Y_CHAR1, display_WIDTH, Y_CHAR1, TFT_BLACK);

    //plot scale on top of waterfall
    tft.drawFastHLine (0, Y_MIN_DRAW - 11, display_WIDTH, TFT_WHITE);
    tft.fillRect(0, Y_MIN_DRAW - 10, display_WIDTH, 10, TFT_BLACK);
    tft.fillRect(triang_x_min, Y_MIN_DRAW - 10, (triang_x_max - triang_x_min + 1), 11, tft.color565(25, 25, 25)); //shadow
/*    
    //using #define TFT_RGB_ORDER TFT_GRB at Setup60_RP2040_ILI9341.h  (not affected by tft.setSwapBytes() )
    tft.fillRect(triang_x_min - 75, Y_MIN_DRAW - 10 -25, 12, 12, tft.color565(50, 50, 50));  //cinza escuro
    tft.fillRect(triang_x_min - 50, Y_MIN_DRAW - 10 -25, 12, 12, tft.color565(105, 105, 105));  //cinza
    tft.fillRect(triang_x_min - 25, Y_MIN_DRAW - 10 -25, 12, 12, tft.color565(200, 0, 0));  //red
    tft.fillRect(triang_x_min - 0, Y_MIN_DRAW - 10 -25, 12, 12, tft.color565(0, 200, 0));   //green
    tft.fillRect(triang_x_min + 25, Y_MIN_DRAW - 10 -25, 12, 12, tft.color565(0, 0, 200));  //blue
*/

    x=0;
    for(; freq_graf_ini < freq_graf_fim; freq_graf_ini+=1)
    {
      if((freq_graf_ini % 10) == 0)
      {
        tft.drawFastVLine (x, Y_MIN_DRAW - 11, 5, TFT_WHITE);
      }
      if((freq_graf_ini % 50) == 0)
      {
        tft.drawFastVLine (x-1, Y_MIN_DRAW - 11, 7, TFT_WHITE);
        tft.drawFastVLine (x, Y_MIN_DRAW - 11, 7, TFT_WHITE);
        tft.drawFastVLine (x+1, Y_MIN_DRAW - 11, 7, TFT_WHITE);
      }
      if((freq_graf_ini % 100) == 0)
      {
         tft.drawFastVLine (x-1, Y_MIN_DRAW - 11, 10, TFT_WHITE);
         tft.drawFastVLine (x, Y_MIN_DRAW - 11, 10, TFT_WHITE);
         tft.drawFastVLine (x+1, Y_MIN_DRAW - 11, 10, TFT_WHITE);
    
         //write new freq values  on top of scale
         sprintf(vet_char, "%lu", freq_graf_ini);
         siz = strlen(vet_char);
         if(x < (2*X_CHAR1))   //to much to left
         {
            tft_writexy_plus(1, TFT_MAGENTA, TFT_BLACK,0,0,7,0,(uint8_t *)vet_char);  
         }
         else if((x+((siz-2)*X_CHAR1)) > display_WIDTH)  //to much to right
         {
            j = display_WIDTH - (siz*X_CHAR1);
            tft_writexy_plus(1, TFT_MAGENTA, TFT_BLACK,0,j,7,0,(uint8_t *)vet_char);  
         }
         else
         {
            j = x - (2*X_CHAR1);
            tft_writexy_plus(1, TFT_MAGENTA, TFT_BLACK,0,j,7,0,(uint8_t *)vet_char);  
         }
      }
      x+=2;
    }
  

}







uint8_t vet_graf_fft[GRAPH_NUM_LINES][FFT_NSAMP];    // [NL][NCOL]
//uint16_t vet_graf_fft_pos = 0;


/*********************************************************
  Waterfall colors

  vet_graf_fft[] holds the log level of each bin (see mag_level() in dsp.cpp), one level
  step is 1.5 dB. The color is taken from the difference between the level and the noise
  floor of the band, not from the level itself.

  Why not from the level: with the AGC, with fft_gain and with propagation the band is
  sometimes generally weak and sometimes generally strong, so an absolute scale saturates.

  Why the floor is measured over the whole band and not per column: a carrier that stays
  on cannot be told apart from a band that got stronger. Any floor that follows one column
  eventually walks up into that carrier and erases it, which is exactly what happened here,
  first with a slow creep and then with a windowed minimum. Over the whole band it cannot
  happen, because a signal is narrow.

  Why the median and not the average or the minimum: a strong signal entering the band
  would drag the average up and wipe out the rest of the band, and a single quiet bin would
  drag a minimum down. The median ignores both, a signal only moves a handful of the
  320 columns.
 *********************************************************/
//marks the floor as not measured yet
#define WF_FLOOR_UNKNOWN  (0xFFu)

static uint16_t wf_palette[WF_COLOR_NUM];        //color index -> RGB565
static uint16_t wf_hist[WF_COLOR_NUM];           //histogram of the levels of one line
static uint16_t wf_line[GRAPH_NUM_COLS];         //one waterfall line in RGB565, sent in a single SPI burst
static uint8_t  wf_floor = WF_FLOOR_UNKNOWN;     //noise floor of the band, in levels
static uint8_t  wf_floor_tic = 0;                //lines counter, steps the floor up

// Shadow color for tune cursor area (bitwise OR blend)
// Initialized in wf_palette_setup() to match palette color space (RGB565/BGR565)
static uint16_t wf_shadow_color;

// Per-line horizontal offset for waterfall shifting
static int16_t wf_line_offset[GRAPH_NUM_LINES];

/*
//color ramp control points, RGB888, interpolated in wf_palette_setup()
// Index 0 = black (background), Index 1 = white (weakest signal), then yellow -> pink -> red
#define WF_RAMP_NUM  6
static const uint8_t wf_ramp_r[WF_RAMP_NUM] = {   0, 255, 255, 255, 255, 255 };
static const uint8_t wf_ramp_g[WF_RAMP_NUM] = {   0, 255, 255, 180,  50,   0 };
static const uint8_t wf_ramp_b[WF_RAMP_NUM] = {   0, 255,   0,   0, 100,   0 };
*/

// Color ramp control points, RGB888.
// Index 0 is reserved for black.
// Index 1 starts at white, then yellow -> orange -> pink -> red.
#define WF_RAMP_NUM  5
static const uint8_t wf_ramp_r[WF_RAMP_NUM] =  { 255, 255, 255, 255, 255 };
static const uint8_t wf_ramp_g[WF_RAMP_NUM] =  { 255, 255, 180,  50,   0 };
static const uint8_t wf_ramp_b[WF_RAMP_NUM] =  { 255,   0,   0, 100,   0 };


static uint8_t wf_ramp_lerp(const uint8_t *ramp, uint16_t seg, uint16_t frac)
{
  return (uint8_t)((int16_t)ramp[seg] + (((int16_t)ramp[seg+1] - (int16_t)ramp[seg]) * (int16_t)frac) / 255);
}


#if 0
/*********************************************************
  Build the 64 color ramp once, so that color565() is not called
  48*320 times per frame
 *********************************************************/
static void wf_palette_setup(void)
{
  static bool palette_ready = false;
  uint16_t i, seg, frac;

  if(palette_ready == true)
  {
    return;
  }

  for(i=0; i<WF_COLOR_NUM; i++)
  {
    seg  = (uint16_t)(((uint32_t)i * (WF_RAMP_NUM-1u)) / (WF_COLOR_NUM-1u));
    frac = (uint16_t)((((uint32_t)i * (WF_RAMP_NUM-1u)) % (WF_COLOR_NUM-1u)) * 255u / (WF_COLOR_NUM-1u));
    if(seg >= (WF_RAMP_NUM-2u))
    {
      seg  = WF_RAMP_NUM-2u;
      frac = 255u;
    }

    uint8_t r = wf_ramp_lerp(wf_ramp_r, seg, frac);
    uint8_t g = wf_ramp_lerp(wf_ramp_g, seg, frac);
    uint8_t b = wf_ramp_lerp(wf_ramp_b, seg, frac);
    wf_palette[i] = tft.color565(r, g, b);
/*
    // Debug: print first few palette entries with intermediate values
    if(i < 5)
    {
      Serialx.print("i="); Serialx.print(i);
      Serialx.print(" seg="); Serialx.print(seg);
      Serialx.print(" frac="); Serialx.print(frac);
      Serialx.print(" rgb=("); Serialx.print(r); Serialx.print(","); Serialx.print(g); Serialx.print(","); Serialx.print(b); Serialx.print(")");
      Serialx.print(" palette=0x"); Serialx.println(wf_palette[i], HEX);
    }
*/
  }

  wf_shadow_color = tft.color565(25, 25, 25);  // init after tft is ready, matches palette color space

  palette_ready = true;
}
#endif


/*********************************************************
  Build the 64-entry palette:
  Index 0 = black (background)
  Index 1 = white (weakest visible signal)
  Index 2–15	= white to yellow transition
  Index 16–31	= yellow to orange transition
  Index 32–47	= orange to pink-red transition
  Index 48–63	= pink-red to red transition
  Index 63 = red (strongest signal)
 *********************************************************/
static void wf_palette_setup(void)
{
  //static bool palette_ready = false;
  uint16_t i, seg, frac;
  uint32_t pos;
/*
  if(palette_ready == true)    
  {
    return;
  }
*/
  // Reserve index 0 exclusively for black.
  wf_palette[0] = tft.color565(0, 0, 0);

  // Spread the five color control points over indices 1..63.
  for(i = 1; i < WF_COLOR_NUM; i++)
  {
    pos = (uint32_t)(i - 1u) * (WF_RAMP_NUM - 1u);

    seg = (uint16_t)(pos / (WF_COLOR_NUM - 2u));

    frac = (uint16_t)(
      (pos % (WF_COLOR_NUM - 2u)) * 255u
      / (WF_COLOR_NUM - 2u)
    );

    // Ensure the final index is exactly red.
    if(seg >= (WF_RAMP_NUM - 1u))
    {
      seg  = WF_RAMP_NUM - 2u;
      frac = 255u;
    }

    uint8_t r = wf_ramp_lerp(wf_ramp_r, seg, frac);
    uint8_t g = wf_ramp_lerp(wf_ramp_g, seg, frac);
    uint8_t b = wf_ramp_lerp(wf_ramp_b, seg, frac);

    wf_palette[i] = tft.color565(r, g, b);
  }

  // Initialize the tuning cursor shadow.
  wf_shadow_color = tft.color565(25, 25, 25);

//  palette_ready = true;
}



#if 0
/*********************************************************
  Darken a color for the tune cursor shadow.
  Keeping the top bits of each channel, the former TFT_WHITE|extra_color was a
  no op (0xFFFF | 0x18C3 == 0xFFFF) and the signal pixels were never dimmed.
  Halving each channel scales the intensity without moving the hue, and black
  stays black and white becomes grey.
 *********************************************************/
static uint16_t wf_shadow(uint16_t color)
{
  uint16_t r = (color >> 11) & 0x1Fu;
  uint16_t g = (color >>  5) & 0x3Fu;
  uint16_t b =  color        & 0x1Fu;

  return (uint16_t)(((r >> 1) << 11) | ((g >> 1) << 5) | (b >> 1));
}
#endif

/*********************************************************
  Convert the newest FFT line (row GRAPH_NUM_LINES-1) from levels into color
  indices, measuring the noise floor of the band on the way.

  The floor is the median level of the line, found with a histogram so that it costs
  one increment per column plus one scan of 64 bins, no sorting. The median is what
  makes a carrier safe: it only occupies a few of the 320 columns, so it cannot move
  the reference, no matter how long it stays on.

  The floor drops at once, so a band that gets weaker re centers without lag, and it
  climbs one step every WF_FLOOR_RISE_DIV lines, so a single quiet bin cannot darken
  the whole waterfall.

  Called before the drawing
 *********************************************************/
static void wf_colorize_new_line(void)
{
  uint16_t x, acc;
  uint8_t  lvl, idx, median;
  int16_t  above;
  bool     step_up;

  //histogram of the levels of this line
  memset(wf_hist, 0, sizeof(wf_hist));
  for(x=0; x<GRAPH_NUM_COLS; x++)
  {
    lvl = vet_graf_fft[GRAPH_NUM_LINES-1][x];
    if(lvl < WF_COLOR_NUM)
    {
      wf_hist[lvl]++;
    }
  }

  //median: the level that half of the columns are below
  acc    = 0;
  median = 0;
  for(x=0; x<WF_COLOR_NUM; x++)
  {
    acc += wf_hist[x];
    if((acc * 2u) >= GRAPH_NUM_COLS)
    {
      median = (uint8_t)x;
      break;
    }
  }

  if(wf_floor == WF_FLOOR_UNKNOWN)
  {
    wf_floor = median;             //first line, take the measurement as it is
  }
  else if(median < wf_floor)
  {
    wf_floor = median;             //weaker band, follow it down at once
  }
  else
  {
    step_up = ((wf_floor_tic++ & (WF_FLOOR_RISE_DIV-1u)) == 0u);
    if((step_up == true) && (wf_floor < WF_COLOR_NUM-1u))
    {
      wf_floor++;
    }
  }

  #if WF_DEBUG_LEVELS
    {   //one report every 32 lines: where the band sits and how much is above the reference
      static uint8_t dbg_tic = 0;
      uint8_t  dbg_min = 255, dbg_max = 0, dbg_over = 0;
      uint16_t n;
      for(n=0; n<GRAPH_NUM_COLS; n++)
      {
        lvl = vet_graf_fft[GRAPH_NUM_LINES-1][n];
        if(lvl < dbg_min) { dbg_min = lvl; }
        if(lvl > dbg_max) { dbg_max = lvl; }
        if(lvl > (uint8_t)(wf_floor + WF_ABOVE_FLOOR)) { dbg_over++; }
      }
      if(((dbg_tic++) & 31u) == 0u)
      {
        Serialx.print("wf line ");   Serialx.print(dbg_tic);
        Serialx.print("  min=");      Serialx.print(dbg_min);
        Serialx.print("  median=");   Serialx.print(median);
        Serialx.print("  max=");      Serialx.print(dbg_max);
        Serialx.print("  floor=");    Serialx.print(wf_floor);
        Serialx.print("  above_ref="); Serialx.print(dbg_over);
        Serialx.print("  brightest_color="); Serialx.println((int)dbg_max);
      }
    }
#endif

  for(x=0; x<GRAPH_NUM_COLS; x++)
  {
    lvl = vet_graf_fft[GRAPH_NUM_LINES-1][x];

    //color index from how much the bin is above the reference of the band
    above = (int16_t)lvl - (int16_t)wf_floor - (int16_t)WF_ABOVE_FLOOR;
    if(above <= 0)
    {
      idx = 0;                //at or below the reference, this is background
    }
    else
    {
      idx = (uint8_t)(((uint32_t)above * WF_COLOR_NUM) / WF_WINDOW);
      if(idx > (WF_COLOR_NUM-1u))
      {
        idx = WF_COLOR_NUM-1u;
      }

      //A bin that is above the reference never gets painted like the background.
      //The scale is relative to the band, but this floor of one step does not
      //depend on it, so a signal can never be hidden by the floor calculation.
      if(idx == 0)
      {
        idx = 1;
      }
    }

    vet_graf_fft[GRAPH_NUM_LINES-1][x] = idx;
  }
}


/*********************************************************
  The floor belongs to the band and not to a position on the screen, so it does not
  move with the dial. A dial jump bigger than the screen brings an unknown band, drop
  the reference so it is measured again from scratch
 *********************************************************/
static void wf_floor_move(int16_t freq_change)
{
  if((freq_change >= (int16_t)GRAPH_NUM_COLS) || (freq_change <= -(int16_t)GRAPH_NUM_COLS))
  {
    wf_floor = WF_FLOOR_UNKNOWN;
    wf_floor_tic = 0;
  }
}

  
/*********************************************************
  Plot the waterfall, and move it sideways when the dial moved
*********************************************************/
void display_fft_graf(uint16_t freq)    //receive the actual freq to move the waterfall as changing the dial
{                                       //consider 1kHz for each pixel on horizontal
  uint16_t x, y;
  uint16_t y_screen;
  uint16_t color;
  static uint16_t freq_old = 7080;
  int16_t freq_change;

  //wf_palette_setup();  moved to display_tft_setup()

  freq_change = ((int16_t)freq - (int16_t)freq_old);  //how much then dial (freq) changed

  //the floors move with the dial, before the new line is colored
  wf_floor_move(freq_change);

  //levels -> color indices of the new line, this also refreshes the noise floors
  wf_colorize_new_line();

  // Apply frequency change to all line offsets (visual shift effect)
  if(freq_change != 0)
  {
    for(y=0; y<GRAPH_NUM_LINES; y++)
    {
      wf_line_offset[y] += freq_change;
      // Clamp to valid range
      if(wf_line_offset[y] > (int16_t)GRAPH_NUM_COLS) wf_line_offset[y] = GRAPH_NUM_COLS;
      if(wf_line_offset[y] < -(int16_t)GRAPH_NUM_COLS) wf_line_offset[y] = -GRAPH_NUM_COLS;
    }
  }

  // New line captured at current frequency -> offset 0
  wf_line_offset[GRAPH_NUM_LINES-1] = 0;

  //plot waterfall
  //vet_graf_fft[GRAPH_NUM_LINES][GRAPH_NUM_COLS]   [NL][NCOL]
  for(y=0; y<GRAPH_NUM_LINES; y++)   // y=0 is the line at the bottom   y=(GRAPH_NUM_LINES-1) is the new line
  {
    y_screen = (GRAPH_NUM_LINES + Y_MIN_DRAW - y);

    //plot waterfall line, whole line in RGB565 sent in a single SPI burst
    // Each line drawn with its own horizontal offset (tracked per line)
    int16_t offset = wf_line_offset[y];
    for(x=0; x<GRAPH_NUM_COLS; x++)
    {
  
      int16_t src_x = x + offset;
      uint8_t idx = 0;
      if(src_x >= 0 && src_x < (int16_t)GRAPH_NUM_COLS)
      {
        idx = vet_graf_fft[y][src_x];
      }
      color = wf_palette[idx];

      if((x >= (uint16_t)triang_x_min) && (x <= (uint16_t)triang_x_max))  //tune shadow area
      {
        if(idx==0)  //tft.color565(0,0,0) = 0x0000 = black = background without color
          color = tft.color565(25, 25, 25);  //dark gray
      }

      wf_line[x] = color;
  
/*
    //it needs tft.setSwapBytes(true) to set the correct colors
    if(x<(GRAPH_NUM_COLS/3))
      wf_line[x] =  tft.color565(200, 0, 0);  //red
    else if(x<((GRAPH_NUM_COLS * 2)/3))
      wf_line[x] =  tft.color565(0, 200, 0);   /green
    else
      wf_line[x] =  tft.color565(0, 0, 200);  //blue
*/
    }

    tft.startWrite();
    tft.setAddrWindow(0, y_screen, GRAPH_NUM_COLS, 1);
    tft.pushPixels(wf_line, GRAPH_NUM_COLS);
    tft.endWrite();
  }


  //move graph data array one line up to open space for next FFT (new line will be the last line)
  for(y=0; y<(GRAPH_NUM_LINES-1); y++)
  {
    memcpy(vet_graf_fft[y], vet_graf_fft[y+1], GRAPH_NUM_COLS);
    wf_line_offset[y] = wf_line_offset[y+1];
  }


  freq_old = freq;  //take note of the freq center on last waterfall printed on display
}






void display_aud_graf_var(uint16_t aud_pos, uint16_t aud_var, uint16_t color)
{  
  int16_t x;
  int16_t aud; 
  
  for(x=0; x<AUD_GRAPH_NUM_COLS; x++)
  {
    aud = (aud_samp[aud_var][x+aud_pos]);
    
    if(aud < AUD_GRAPH_MIN)  //check boundaries
    {
      tft.drawPixel((x + X_MIN_AUD_GRAPH), (Y_MIN_AUD_GRAPH + AUD_GRAPH_MAX - AUD_GRAPH_MIN), color);    //lower line    
    }
    else if(aud > AUD_GRAPH_MAX)  //check boundaries
    {
      tft.drawPixel((x + X_MIN_AUD_GRAPH), (Y_MIN_AUD_GRAPH + AUD_GRAPH_MAX - AUD_GRAPH_MAX), color);    //upper line 
    }
    else
    {
      tft.drawPixel((x + X_MIN_AUD_GRAPH), (Y_MIN_AUD_GRAPH + AUD_GRAPH_MAX - aud), color);        
    }
  }
  
}




void display_aud_graf(void)
{
uint16_t aud_pos;
int16_t x;
int16_t aud_samp_trigger;
  
  //erase graphic area
  //tft.fillRect(0, Y_MIN_DRAW - 10, display_WIDTH, 10, TFT_BLACK);
  tft.fillRect(X_MIN_AUD_GRAPH, Y_MIN_AUD_GRAPH, AUD_GRAPH_NUM_COLS, (AUD_GRAPH_MAX - AUD_GRAPH_MIN + 1), TFT_BLACK);

 // if(tx_enabled)
  {
 //   aud_samp_trigger = AUD_SAMP_MIC;  
  }
//  else
  {
    aud_samp_trigger = AUD_SAMP_I;
  }

  //find trigger point to start ploting
  for(x=0; x<AUD_GRAPH_NUM_COLS; x++)
  {
     if((aud_samp[aud_samp_trigger][x+0] > 0) &&
        (aud_samp[aud_samp_trigger][x+1] > 0) &&
        (aud_samp[aud_samp_trigger][x+2] > 0) &&
        (aud_samp[aud_samp_trigger][x+3] > 0) &&
        (aud_samp[aud_samp_trigger][x+4] > 0))
        {
          break;
        }
  }
  for(; x<AUD_GRAPH_NUM_COLS; x++)
  {
     if(aud_samp[aud_samp_trigger][x] < 0)
        {
          break;
        }
  }
  aud_pos = x;

  //plot each variable
  display_aud_graf_var(aud_pos, AUD_SAMP_I, TFT_GREEN);
  display_aud_graf_var(aud_pos, AUD_SAMP_Q, TFT_CYAN);
  display_aud_graf_var(aud_pos, AUD_SAMP_MIC, TFT_RED);
  display_aud_graf_var(aud_pos, AUD_SAMP_A, TFT_PINK);
  display_aud_graf_var(aud_pos, AUD_SAMP_PEAK, TFT_YELLOW);
  display_aud_graf_var(aud_pos, AUD_SAMP_GAIN, TFT_MAGENTA);

}










/*********************************************************
  Initial msgs on display  (after reset)
*********************************************************/
void display_tft_setup0(void) {
  char s[32];
  
  tft.init();
  tft.setRotation(ROTATION_SETUP);
 
  tft.fillScreen(TFT_BLACK);

//  sprintf(s, "uSDR Pico");
  sprintf(s, "Arjan-5");  //name changed from uSDR Pico FFT
  tft_writexy_plus(3, TFT_YELLOW, TFT_BLACK, 2,10,1,0,(uint8_t *)s);

  //tft.fillRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t radius, uint32_t color)
  //tft.drawRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t radius, uint32_t color)
  tft.drawRoundRect(35, 25, 250, 70, 15, TFT_YELLOW);

//  sprintf(s, "uSDR Pico FFT");  //name changed from uSDR Pico FFT
  sprintf(s, "5 Band SSB/AM/CW");  //name changed from uSDR Pico FFT
//  tft_writexy_plus(2, TFT_YELLOW, TFT_BLACK, 0,0,3,10,(uint8_t *)s);
  tft_writexy_plus(1, TFT_YELLOW, TFT_BLACK, 3,0,5,0,(uint8_t *)s);

  sprintf(s, "HF Transceiver");  //name changed from uSDR Pico FFT
//  tft_writexy_plus(2, TFT_YELLOW, TFT_BLACK, 0,10,4,10,(uint8_t *)s);
  tft_writexy_plus(1, TFT_YELLOW, TFT_BLACK, 4,0,6,6,(uint8_t *)s);

  tft_writexy_plus(1, TFT_YELLOW, TFT_BLACK, 6,0,7,12,(uint8_t *)SW_VERSION);

//#ifdef PY2KLA_setup
//  sprintf(s, "by");
//  tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 0,0,5,0,(uint8_t *)s);
//  sprintf(s, "PE1ATM");
//  sprintf(s, "by");
//  tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 0,0,7,0,(uint8_t *)s);
  sprintf(s, "Arjan te Marvelde");
  tft_writexy_plus(1, TFT_SKYBLUE, TFT_BLACK, 3,0,9,0,(uint8_t *)s);
//  sprintf(s, "and");
//  tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 0,0,8,0,(uint8_t *)s);
// sprintf(s, "PY2KLA");
//  sprintf(s, "and");
//  tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 0,0,9,0,(uint8_t *)s);
  sprintf(s, "Klaus Fensterseifer");
  tft_writexy_plus(1, TFT_SKYBLUE, TFT_BLACK, 2,0,10,0,(uint8_t *)s);
//#endif  
}


/*********************************************************
  Show the num of memories on setup0 display
*********************************************************/
void display_tft_setup0_num_mem_ok(uint8_t num) 
{
  char s[12];
  sprintf(s, "%d", num);  //num of memories read from eeprom
  tft_writexy_plus(1, TFT_YELLOW, TFT_BLACK, 6+12,0,7,12,(uint8_t *)s);
}



/*********************************************************
  Setup
*********************************************************/
void display_tft_setup(void) {

//uint16_t x, y;
char s[32];
  
//  tft.init();
//  tft.setRotation(ROTATION_SETUP);

  tft.fillScreen(TFT_BLACK);
  
  tft.setSwapBytes(true);  //used with tft.pushPixels() to set the correct colors

  //tft.drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
  //tft.drawRect(X_MIN_AUD_GRAPH-1, Y_MIN_AUD_GRAPH-1, AUD_GRAPH_NUM_COLS+2, (AUD_GRAPH_MAX - AUD_GRAPH_MIN + 1)+2, TFT_WHITE);

  //plot scale on left of audio scope
  tft.drawFastVLine (X_MIN_AUD_GRAPH - 11, Y_MIN_AUD_GRAPH, (AUD_GRAPH_MAX - AUD_GRAPH_MIN), TFT_WHITE);
  
  //tft.fillRect(0, Y_MIN_DRAW - 10, display_WIDTH, 10, TFT_BLACK);

  tft.drawFastHLine (X_MIN_AUD_GRAPH - 11, Y_MIN_AUD_GRAPH, 8, TFT_WHITE);
  tft.drawFastHLine (X_MIN_AUD_GRAPH - 11, Y_MIN_AUD_GRAPH+5, 5, TFT_WHITE);
  tft.drawFastHLine (X_MIN_AUD_GRAPH - 11, Y_MIN_AUD_GRAPH+15, 5, TFT_WHITE);
  tft.drawFastHLine (X_MIN_AUD_GRAPH - 11, Y_MIN_AUD_GRAPH+25, 8, TFT_WHITE);
  tft.drawFastHLine (X_MIN_AUD_GRAPH - 11, Y_MIN_AUD_GRAPH+35, 5, TFT_WHITE);
  tft.drawFastHLine (X_MIN_AUD_GRAPH - 11, Y_MIN_AUD_GRAPH+45, 5, TFT_WHITE);
  tft.drawFastHLine (X_MIN_AUD_GRAPH - 11, Y_MIN_AUD_GRAPH+50, 8, TFT_WHITE);
 




  sprintf(s, "I");
  tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 8,6,1,0,(uint8_t *)s);
  sprintf(s, "Q");
  tft_writexy_plus(1, TFT_CYAN, TFT_BLACK, 8,6,2,0,(uint8_t *)s);
  sprintf(s, "A");
  tft_writexy_plus(1, TFT_PINK, TFT_BLACK, 8,6,3,0,(uint8_t *)s);
  sprintf(s, "MIC");
  tft_writexy_plus(1, TFT_RED, TFT_BLACK, 10,4,1,0,(uint8_t *)s);
  sprintf(s, "PEAK");
  tft_writexy_plus(1, TFT_YELLOW, TFT_BLACK, 10,4,2,0,(uint8_t *)s);
  sprintf(s, "GAIN");
  tft_writexy_plus(1, TFT_MAGENTA, TFT_BLACK, 10,4,3,0,(uint8_t *)s);



  wf_palette_setup();  //setup for graphic colors

   
} 




/*********************************************************
  Initial msgs on display  (after reset)
*********************************************************/
void display_tft_countdown(bool show, uint16_t val) 
{
  char s[32];

  if(show == true)
  {
    tft.drawRoundRect(260, 100, 50, 40, 10, TFT_ORANGE);
    sprintf(s, "%d", val);  //name changed from uSDR Pico FFT

    //tft.fillRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t radius, uint32_t color)
    if(val < 10)
    {
      tft_writexy_plus(1, TFT_ORANGE, TFT_BLACK, 20,4,5,0,(uint8_t *)s);
      //tft.drawRoundRect(275, 100, 35, 40, 10, TFT_ORANGE);
    }
    else
    {
      tft_writexy_plus(1, TFT_ORANGE, TFT_BLACK, 19,4,5,0,(uint8_t *)s);
      //tft.drawRoundRect(260, 100, 50, 40, 10, TFT_ORANGE);
    }
  }
  else  //fill with black = erase = close the window
  {
    tft.fillRoundRect(260, 100, 50, 40, 10, TFT_BLACK);
  }
}





/*********************************************************
  Loop
*********************************************************/
void display_tft_loop(void) 
{
  static uint32_t hmi_freq_fft;

  if (TX_ENABLED_OUT == false)  //waterfall only during RX
  {
    if (fft_display_graf_new == 1)    //design a new graphic only when a new line is ready from FFT
    {
      if(hmi_freq == hmi_freq_fft)
      {
        //plot waterfall graphic     
        display_fft_graf((uint16_t)(hmi_freq/500));  // warefall 110ms
      }
      else
      {
        //plot waterfall graphic     
        display_fft_graf((uint16_t)(hmi_freq_fft/500));  // warefall 110ms
        hmi_freq_fft = hmi_freq;
      }

      fft_display_graf_new = 0;  
      fft_samples_ready = 2;  //ready to start new sample collect
    }
  }

}
