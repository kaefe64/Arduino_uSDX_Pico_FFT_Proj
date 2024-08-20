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
#include "dsp.h"
#include "TFT_eSPI.h"
#include "display_tft.h"
#include "hmi.h"





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



/* used to allow calling from other modules, concentrate the use of tft variable locally */
uint16_t tft_color565(uint16_t r, uint16_t g, uint16_t b)
{
  return tft.color565(r, g, b);
}









// Smeter barr graph definitions
#define MAX_Smeter_table  11   // S1, S2..   S9, S9+  S9++  = 11 steps
#define Smeter_Y   96   //line of display
#define Smeter_dY  6    //block high
#define Smeter_X   0     //initial column 
#define Smeter_dX  4    //block wide
#define Smeter_dX_space  1  //space between blocks
/*
S Meter  	Antenna input
Reading   uVrms @ 50R
S9+20	    500
S9+10	    160
S9 	       50
S8 	       25
S7 	       12,5
S6 	       6,25
S5 	       3,125
S4 	       1,5625
S3 	       0,78125
S2 	       0,39063
S1 	       0,19531
*/
//                                             S  1  2  3  4   5   6   7    8    9   9+  9++
int16_t Smeter_table_level[MAX_Smeter_table] = {  1, 2, 4, 9, 18, 35, 75, 150, 300, 400, 600 };  //audio signal value after filters for each antenna level input
int16_t Smeter_table_color[MAX_Smeter_table] = {  TFT_GREEN, TFT_GREEN, TFT_GREEN, TFT_GREEN, TFT_YELLOW, TFT_YELLOW, TFT_YELLOW, TFT_YELLOW, TFT_RED, TFT_RED, TFT_RED };


int16_t Smeter(int16_t v)
{
  int16_t Smeter_index_new, i;
  static int16_t Smeter_index = 0;  //smeter table index = number of blocks to draw on smeter bar graph

  //look for smeter table index
  for(Smeter_index_new=(MAX_Smeter_table-1);  Smeter_index_new>0; Smeter_index_new--)
  {
    if(v > Smeter_table_level[Smeter_index_new])
      {
      break;
      }
  }

  if(Smeter_index_new > Smeter_index)
  {
    //bigger signal
    for(i=Smeter_index+1; i<=Smeter_index_new; i++)
    {
      //draw blocks from actual to the new index
      tft.fillRect((Smeter_X + ((Smeter_dX + Smeter_dX_space) * i)), Smeter_Y, Smeter_dX, Smeter_dY, Smeter_table_color[i]);
    }
  }
  else if(Smeter_index_new < Smeter_index)
  {
    //smaller signal
    for(i=Smeter_index_new+1; i<=Smeter_index; i++)
    {
      //erase blocks from actual to the new index
      tft.fillRect((Smeter_X + ((Smeter_dX + Smeter_dX_space) * i)), Smeter_Y, Smeter_dX, Smeter_dY, TFT_BLACK);
    }
  }

  Smeter_index = Smeter_index_new;

  return (Smeter_index+1);  // S level = index + 1
}












#define ABOVE_SCALE   12
#define TRIANG_TOP   (ABOVE_SCALE + 6)
#define TRIANG_WIDTH    8

int16_t triang_x_min, triang_x_max;

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
        tft.fillTriangle(display_WIDTH/2, Y_MIN_DRAW - ABOVE_SCALE, display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, (display_WIDTH/2)+TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, TFT_YELLOW);
        tft.fillTriangle((display_WIDTH/2)-1, Y_MIN_DRAW - ABOVE_SCALE, display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, (display_WIDTH/2)-TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, TFT_BLACK);
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
        tft.fillTriangle((display_WIDTH/2)-TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, (display_WIDTH/2)+TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, TFT_YELLOW);
        break;
      case 3:  //CW = LSB
        triang_x_min = (display_WIDTH/2)-(TRIANG_WIDTH*2/4);
        triang_x_max = (display_WIDTH/2);   //-(TRIANG_WIDTH*1/4);
        tft.fillTriangle(display_WIDTH/2, Y_MIN_DRAW - ABOVE_SCALE, 
                         display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, (display_WIDTH/2)-TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, 
                         TFT_YELLOW);
        tft.fillTriangle((display_WIDTH/2)+1, Y_MIN_DRAW - ABOVE_SCALE, 
                          display_WIDTH/2, Y_MIN_DRAW - TRIANG_TOP, (display_WIDTH/2)+TRIANG_WIDTH, Y_MIN_DRAW - ABOVE_SCALE, 
                          TFT_BLACK);
        break;
    }

    //erase old freqs on top of scale
    tft.fillRect(0, Y_MIN_DRAW - TRIANG_TOP - Y_CHAR1, display_WIDTH, Y_CHAR1, TFT_BLACK);

    //plot scale on top of waterfall
    tft.drawFastHLine (0, Y_MIN_DRAW - 11, display_WIDTH, TFT_WHITE);
    tft.fillRect(0, Y_MIN_DRAW - 10, display_WIDTH, 10, TFT_BLACK);
    tft.fillRect(triang_x_min, Y_MIN_DRAW - 10, (triang_x_max - triang_x_min + 1), 11, tft.color565(25, 25, 25)); //shadow
    
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
  
*********************************************************/
void display_fft_graf(uint16_t freq)    //receive the actual freq to move the waterfall as changing the dial
{                                       //consider 1kHz for each pixel on horizontal
  int16_t x, y;
  uint16_t extra_color;
  static uint16_t freq_old = 7080;
  int16_t freq_change;

  freq_change = ((int16_t)freq - (int16_t)freq_old);  //how much then dial (freq) changed

  //erase waterfall area
  //tft.fillRect(0, Y_MIN_DRAW, GRAPH_NUM_COLS, GRAPH_NUM_LINES, TFT_BLACK);

  extra_color = tft.color565(25, 25, 25);  //light shadow on center freq

  //plot waterfall
  //vet_graf_fft[GRAPH_NUM_LINES][GRAPH_NUM_COLS]   [NL][NCOL]
  for(y=0; y<GRAPH_NUM_LINES; y++)   // y=0 is the line at the bottom   y=(GRAPH_NUM_LINES-1) is the new line
  {
    //erase one waterfall line
    tft.drawFastHLine (0, (GRAPH_NUM_LINES + Y_MIN_DRAW - y), GRAPH_NUM_COLS, TFT_BLACK);

#ifdef WATERFALL_IN_BLOCK   // all lines in the waterfall move with the freq change (not only the new line)
    //move the waterfall in block according the freq change
    //if (freq changed) and (its is not the last line) // new line does not move, it is in the right position
    if((freq_change != 0) && (y < (GRAPH_NUM_LINES-1)))
    {      
      //  move the waterfall line to left or right  according to the freq change
      if(freq_change > 0)    // new freq greater than old
      {
        if(freq_change < GRAPH_NUM_COLS)   //change in the visible area
        {
          for(x=0; x<GRAPH_NUM_COLS; x++)
          {
            if((x + freq_change) < GRAPH_NUM_COLS)
            {
              vet_graf_fft[y][x] = vet_graf_fft[y][x + freq_change];   //move to left
            }
            else
            {
              vet_graf_fft[y][x] = 0;  //fill with empty
            }
          }    
        }
        else  //big change, make a empty line (erase the old data)
        {
          vet_graf_fft[y][x] = 0;
        }
      }
      else   // new freq smaller than old = freq_change is negative
      {
        if((-freq_change) < GRAPH_NUM_COLS)   //change in the visible area
        {
          for(x=GRAPH_NUM_COLS-1; x>=0; x--)
          {
            if((x + freq_change) > 0)
            {
              vet_graf_fft[y][x] = vet_graf_fft[y][x + freq_change];   //move to right
            }
            else
            {
              vet_graf_fft[y][x] = 0;  //fill with empty
            }
          }    
        }
        else  //big change, make a empty line (erase the old data)
        {
          vet_graf_fft[y][x] = 0;
        }
      }
    }      
#endif

    //plot waterfall line
    for(x=0; x<GRAPH_NUM_COLS; x++)
    {
      //plot one waterfall line
      if(vet_graf_fft[y][x] > 0)
      {
        if((x>=triang_x_min) && (x<=triang_x_max))  //tune shadow area
        {
          tft.drawPixel(x, (GRAPH_NUM_LINES + Y_MIN_DRAW - y), TFT_WHITE|extra_color); 
        }
        else
        {
          tft.drawPixel(x, (GRAPH_NUM_LINES + Y_MIN_DRAW - y), TFT_WHITE); 
        }
      }
      else
      {
        if((x>=triang_x_min) && (x<=triang_x_max))  //tune shadow area
        {
          tft.drawPixel(x, (GRAPH_NUM_LINES + Y_MIN_DRAW - y), TFT_BLACK|extra_color); 
        }
      }
    }
  }



  //move graph data array one line up to open space for next FFT (new line will be the last line)
  for(y=0; y<(GRAPH_NUM_LINES-1); y++)
  {
    for(x=0; x<GRAPH_NUM_COLS; x++)
    {
      vet_graf_fft[y][x] = vet_graf_fft[y+1][x];
    }
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
  tft_writexy_plus(1, TFT_YELLOW, TFT_BLACK, 4,0,6,10,(uint8_t *)s);


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
  
*********************************************************/
void display_tft_setup(void) {

//uint16_t x, y;
char s[32];
  
//  tft.init();
//  tft.setRotation(ROTATION_SETUP);

  tft.fillScreen(TFT_BLACK);
  

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


/* 
  tft.setFreeFont(FONT1);                 // Select the font
  txt_size = 1;
  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.setTextSize(txt_size);  //size 1 = 10 pixels, size 2 =20 pixels, and so on

  x = 0;
  y = 0;
  sprintf(vet_char, "%dx%d", display_WIDTH/(X_CHAR1 * txt_size) ,display_HEIGHT/(Y_CHAR1 * txt_size));
  //sprintf(vet_char, "%dx%d %dx%d",display_WIDTH ,display_HEIGHT
  //                , display_WIDTH/(X_CHAR1 * txt_size) ,display_HEIGHT/(Y_CHAR1 * txt_size));
  //tft.setCursor(0, 0);
  //tft.println(vet_char);
  tft.drawString(vet_char, x * X_CHAR1 * txt_size, y * Y_CHAR1 * txt_size, 1);// Print the string name of the font

  x = 1;
  y = 1;
  sprintf(vet_char, "x=%d y=%d",x ,y);
  //tft.setCursor(x * X_CHAR1 * txt_size, y * Y_CHAR1 * txt_size);
  //tft.println(vet_char);
  tft.drawString(vet_char, x * X_CHAR1 * txt_size, y * Y_CHAR1 * txt_size, 1);// Print the string name of the font
*/
/*  
  x = 0;
  y = 4;
  sprintf(vet_char, "x=%d y=%d",x ,y);
  //tft.setCursor(x * X_CHAR1 * txt_size, y * Y_CHAR1 * txt_size);
  //tft.println(vet_char);
  tft.drawString(vet_char, x * X_CHAR1 * txt_size, y * Y_CHAR1 * txt_size, 1);// Print the string name of the font
*/

/*
  tft.setFreeFont(FONT2);                 // Select the font
  txt_size = 1;
  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.setTextSize(txt_size);  //size 1 = 10 pixels, size 2 =20 pixels, and so on

  
  x = 0;
  y = 6;
  sprintf(vet_char, "%dx%d x=%d y=%d", display_WIDTH/(X_CHAR2 * txt_size), display_HEIGHT/(Y_CHAR2 * txt_size), x, y);
  //sprintf(vet_char, "x=%d y=%d",x ,y);
  //tft.setCursor(x * X_CHAR2 * txt_size, y * Y_CHAR2 * txt_size);
  //tft.println(vet_char);
  tft.drawString(vet_char, x * X_CHAR2 * txt_size, y * Y_CHAR2 * txt_size, 1);// Print the string name of the font
  
  x = 5;
  y = 7;
  sprintf(vet_char, "x=%d  y=%d",x ,y);
  //tft.setCursor(x * X_CHAR2 * txt_size, y * Y_CHAR2 * txt_size);
  //tft.println(vet_char);
  tft.drawString(vet_char, x * X_CHAR2 * txt_size, y * Y_CHAR2 * txt_size, 1);// Print the string name of the font
*/
/*
  x = 14;
  y = 10;
  sprintf(vet_char, "x=%d y=%d",x ,y);
  //tft.setCursor(x * X_CHAR2 * txt_size, y * Y_CHAR2 * txt_size);
  //tft.println(vet_char);
  tft.drawString(vet_char, x * X_CHAR2 * txt_size, y * Y_CHAR2 * txt_size, 1);// Print the string name of the font
*/


/*

//  tft.startWrite();  //CS on for SPI

  tft.drawFastHLine (0, Y_MIN_DRAW, display_WIDTH, TFT_WHITE);
  for(x=0; x<=((display_WIDTH/(X_CHAR2 * SIZE2))*X_CHAR2); x+=X_CHAR2)
  {
    tft.drawFastVLine (x, Y_MIN_DRAW, 10, TFT_WHITE);
    //x+=X_CHAR2;
  }



  for(y=0; y<=((display_HEIGHT/(Y_CHAR2 * SIZE2))*Y_CHAR2); y+=Y_CHAR2)
  {
    tft.drawFastHLine (0, y, 10, TFT_WHITE);
    //y+=Y_CHAR2;
  }


  for(x=0; x<80; x++)
  {
    y = x/2;
    tft.drawPixel(x, y + Y_MIN_DRAW, TFT_RED); 
    tft.drawPixel(x, y + Y_MIN_DRAW + 1, TFT_RED); 
    tft.drawPixel(x, y + Y_MIN_DRAW + 2, TFT_RED); 
  }

//  tft.endWrite();  //CS off for SPI


*/



  //draw Smeter first block
  tft.fillRect((Smeter_X + ((Smeter_dX + Smeter_dX_space) * 0)), Smeter_Y, Smeter_dX, Smeter_dY, Smeter_table_color[0]);


   
} // main




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





void display_tft_loop(void) 
{
  static uint32_t hmi_freq_fft;

  if (tx_enabled == false)  //waterfall only during RX
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
