/*
 * display.cpp
 * 
 * Created: May 2022
 * Author: Klaus Fensterseifer
 * https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj)
 * 
*/

#include "Arduino.h"
#include "SPI.h"
//#include "uSDR.h"
#include "dsp.h"
#include "display_tft.h"
#include "TFT_eSPI.h"
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
/*
void tft_setup(void)
{
  tft.setFreeFont(FONT1);                 // Select the font
  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.setTextSize(SIZE1);  //size 1 = 10 pixels, size 2 =20 pixels, and so on
}


void tft_writexy(uint8_t x, uint8_t y, uint8_t *s)
{
  tft.drawString((const char *)s, x * X_CHAR1 * SIZE1, y * Y_CHAR1 * SIZE1, 1);// Print the string name of the font
}
*/

void tft_writexy_(uint16_t font, uint16_t color, uint16_t color_back, uint8_t x, uint8_t y, uint8_t *s)
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



void tft_writexy_2(uint16_t font, uint16_t color, uint16_t color_back, uint8_t x, uint8_t x_plus, uint8_t y, uint8_t y_plus, uint8_t *s)
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


void tft_cursor_2(uint16_t font, uint16_t color, uint8_t x, uint8_t x_plus, uint8_t y, uint8_t y_plus)
{
    if(font == 3)
    {
      for(uint16_t i=1; i<((Y_CHAR3 * SIZE3)/8); i++)
      {
      tft.drawFastHLine ((x * X_CHAR3 * SIZE3)+x_plus, (((y+1) * Y_CHAR3 * SIZE3) - i)+y_plus , ((X_CHAR3 * SIZE3)*9)/10, color);
      }
    }
    else if (font == 2)
    {
      for(uint16_t i=1; i<((Y_CHAR2 * SIZE2)/8); i++)
      {
      tft.drawFastHLine ((x * X_CHAR2 * SIZE2)+x_plus, (((y+1) * Y_CHAR2 * SIZE2) - i)+y_plus , X_CHAR2 * SIZE2, color);
      }
    }
    else
    {
      for(uint16_t i=1; i<((Y_CHAR1 * SIZE1)/8); i++)
      {
      tft.drawFastHLine ((x * X_CHAR1 * SIZE1)+x_plus, (((y+1) * Y_CHAR1 * SIZE1) - i)+y_plus , X_CHAR1 * SIZE1, color);
      }
    }
}




uint16_t tft_color565(uint16_t r, uint16_t g, uint16_t b)
{
  return tft.color565(r, g, b);
}








//uint8_t fft_display_graf_new = 0;
uint8_t vet_graf_fft[GRAPH_NUM_LINES][FFT_NSAMP];    // [NL][NCOL]
//uint16_t vet_graf_fft_pos = 0;
uint32_t freq_graf_ini;
uint32_t freq_graf_fim;
/*********************************************************
  
*********************************************************/
void display_fft_graf(int16_t change) 
{
  int16_t x, y;


  //plot data and scale only when something changes
  if(change != 0)
  {
   
    //graph min freq
    freq_graf_ini = (hmi_freq - ((FFT_NSAMP/2)*FRES) )/1000;
    sprintf(vet_char, "%d ", freq_graf_ini);
    tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,0,7,(uint8_t *)vet_char);  
  
    //graph max freq
    freq_graf_fim = (hmi_freq + ((FFT_NSAMP/2)*FRES) )/1000;
    sprintf(vet_char, "%5d", freq_graf_fim);
    tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,18,7,(uint8_t *)vet_char);  
  
   
    //little triangle indicating the center freq
    switch(dsp_getmode())  //{"USB","LSB","AM","CW"}
    //switch(2)  //{"USB","LSB","AM","CW"}
    {
      case 0:  //USB
        tft.fillTriangle(display_WIDTH/2, Y_MIN_DRAW - 12, display_WIDTH/2, Y_MIN_DRAW - 30, (display_WIDTH/2)+8, Y_MIN_DRAW - 12, TFT_YELLOW);
        tft.fillTriangle((display_WIDTH/2)-1, Y_MIN_DRAW - 12, display_WIDTH/2, Y_MIN_DRAW - 30, (display_WIDTH/2)-8, Y_MIN_DRAW - 12, TFT_BLACK);
        break;
      case 1:  //LSB
        tft.fillTriangle(display_WIDTH/2, Y_MIN_DRAW - 12, display_WIDTH/2, Y_MIN_DRAW - 30, (display_WIDTH/2)-8, Y_MIN_DRAW - 12, TFT_YELLOW);
        tft.fillTriangle((display_WIDTH/2)+1, Y_MIN_DRAW - 12, display_WIDTH/2, Y_MIN_DRAW - 30, (display_WIDTH/2)+8, Y_MIN_DRAW - 12, TFT_BLACK);
        break;
      case 2:  //AM
        tft.fillTriangle((display_WIDTH/2)-8, Y_MIN_DRAW - 12, display_WIDTH/2, Y_MIN_DRAW - 30, (display_WIDTH/2)+8, Y_MIN_DRAW - 12, TFT_YELLOW);
        break;
      case 3:  //CW
        tft.fillTriangle((display_WIDTH/2)-8, Y_MIN_DRAW - 12, display_WIDTH/2, Y_MIN_DRAW - 30, (display_WIDTH/2)+8, Y_MIN_DRAW - 12, TFT_YELLOW);
        break;
    }

 
    //plot scale on top of waterfall
    tft.drawFastHLine (0, Y_MIN_DRAW - 11, display_WIDTH, TFT_WHITE);
    tft.fillRect(0, Y_MIN_DRAW - 10, display_WIDTH, 10, TFT_BLACK);
    x=0;
    for(; freq_graf_ini < freq_graf_fim; freq_graf_ini+=1)
    {
      if((freq_graf_ini % 10) == 0)
      {
        tft.drawFastVLine (x, Y_MIN_DRAW - 11, 5, TFT_WHITE);
      }
      if((freq_graf_ini % 100) == 0)
      {
        tft.drawFastVLine (x, Y_MIN_DRAW - 11, 10, TFT_WHITE);
      }
      x+=2;
    }
    
  }

        
  //plot waterfall
  //vet_graf_fft[GRAPH_NUM_LINES][GRAPH_NUM_COLS]   [NL][NCOL]
  for(y=0; y<GRAPH_NUM_LINES; y++)
  {
    for(x=0; x<GRAPH_NUM_COLS; x++)
    {
      //plot graph one line
      if(vet_graf_fft[y][x] > 0)
      {
        tft.drawPixel(x, (y + Y_MIN_DRAW), TFT_WHITE);        
      }
      else
      {
        tft.drawPixel(x, (y + Y_MIN_DRAW), TFT_BLACK); 
      }
    }
  }
  

  //move graph data one line up to open space for next FFT (= last line)
  for(y=0; y<(GRAPH_NUM_LINES-1); y++)
  {
    for(x=0; x<GRAPH_NUM_COLS; x++)
    {
      vet_graf_fft[y][x] = vet_graf_fft[y+1][x];
    }
  }


 
}






void display_aud_graf(uint16_t aud_pos, uint16_t aud_var, uint16_t color)
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

//#if 0
  
  //erase graphic area
  //tft.fillRect(0, Y_MIN_DRAW - 10, display_WIDTH, 10, TFT_BLACK);
  tft.fillRect(X_MIN_AUD_GRAPH, Y_MIN_AUD_GRAPH, AUD_GRAPH_NUM_COLS, (AUD_GRAPH_MAX - AUD_GRAPH_MIN + 1), TFT_BLACK);

  //find trigger point to start ploting
  for(x=0; x<AUD_GRAPH_NUM_COLS; x++)
  {
     if((aud_samp[AUD_SAMP_I][x+0] > 0) &&
        (aud_samp[AUD_SAMP_I][x+1] > 0) &&
        (aud_samp[AUD_SAMP_I][x+2] > 0) &&
        (aud_samp[AUD_SAMP_I][x+3] > 0) &&
        (aud_samp[AUD_SAMP_I][x+4] > 0))
        {
          break;
        }
  }
  for(; x<AUD_GRAPH_NUM_COLS; x++)
  {
     if(aud_samp[AUD_SAMP_I][x] < 0)
        {
          break;
        }
  }
  aud_pos = x;

  //plot each variable
  display_aud_graf(aud_pos, AUD_SAMP_I, TFT_GREEN);
  display_aud_graf(aud_pos, AUD_SAMP_Q, TFT_CYAN);
  display_aud_graf(aud_pos, AUD_SAMP_MIC, TFT_RED);
  display_aud_graf(aud_pos, AUD_SAMP_A, TFT_PINK);
  display_aud_graf(aud_pos, AUD_SAMP_PEAK, TFT_YELLOW);
  display_aud_graf(aud_pos, AUD_SAMP_GAIN, TFT_MAGENTA);

//#endif

}












/*********************************************************
  
*********************************************************/
void display_tft_setup(void) {

uint16_t x, y;
char s[32];
  
  tft.init();
  tft.setRotation(ROTATION_SETUP);
 
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
  tft_writexy_(1, TFT_GREEN, TFT_BLACK, 8,1,(uint8_t *)s);
  sprintf(s, "Q");
  tft_writexy_(1, TFT_CYAN, TFT_BLACK, 8,2,(uint8_t *)s);
  sprintf(s, "A");
  tft_writexy_(1, TFT_PINK, TFT_BLACK, 8,3,(uint8_t *)s);
  sprintf(s, "MIC");
  tft_writexy_(1, TFT_RED, TFT_BLACK, 10,1,(uint8_t *)s);
  sprintf(s, "PEAK");
  tft_writexy_(1, TFT_YELLOW, TFT_BLACK, 10,2,(uint8_t *)s);
  sprintf(s, "GAIN");
  tft_writexy_(1, TFT_MAGENTA, TFT_BLACK, 10,3,(uint8_t *)s);



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
   
} // main





void display_tft_loop(void) {



}
