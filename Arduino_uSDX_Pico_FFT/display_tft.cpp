/*
  display.cpp

*/

#include "Arduino.h"
#include "SPI.h"
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




uint16_t tft_color565(uint16_t r, uint16_t g, uint16_t b)
{
  return tft.color565(r, g, b);
}








//uint8_t display_graf_new = 0;
uint8_t vet_graf_fft[GRAPH_NUM_LINES][FFT_NSAMP];    // [NL][NCOL]
//uint16_t vet_graf_fft_pos = 0;
uint32_t freq_graf_ini;
uint32_t freq_graf_fim;

/*********************************************************
  
*********************************************************/
void display_graf(void) 
{
  int16_t x, y;
/*
 step 1000Hz
 FFT_NSAMP = 160
 FFT results 80 freqs of 1kHz

 step 500Hz
 FFT_NSAMP = 320
 FFT results 160 freqs of 500Hz

 Display ILI9341  240 x 320
 show 160+160 freqs with 1 pixels per freq
 inicialmente 
 plota o grafico com a freq de sintonia fixa no centro do display
 se a freq de sintonia muda, vai deslocando/entortando as linhas no grafico

*/


// escrever a freq inicial e a final
// colocar uma marca na posicao da freq de sintonia
// desenhar os pontos da matriz de FFT
// subir 1 linha na matriz para preparar lugar para prox FFT




  freq_graf_ini = (hmi_freq - ((FFT_NSAMP/2)*FRES) )/1000;
  sprintf(vet_char, "%d ", freq_graf_ini);
  //tft.setTextSize(2);  //size 1 = 10 pixels, size 2 =20 pixels, and so on
  //tft.setCursor(0, 10);
  //tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  //tft.println(vet_char);
  tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,0,7,(uint8_t *)vet_char);  

  freq_graf_fim = (hmi_freq + ((FFT_NSAMP/2)*FRES) )/1000;
  sprintf(vet_char, "%5d", freq_graf_fim);
  //tft.setTextSize(2);  //size 1 = 10 pixels, size 2 =20 pixels, and so on
  //tft.setCursor(20, 10);
  //tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  //tft.println(vet_char);
  tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,18,7,(uint8_t *)vet_char);  


  switch(dsp_getmode())  //{"USB","LSB","AM","CW"}
  //switch(2)  //{"USB","LSB","AM","CW"}
  {
    case 0:  //USB
      //tft.drawFastVLine (display_WIDTH/2, Y_MIN_DRAW - 21, 10, TFT_YELLOW);
      //tft.drawLine (display_WIDTH/2, Y_MIN_DRAW - 21, (display_WIDTH/2) - 8, Y_MIN_DRAW - 11,TFT_YELLOW);
      tft.fillTriangle(display_WIDTH/2, Y_MIN_DRAW - 12, display_WIDTH/2, Y_MIN_DRAW - 30, (display_WIDTH/2)+8, Y_MIN_DRAW - 12, TFT_YELLOW);
      break;
    case 1:  //LSB
      tft.fillTriangle(display_WIDTH/2, Y_MIN_DRAW - 12, display_WIDTH/2, Y_MIN_DRAW - 30, (display_WIDTH/2)-8, Y_MIN_DRAW - 12, TFT_YELLOW);
      break;
    case 2:  //AM
      tft.fillTriangle((display_WIDTH/2)-8, Y_MIN_DRAW - 12, display_WIDTH/2, Y_MIN_DRAW - 30, (display_WIDTH/2)+8, Y_MIN_DRAW - 12, TFT_YELLOW);
      break;
    case 3:  //CW
      tft.fillTriangle((display_WIDTH/2)-8, Y_MIN_DRAW - 12, display_WIDTH/2, Y_MIN_DRAW - 30, (display_WIDTH/2)+8, Y_MIN_DRAW - 12, TFT_YELLOW);
      break;
  }


  
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






   //vet_graf_fft[(GRAPH_NUM_LINES-1)][30] = 1;          
   //vet_graf_fft[(GRAPH_NUM_LINES-2)][50] = 1;          
   //vet_graf_fft[(GRAPH_NUM_LINES-1)][22] = 0;          

   //vet_graf_fft[4][20] = 0;          
   //vet_graf_fft[4][21] = 0;          
   //vet_graf_fft[4][22] = 0;          

  //vet_graf_fft[GRAPH_NUM_LINES][GRAPH_NUM_COLS]   [NL][NCOL]
  for(y=0; y<GRAPH_NUM_LINES; y++)
  {
    for(x=0; x<GRAPH_NUM_COLS; x++)
    {
//      if((x ==25) && (y== (GRAPH_NUM_LINES-1))) vet_graf_fft[y][x] = 1;
      
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

  for(y=0; y<(GRAPH_NUM_LINES-1); y++)
  {
    for(x=0; x<GRAPH_NUM_COLS; x++)
    {
      //move graph data one line up to open space for next FFT (= last line)
      vet_graf_fft[y][x] = vet_graf_fft[y+1][x];
    }
  }


  
#if 0
  
  int y;
  //static int num;
  static uint16_t freq_graf_min_old = 1;
  static uint16_t freq_graf_max_old = 0;
  static uint32_t val_old = 0;

  tft.startWrite();  //CS on for SPI

  if (val_old != val)
    {
    val_old = val;
    //tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
    //tft.setCursor(0, 0);
    //tft.println("                      ");
    //tft.fillRect(0,0,240,20,TFT_BLACK); //tft.fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
    sprintf(vet_char, "cap us %d          ", val);
    //oledWriteString(&oled, 0,0,0,vet_char, FONT_6x8, 0, 0);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
    tft.setTextSize(2);  //size 1 = 10 pixels, size 2 =20 pixels, and so on
    tft.println(vet_char);
    }

  if(freq_graf_min_old != freq_graf_min)
    {
    freq_graf_min_old = freq_graf_min;
    //tft.setCursor(0, 20);
    //tft.println("        ");
    //tft.fillRect(0,20,120,20,TFT_BLACK); //tft.fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
    sprintf(vet_char, "%dHz", freq_graf_min);
    //oledWriteString(&oled, 0,0,1,vet_char, FONT_6x8, 0, 0);
    tft.setCursor(0, 20);
    tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
    tft.setTextSize(2);  //size 1 = 10 pixels, size 2 =20 pixels, and so on
    tft.println(vet_char);
    }

  if(freq_graf_max_old != freq_graf_max)
    {
    freq_graf_max_old = freq_graf_max;
    //tft.setCursor((display_WIDTH-(12*5)), 20);
    //tft.println("   ");
    //tft.fillRect(120,20,120,20,TFT_BLACK); //tft.fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
    sprintf(vet_char, "%dHz", freq_graf_max);
    //oledWriteString(&oled, 0,(OLED_WIDTH-(6*strlen(vet_char))),1,vet_char, FONT_6x8, 0, 0);
    tft.setCursor((display_WIDTH-(12*strlen(vet_char))), 20);
    tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
    tft.setTextSize(2);  //size 1 = 10 pixels, size 2 =20 pixels, and so on
    tft.println(vet_char);
    }

// scroll the table up
  for(y=1; y<GRAPH_NUM_LINES; y++)
  {
    tft.drawFastHLine (0, (y-1) + Y_MIN_DRAW, display_WIDTH, TFT_BLACK);

    for(int x=0; x<GRAPH_NUM_COLS; x++)
      {
          graf[x][y-1] = graf[x][y];
          //oledSetPixel(&oled, x, (y-1) + Y_MIN_DRAW, graf[x][y], 0);
          //tft.drawPixel(x, (y-1) + Y_MIN_DRAW, tft.color565(x<<3, y<<3, x*y));
          if(graf[x][y] > 0)
            {
              tft.drawPixel(x, (y-1) + Y_MIN_DRAW, ColorGraf(graf[x][y-1]));
            }
      }
  }

  tft.drawFastHLine (0, (GRAPH_NUM_LINES-1) + Y_MIN_DRAW, display_WIDTH, TFT_BLACK);
  for(int x=0; x<GRAPH_NUM_COLS; x++)
    {
        graf[x][GRAPH_NUM_LINES-1] = vet_graf_fft[x];
/*        
        if(x > 127)
          {
          graf[x][GRAPH_NUM_LINES-1] = 0;
          }
        else
          {
          graf[x][GRAPH_NUM_LINES-1] = x*2;
          }
*/
        //if(x == num)  graf[x][GRAPH_NUM_LINES-1] = 1;  else   graf[x][GRAPH_NUM_LINES-1] = 0; 
        //oledSetPixel(&oled, x, (GRAPH_NUM_LINES-1) + Y_MIN_DRAW, graf[x][NUM_LINES-1], 0);
        if(graf[x][GRAPH_NUM_LINES-1] > 0)
          {        
            tft.drawPixel(x, (GRAPH_NUM_LINES-1) + Y_MIN_DRAW, ColorGraf(graf[x][GRAPH_NUM_LINES-1]));
          }
    }

  tft.endWrite();  //CS off for SPI

#endif

}























/*********************************************************
  
*********************************************************/
void display_tft_setup(void) {

uint16_t x, y;

  tft.init();
  tft.setRotation(ROTATION_SETUP);
 
  tft.fillScreen(TFT_BLACK);
  

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
