/*
 * hmi.c
 *
 * Created: Oct 2024
 * Author: Klaus Fensterseifer 
 * https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj
 * 
 * This file contains the 
 * It will also do the logic 
 *
 *
 */

#include "Arduino.h"
#include "uSDR.h"
//#include "relay.h"
//#include "si5351.h"
#include "dsp.h"
//#include "hmi.h"
//#include "dsp.h"
//#include "pico/multicore.h"
//#include "SPI.h"
#include "TFT_eSPI.h"
#include "display_tft.h"
#include "CwDecoder.h"






struct st_tabcw
  {
  uint16_t nbit;  /* numero de pontos e tracos do dado */
  uint16_t cod;   /* pontos bit = 0  e tracos bit = 1 */
  } tabcw[128] = {0};

// reference to the morse cw table tabcw[],  grouped by number os dots and dashs
#define TABCW1  2
uint16_t tabcw1[TABCW1] = { 'E', 'T' };
#define TABCW2  4
uint16_t tabcw2[TABCW2] = { 'A', 'I', 'M', 'N' };
#define TABCW3  8
uint16_t tabcw3[TABCW3] = { 'K', 'U', 'S', 'D', 'G', 'O', 'R', 'W' };
#define TABCW4  12
uint16_t tabcw4[TABCW4] = { 'L', 'F', 'B', 'C', 'H', 'J', 'P', 'Q', 'V', 'X', 'Y', 'Z' };
#define TABCW5  12
uint16_t tabcw5[TABCW5] = { '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '=', '/' };
#define TABCW6  7
uint16_t tabcw6[TABCW6] = { '.', ',', ';', ':', '-', '\'', '?' };
#define TABCW8   1 
uint16_t tabcw8[TABCW8] = { '*' };





uint16_t cw_rx[MAX_CW_RX_INDEX][2];
uint16_t cw_rx_array;
uint16_t cw_rx_index;
uint32_t cw_rx_avg;
uint16_t cw_rx_cnt;

// wpm = 1200 / tponto ms 
//#define TDOT_MIN      16          // x 2.5ms =  40ms
#define TDOT_MIN       4         // x 2.5ms =  10ms   120wpm
#define TDOT_MAX      96         // x 2.5ms = 240ms     5wpm
#define TDOT_SHIFT     1         // tdot accumulate 4x
// wpm = 1200 / tponto ms   (time base for reading the cw audio is 2.5ms)
// 60  =  20ms   / 2.5ms =  8 counts
// 30  =  40ms   / 2.5ms = 16
// 20  =  60ms   / 2.5ms = 24
// 15  =  80ms   / 2.5ms = 32
// 10  = 120ms   / 2.5ms = 48
//  5  = 240ms   / 2.5ms = 96
uint16_t tdot;
uint16_t tdot_acc;
uint16_t limiar_min_dot;    // 1/2 dot
uint16_t limiar_min_dash;  //2x_dot
uint16_t limiar_min_space;    //5x dot
uint16_t limiar_space;    //7x dot
uint16_t cw_rx_limiar;
uint16_t count_low;
uint16_t count_high;
uint16_t cw_level;

#define CW_RX_LIMIAR_INIC   4
//#define CW_RX_LIMIAR_MIN   3
//#define CW_RX_LIMIAR_MAX  10


uint16_t cw_letter_pos;
uint16_t cw_letter;

#define SCW_MAX   15
//                     "123456789012345"
char scw[SCW_MAX+1] =  "123456789012345";    //chars received to print to the screen [0] to [29]  + [30] = 0;
char s1[32];   //aux          


#define CWN   1000
int8_t cwn[CWN][2];
int16_t cwnin = 0;
int16_t cwnout = 0;

//#define cwin(v,u)  { if(++cwnin>=CWN) cwnin = 0; cwn[cwnin] = v; res[cwnin] = u; }

/**************************************************************************************
    
**************************************************************************************/
uint16_t prox(uint16_t p)
{
  if(++p >= CWN)
    p = 0;
  return p;
}


/**************************************************************************************
    
**************************************************************************************/
void cw_in(int8_t v, int8_t u)
{
  cwnin = prox(cwnin);  //next position to fill
  cwn[cwnin][0] = v;
  cwn[cwnin][1] = u;
  if(cwnin == cwnout)   //list is full
  {
    cwnout = prox(cwnout);  //move the end one step ahead
  } 
}

/**************************************************************************************
    
**************************************************************************************/
bool cw_out(int8_t *v, int8_t *u)
{
  if(cwnout != cwnin)  //not empty
  {
    cwnout = prox(cwnout);
    *v = cwn[cwnout][0];
    *u = cwn[cwnout][1];
    return true;
  }
  else
  {
  return false;
  }
}



/**************************************************************************************
    
**************************************************************************************/
void CwCalcTime()
{
  //tdash = 3 * tdot;
  //spchar = 3 * tdot;
  //spword = 7 * tdot;

  // dot = 1
  // dash = 3
  // space dot-dash = 1
  // space letter = 3
  // space word = 7

  tdot = tdot_acc>>TDOT_SHIFT;  //tdot_acc needs 16x up to chanag 1 in tdot

  limiar_min_dot = tdot>>1;       // 1/2 dot = min time for dot and space  less than this = noise or high wpm
  limiar_min_dash = 2 * tdot;  //2x_dot
  limiar_min_space = 5 * tdot;    //5x dot
  limiar_space = 7 * tdot;    //7x dot

  sprintf(s1, "%d_", tdot);
  tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 1, 0, 3, 20, (uint8_t *)s1);   
}


/**************************************************************************************
    
**************************************************************************************/
void  to_scw(char c)
{
  uint16_t i;
  for(i=0; i<SCW_MAX-1; i++)
    scw[i] = scw[i+1];
  scw[i] = c; 
}



/**************************************************************************************
    
**************************************************************************************/
void  to_display(char c)
{
  uint16_t i;
  for(i=0; i<SCW_MAX-1; i++)
    scw[i] = scw[i+1];
  scw[i] = c; 
  tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 8, 0, 3, 20, (uint8_t *)scw);   
}



#define SCWGRAPH_X    0
#define SCWGRAPH_Y    10
#define SCWGRAPH_H    16

#define SCWGRAPH_MAX   300     //TFT_WIDTH=320    must be < 320-4  (there is an erase width=4 below)
uint8_t scwgraph[SCW_MAX+1] =  {0};    //cw audio level received to print as graphic
uint16_t scwgraph_pos = 0;

// 10ms dot  @  16kHz   = 160 samples     /16 to show on graph

/**************************************************************************************
    
**************************************************************************************/
void  to_graph(uint16_t v)
{
  //graphic with 16 pixels for level
  
  static uint16_t accu = 0;
  static uint16_t cont = 0;
  static uint16_t cont_low = 0;

  accu += v;
  if(++cont>=2)
  {
    v = accu>>1;
    if(v < 4) 
      cont_low++;
    else
      cont_low = 0;
    accu = 0;
    cont = 0;

    if(cont_low > 20) //if level low for some time, stop graph
    {
      cont_low = 22;
    }
    else
    {
      if(v>0)
      {
        v = (v-1);  //>>1;
        if(v>(SCWGRAPH_H-1)) v = (SCWGRAPH_H-1);
      }
      scwgraph[scwgraph_pos] = (uint8_t)v;
      if((scwgraph_pos+1) == SCWGRAPH_MAX)
        scwgraph_pos = 0;
      else
        scwgraph_pos++;
    }
  }
}



/**************************************************************************************
    
**************************************************************************************/
void  graph_to_display(void)
{
  static uint16_t scwgraph_pos_old;
  uint16_t scwgraph_pos_loc;
  uint16_t i;

  scwgraph_pos_loc = scwgraph_pos;  //local value    avoid mix with interrupt on CORE1
  while(scwgraph_pos_loc != scwgraph_pos_old)
  {
    //tft.fillRect((SCWGRAPH_X+scwgraph_pos_old), SCWGRAPH_Y, 4, SCWGRAPH_H, TFT_BLACK);  //erase old area
    tft.drawFastVLine ((SCWGRAPH_X+scwgraph_pos_old), SCWGRAPH_Y+1, SCWGRAPH_H, TFT_BLACK);
    tft.drawPixel((SCWGRAPH_X+scwgraph_pos_old), ((SCWGRAPH_Y+SCWGRAPH_H) - scwgraph[scwgraph_pos_old]), TFT_WHITE); 
    if(++scwgraph_pos_old >= SCWGRAPH_MAX)
      scwgraph_pos_old = 0;
  }
  tft.fillRect((SCWGRAPH_X+scwgraph_pos_old), SCWGRAPH_Y+1, 4, SCWGRAPH_H, TFT_BLACK);  //erase old area
}



/**************************************************************************************
    
**************************************************************************************/
void CwDecoder_Inic(void)
{
  tdot_acc = 10<<TDOT_SHIFT;   //  tdot = tdot_acc>>TDOT_SHIFT;     // 32 = 15wpm  cw dot time (time base for other parameters)
  CwCalcTime();

  //tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 8, 0, 3, 20, (uint8_t *)scw);   
  //to_display('K');


  cw_rx_limiar = CW_RX_LIMIAR_INIC;
  count_low = 0;
  count_high = 0;
  cw_level = 0;
  cw_letter_pos = 0;
  cw_letter = 0;
}




/**************************************************************************************
    
**************************************************************************************/
void CwDecoder_Exit(void)
{

  uint16_t i = 0;
  int8_t a, b;

  Serialx.println("*** CW Decoder ***");
  while(cw_out(&a, &b))
  {
    sprintf(s1, "%d %d %d", i++, a, b);
    Serialx.println(s1);
  }


  //clear the cw text line
  tft.fillRect(1, ((3*Y_CHAR1)+20), (SCW_MAX+8)*X_CHAR1, Y_CHAR1-2, TFT_BLACK);
/*
#define X_CHAR1  14
#define Y_CHAR1  22
  tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 1, 0, 3, 20, (uint8_t *)"                ");   
*/
}




/**************************************************************************************
    
**************************************************************************************/
void new_dot(void)
{
  //include the dot on the letter received
  //including 0 = dot
  cw_letter_pos++;   //dot is 0   and cw_letter is already 0
  //use the counter_high to adjust the tdot
  //to_display('.');

  sprintf(s1, "%d_", count_high);
  tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 4, 0, 3, 20, (uint8_t *)s1); 
}


/**************************************************************************************
    
**************************************************************************************/
void new_dash(void)
{
  //include the dash on the letter received
  cw_letter |= (0x8000>>cw_letter_pos); //including 1 = dash
  cw_letter_pos++;
  //use the counter_high to adjust the tdot->tdash
  //to_display('-');
}


/**************************************************************************************
    
**************************************************************************************/
void new_space_dot_dash(void)
{
  //ok,  do nothing     next dot or dash is comming
  //use the counter_low to adjust the tdot->limiar_min_space
}



char cw_search_letter(uint16_t num, uint16_t *tab)
{
  uint16_t i;
  char ret;

  for(i=0; i<num; i++)
  {
    if(cw_letter == tabcw[tab[i]].cod)
      break;
  }
  if(i<num)
  {
    ret = tab[i];
  }
  else
  {
    ret = '#';  //not find
  }
  return ret;
}



/**************************************************************************************
    new space between letters
**************************************************************************************/
void new_space_letters(void)
{
/*
  //if last letter on the word is not a space
  if(scw[SCW_MAX-1] != ' ')
  {
    //   insert a space on the word
    to_display(' '); 
  }
*/
  //finished the letter
  //  look for the letter on the table  and show on display
  switch(cw_letter_pos)
  {
    case 1:
        to_display(cw_search_letter(TABCW1, tabcw1));
      break;
    case 2:
        to_display(cw_search_letter(TABCW2, tabcw2));
      break;
    case 3:
        to_display(cw_search_letter(TABCW3, tabcw3));
      break;
    case 4:
        to_display(cw_search_letter(TABCW4, tabcw4));
      break;
    case 5:
        to_display(cw_search_letter(TABCW5, tabcw5));
      break;
    case 6:
        to_display(cw_search_letter(TABCW6, tabcw6));
      break;
    case 8:
        to_display(cw_search_letter(TABCW8, tabcw8));
      break;
    default:
        to_display('#');  // don't know
      break;
  }

  cw_letter = 0;     //clear the dot/dashes for new letter
  cw_letter_pos = 0;   //clear the count of dot/dashes
}


/**************************************************************************************
    
**************************************************************************************/
void new_space_words(void)
{
  //if last letter on the word is not a space
  if(scw[SCW_MAX-1] != ' ')
  {
    //   insert a space on the word
    to_display(' '); 
  }
}

/*
#define LEV_SHORT   0
#define LEV_OK      1
#define LEV_LONG    2

uint16_t lev_high = LEV_OK;
uint16_t lev_low = LEV_OK;
*/

/**************************************************************************************
    
**************************************************************************************/
/*
void wpm_adjust(void)
{
//  if((lev_low == LEV_SHORT) && (lev_high == LEV_SHORT))
  if(lev_high == LEV_SHORT)
  {
    if(tdot > TDOT_MIN)
    {  
      tdot_acc--;
      CwCalcTime();
    }
    lev_high = LEV_OK;
  }

//  if((lev_low == LEV_LONG) && (lev_high == LEV_LONG))
  if(lev_high == LEV_LONG)
  {
    if(tdot < TDOT_MAX)
    {
      tdot_acc++;
      CwCalcTime();
    } 
    lev_high = LEV_OK;
  }
}
*/

/**************************************************************************************
    
**************************************************************************************/
void wpm_up(void)
{
  if(tdot > TDOT_MIN)
  {  
    tdot_acc--;
    CwCalcTime();

    sprintf(s1, "%d_", tdot);
    tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 1, 0, 3, 20, (uint8_t *)s1);   
  }
}

/**************************************************************************************
    
**************************************************************************************/
void wpm_down(void)
{
  if(tdot < TDOT_MAX)
  {
    tdot_acc++;
    CwCalcTime();

    sprintf(s1, "%d_", tdot);
    tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 1, 0, 3, 20, (uint8_t *)s1);   
  }  
}


#define DOT    1
#define DASH   3
#define SPACE_DOT_DASH   1
#define SPACE_LETTERS    3
#define SPACE_WORDS    7

/**************************************************************************************
    
**************************************************************************************/
void wpm_ok(uint16_t count, uint16_t size)
{

  if((size * tdot) > count)  //ok, but will adjust
  {
    tdot_acc--;
    CwCalcTime();
  }
  else if((size * tdot) < count)  //ok, but will adjust
  {
    tdot_acc++;
    CwCalcTime();
  }

/*
  if((tdot_acc>>TDOT_SHIFT) > tdot)  //if tdot is good   try to bring tdot_acc to the correct value
  {
    tdot_acc--;
    tdot = tdot_acc>>TDOT_SHIFT;  //tdot_acc needs 16x up to chanag 1 in tdot
    CwCalcTime();
  }
  else if((tdot_acc>>TDOT_SHIFT) < tdot) 
  {
    tdot_acc++;
    tdot = tdot_acc>>TDOT_SHIFT;  //tdot_acc needs 16x up to chanag 1 in tdot
    CwCalcTime();
  }
*/
}


/**************************************************************************************
    
**************************************************************************************/
void cw_limiar_up(void)
{
}

/**************************************************************************************
    
**************************************************************************************/
void cw_limiar_down(void)
{
}








/**************************************************************************************
    CwDecoder_array_in - uses the cw audio level to search for cw characters
    array received with samples @ 2.5ms
**************************************************************************************/
void CwDecoder_array_in(void)
{
uint16_t i = 0;
static uint16_t cw_rx_array_old = 0;
uint16_t cw_letter_old = 0;

  if(cw_rx_array != cw_rx_array_old)  //new array ready to analysis
  {

    //look the level on the received array
    for(i=0; i<MAX_CW_RX_INDEX; i++)
      {
        //to_graph(cw_rx[i][cw_rx_array_old]);  //put the signal on graphic

        //sprintf(s1, "%d_", cw_rx[i][cw_rx_array_old]);
        //tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 4, 0, 3, 20, (uint8_t *)s1); 

        if(cw_rx[i][cw_rx_array_old] > cw_rx_limiar)
          {
            // *******************  input signal level = high  *******************

            if(cw_level == 1)  //if it was already high
              {
                count_high++;
                if(count_high > limiar_space)        //7x dot = tone for so long (rf carrier)
                {
                  count_high = limiar_space;      //7x dot = just to limit the counter
                }
              }
            else  // cw_level == 0   was low and changed to high (look for spaces)
              {
                // ********************************************************************* 
                // *************************** level raising ***************************
                // ********************************************************************* 

                cw_in(-count_low, tdot);  //make a list of info to send through serial when out of CW mode

                if(count_low < limiar_min_dot)   // 1/2 dot = noise ?   faster wpm ?
                {
                  //possible noise, but counts as space between dots and dashs  
                  new_space_dot_dash();
                  //wpm_up();    //lev_low = LEV_SHORT;
                  //wpm_up(0);   //increase wpm
                }
                else if(count_low < limiar_min_dash)   //2x_dot
                {
                  new_space_dot_dash();
                  //wpm_ok(count_low, SPACE_DOT_DASH);    //lev_low = LEV_OK;
                }
                else if(count_low < limiar_min_space)        //5x dot  = space between letter
                {
                  new_space_letters();
                  //wpm_ok(count_low, SPACE_LETTERS);    //lev_low = LEV_OK;
                }
                else  //big space
                {
                  new_space_words();
                  //cw_limiar_down();  //decrease limiar to detect level high (no high level detected)
                  //wpm_ok(count_low, SPACE_WORDS);    //lev_low = LEV_OK;
                }

                //cw_in(-count_low, cw_letter);  //make a list of info to send through serial when out of CW mode


                //wpm_adjust();

              }
            count_low = 0;
            cw_level = 1;   //receiving the high level
          }
        else
          {
            // *******************  input signal level = low  *******************

            if(cw_level == 0)  //if it was already low
              {
                count_low++;
                if(count_low == limiar_min_space)         //5x dot = space between words (no more signal)
                {
                  new_space_letters();
                  count_low = limiar_space;      //7x dot = just to limit the counter
                }
/*
                if(count_low > limiar_space)        //7x dot =  space between words (no more signal)
                {
                  new_space_words();
                  count_low = limiar_space;      //7x dot = just to limit the counter
                }
*/
              }
            else  // cw_level == 1   was high and changed to low (look for dots and dashs)
              { 
                // *********************************************************************
                // *************************** level falling ***************************
                // *********************************************************************

                //to_scw((count_high / tdot) + '0');
/*
                if(count_high < limiar_min_dot)   // 1/2 dot = noise ?   faster wpm ?
                {
                  //ignores this level - noise?
                  count_low++;
                  lev_high = LEV_SHORT;
                  //wpm_up(1);  //increase wpm
                  cw_limiar_up();  //increase limiar to detect level high (getting spikes os sound)
                }
                else
*/
                  
                //cw_in(count_high, tdot);  //make a list of info to send through serial when out of CW mode
                
                if(count_high < limiar_min_dot)   // 1/2 dot = noise ?   faster wpm ?
                {
                  new_dot();
                  wpm_up();    //lev_high = LEV_SHORT;
                }
                if(count_high < limiar_min_dash)    //2x_dot  =  dot time
                {
                  new_dot();
                  wpm_ok(count_high, DOT);    //lev_high = LEV_OK;
                }
                else if(count_high < limiar_min_space)  //5x dot  = dash time
                {
                  new_dash();
                  wpm_ok(count_high, DASH);    //lev_high = LEV_OK;
                }
                else    // noise?  lower wpm ?
                {
                  new_dash();
                  wpm_down();    //lev_high = LEV_LONG;
                  //wpm_down();   //decrease wpm
                }

                cw_in(count_high, cw_letter);  //make a list of info to send through serial when out of CW mode

                //wpm_adjust();

              }
            count_high = 0;
            cw_level = 0;   //receiving the low level
          }
      }
    cw_rx_array_old = cw_rx_array;
  }
}




void CwDecoder_Loop(void)
{ 

  //graph_to_display();

}




/**************************************************************************************
    
**************************************************************************************/
void CwDecoder_InicTable(void)
{


// TABCW1
tabcw['E'].nbit = 1;
tabcw['E'].cod = 0x0000;    /* 00000000B  ;E      */
tabcw['T'].nbit = 1;
tabcw['T'].cod = 0x8000;    /* 10000000B  ;T      */

// TABCW2
tabcw['A'].nbit = 2;
tabcw['A'].cod = 0x4000;    /* 01000000B  ;A      */
tabcw['I'].nbit = 2;
tabcw['I'].cod = 0x0000;    /* 00000000B  ;I      */
tabcw['M'].nbit = 2;
tabcw['M'].cod = 0xc000;    /* 11000000B  ;M      */
tabcw['N'].nbit = 2;
tabcw['N'].cod = 0x8000;    /* 10000000B  ;N      */

// TABCW3
tabcw['K'].nbit = 3;
tabcw['K'].cod = 0xa000;    /* 10100000B  ;K  */
tabcw['U'].nbit = 3;
tabcw['U'].cod = 0x2000;    /* 00100000B  ;U      */
tabcw['S'].nbit = 3;
tabcw['S'].cod = 0x0000;    /* 00000000B  ;S      */
tabcw['D'].nbit = 3;
tabcw['D'].cod = 0x8000;    /* 10000000B  ;D      */
tabcw['G'].nbit = 3;
tabcw['G'].cod = 0xc000;    /* 11000000B  ;G      */
tabcw['O'].nbit = 3;
tabcw['O'].cod = 0xe000;    /* 11100000B  ;O      */
tabcw['R'].nbit = 3;
tabcw['R'].cod = 0x4000;    /* 01000000B  ;R      */
tabcw['W'].nbit = 3;
tabcw['W'].cod = 0x6000;    /* 01100000B  ;W      */

// TABCW4
tabcw['L'].nbit = 4;
tabcw['L'].cod = 0x4000;    /* 01000000B  ;L      */
tabcw['F'].nbit = 4;
tabcw['F'].cod = 0x2000;    /* 00100000B  ;F      */
tabcw['B'].nbit = 4;
tabcw['B'].cod = 0x8000;    /* 10000000B  ;B      */
tabcw['C'].nbit = 4;
tabcw['C'].cod = 0xa000;    /* 10100000B  ;C      */
tabcw['H'].nbit = 4;
tabcw['H'].cod = 0x0000;    /* 00000000B  ;H      */
tabcw['J'].nbit = 4;
tabcw['J'].cod = 0x7000;    /* 01110000B  ;J      */
tabcw['P'].nbit = 4;
tabcw['P'].cod = 0x6000;    /* 01100000B  ;P      */
tabcw['Q'].nbit = 4;
tabcw['Q'].cod = 0xd000;    /* 11010000B  ;Q      */
tabcw['V'].nbit = 4;
tabcw['V'].cod = 0x1000;    /* 00010000B  ;V      */
tabcw['X'].nbit = 4;
tabcw['X'].cod = 0x9000;    /* 10010000B  ;X      */
tabcw['Y'].nbit = 4;
tabcw['Y'].cod = 0xb000;    /* 10110000B  ;Y      */
tabcw['Z'].nbit = 4;
tabcw['Z'].cod = 0xc000;    /* 11000000B  ;Z      */

// TABCW5
tabcw['1'].nbit = 5;
tabcw['1'].cod = 0x7800;    /* 01111000B  ;1      */
tabcw['2'].nbit = 5;
tabcw['2'].cod = 0x3800;    /* 00111000B  ;2      */
tabcw['3'].nbit = 5;
tabcw['3'].cod = 0x1800;    /* 00011000B  ;3      */
tabcw['4'].nbit = 5;
tabcw['4'].cod = 0x0800;    /* 00001000B  ;4      */
tabcw['5'].nbit = 5;
tabcw['5'].cod = 0x0000;    /* 00000000B  ;5      */
tabcw['6'].nbit = 5;
tabcw['6'].cod = 0x8000;    /* 10000000B  ;6      */
tabcw['7'].nbit = 5;
tabcw['7'].cod = 0xc000;    /* 11000000B  ;7      */
tabcw['8'].nbit = 5;
tabcw['8'].cod = 0xe000;    /* 11100000B  ;8      */
tabcw['9'].nbit = 5;
tabcw['9'].cod = 0xf000;    /* 11110000B  ;9      */
tabcw['0'].nbit = 5;
tabcw['0'].cod = 0xf800;    /* 11111000B  ;0      */
tabcw['='].nbit = 5;
tabcw['='].cod = 0x8800;    /* 10001000B  ; =     */
tabcw['/'].nbit = 5;
tabcw['/'].cod = 0x9000;    /* 10010000B  ; /     */

// TABCW5
tabcw['.'].nbit = 6;
tabcw['.'].cod = 0x5400;    /* 01010100B  ; .     */
tabcw[','].nbit = 6;
tabcw[','].cod = 0xcc00;    /* 11001100B  ; ,     */
tabcw[';'].nbit = 6;
tabcw[';'].cod = 0xa800;    /* 10101000B  ; ;     */
tabcw[':'].nbit = 6;
tabcw[':'].cod = 0xe000;    /* 11100000B  ; :     */
tabcw['-'].nbit = 6;
tabcw['-'].cod = 0x8400;    /* 10000100B  ; -     */
tabcw['\''].nbit = 6;
tabcw['\''].cod = 0x4800;   /* 01001000B  ; '     */
tabcw['?'].nbit = 6;
tabcw['?'].cod = 0x3000;    /* 00110000B  ; ?     */  

// TABCW7
tabcw['*'].nbit = 8;
tabcw['*'].cod = 0x0000;    /* 00000000B  ; ERRO  */



//tabcw['E'].nbit = 5;
//tabcw['E'].cod = 0x2000;    /* 00100000B  ; E'    */
//tabcw['~'].nbit = 5;
//tabcw['~'].cod = 0x2800;    /* 00101000B  ; A~O   */
//tabcw['€'].nbit = 5;
//tabcw['€'].cod = 0xa000;    /* 10100000B  ; C,    */
//tabcw['A'].nbit = 4;
//tabcw['A'].cod = 0x5000;    /* 01010000B  ; A'    */



}




#if 0


  if(cw_rx_array != cw_rx_array_old)  //new array ready to analysis
  {
    // shows the cw audio array on display

    //sprintf(s, "%3d %3d %3d %3d %3d ", cw_rx[i][cw_rx_array_old], cw_rx[i+1][cw_rx_array_old], 
    //                                     cw_rx[i+2][cw_rx_array_old], cw_rx[i+3][cw_rx_array_old], cw_rx[i+4][cw_rx_array_old]);
    //tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 1, 0, 3, 20, (uint8_t *)s);   

    //sprintf(s, "%3d", max_a_sample);
    //for(i=0; i<MAX_CW_RX_INDEX; i++)
    for(i=0; i<20; i++)
      {
      scw[i] = ((cw_rx[i][cw_rx_array_old] > CW_RX_LIMIAR) ? '1' : '0');
      }
    scw[i] = 0;
    tft_writexy_plus(1, TFT_LIGHTGREY, TFT_BLACK, 1, 0, 3, 20, (uint8_t *)scw);   

    cw_rx_array_old = cw_rx_array;
  }









  // CW Decoder - save the audio level to analize on hmi.cpp
  if(dsp_mode == MODE_CW)
  {
    cw_rx_avg += avg_a_sample;  //average 40 samples = ((1/16kHz) * 40) = 2.5ms
    if(++cw_rx_cnt >= MAX_CW_RX_CNT)  // 40 samples to average
    {
      cw_rx[cw_rx_index][cw_rx_array] = (cw_rx_avg >> 6); // save the 2.5ms average on array for further analysis  (40 samples / 64 - I don't like to make a division)
      cw_rx_avg = 0;
      cw_rx_cnt = 0;

      if(++cw_rx_index >= MAX_CW_RX_INDEX)  // 40 averages of 2.5ms saved on array = 100ms
      {
        if(cw_rx_array==0)  // 0 or 1   change the array  (one array is to fill, another to analize/show on display)
          { cw_rx_array = 1; }
        else
          { cw_rx_array = 0; }

        cw_rx_index = 0;
      }
    }
  }
#endif





