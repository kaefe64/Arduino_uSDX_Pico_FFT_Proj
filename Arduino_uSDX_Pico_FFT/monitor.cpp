/*
 * monitor.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * May2022: adapted by Klaus Fensterseifer 
 * https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj
 * 
 * Command shell on stdin/stdout.
 * Collects characters and parses commandstring.
 * Additional commands can easily be added.
 */ 
/*
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
*/
#include "Arduino.h"
/*
#include "SPI.h"
#include "TFT_eSPI.h"
//#include "display.h"
//#include "kiss_fftr.h"
//#include "adc_fft.h"
#include "dma.h"
#include "pwm.h"
#include "adc.h"
#include "irq.h"
#include "time.h"
#include "multicore.h"
*/

//#include "lcd.h"
#include "si5351.h"
#include "dsp.h"
#include "relay.h"
#include "monitor.h"
#include "uSDR.h"


#define CR			13
#define LF			10
#define SP			32
#define CMD_LEN		80
#define CMD_ARGS	16

char mon_cmd[CMD_LEN+1];							// Command string buffer
char *argv[CMD_ARGS];								// Argument pointers
int nargs;											// Nr of arguments

typedef struct 
{
	const char *cmdstr;									// Command string
	int   cmdlen;									// Command string length
	void (*cmd)(void);								// Command executive
	const char *cmdsyn;									// Command syntax
	const char *help;										// Command help text
} shell_t;




/*** Initialisation, called at startup ***/
void mon_init()
{
  //  stdio_init_all();								// Initialize Standard IO
	mon_cmd[CMD_LEN] = '\0';						// Termination to be sure
	Serialx.print("\n");
	Serialx.print("=============\n");
	Serialx.print(" uSDR-Pico   \n");
	Serialx.print("  PE1ATM     \n");
	Serialx.print(" 2021, Udjat \n");
	Serialx.print("=============\n");
	Serialx.print("Pico> ");								// prompt
}



/*** ------------------------------------------------------------- ***/
/*** Below the definitions of the shell commands, add where needed ***/
/*** ------------------------------------------------------------- ***/

/* 
 * Dumps a defined range of Si5351 registers 
 */
uint8_t si5351_reg[200];
void mon_si(void)
{
	int base=0, nreg=200, i;

	for (i=0; i<nreg; i++) si5351_reg[i] = 0xaa;
	si_getreg(si5351_reg, (uint8_t)base, (uint8_t)nreg);
	for (i=0; i<nreg; i++) Serialx.print((int)(si5351_reg[i]), HEX);
	Serialx.print("\n");
}


/* 
 * Dumps the entire built-in and programmed characterset on the LCD 
 */
void mon_lt(void)
{
	Serialx.print("Check LCD...");
	//lcd_test();
	Serialx.print("\n");
}



/*
 * Toggles the PTT status, overriding the HW signal
 */
bool ptt = false;
void mon_pt(void)
{
	if (ptt)
	{
		ptt = false;
		Serialx.print("PTT released\n");
	}
	else
	{
		ptt = true;
		Serialx.print("PTT active\n");
	}
	tx_enabled = ptt;
}

/*
 * Relay read or write
 */
void mon_bp(void)
{
	int ret;
	
	if (*argv[1]=='w')
	{
		if (nargs>=2) 
		{
			ret = atoi(argv[2]);
			relay_setband((uint8_t)ret);
		}
	}
	ret = relay_getband();
	if (ret<0)
		Serialx.println("I2C read error");
	else
		Serialx.println(ret, HEX);
}

/*
 * Relay read or write
 */
void mon_rx(void)
{
	int ret;
	
	if (*argv[1]=='w')
	{
		if (nargs>=2) 
		{
			ret = atoi(argv[2]);
			relay_setattn((uint8_t)ret);
		}
	}
	ret = relay_getattn();
	if (ret<0)
		Serialx.println("I2C read error");
	else
		Serialx.println(ret, HEX);
	
}

/*
 * Command shell table, organize the command functions above
 */
#define NCMD	6
shell_t shell[NCMD]=
{
	{"si", 2, &mon_si, "si <start> <nr of reg>", "Dumps Si5351 registers"},
	{"lt", 2, &mon_lt, "lt (no parameters)", "LCD test, dumps characterset on LCD"},
	{"pt", 2, &mon_pt, "pt (no parameters)", "Toggles PTT status"},
	{"bp", 2, &mon_bp, "bp {r|w} <value>", "Read or Write BPF relays"},
	{"rx", 2, &mon_rx, "rx {r|w} <value>", "Read or Write RX relays"}
};



/*** ---------------------------------------- ***/
/*** Commandstring parser and monitor process ***/
/*** ---------------------------------------- ***/

/*
 * Command line parser
 */
void mon_parse(char* s)
{
	char *p;
	int  i;

	p = s;											// Set to start of string
	nargs = 0;
	while (*p!='\0')								// Assume stringlength >0 
	{
		while (*p==' ') p++;						// Skip whitespace
		if (*p=='\0') break;						// String might end in spaces
		argv[nargs++] = p;							// Store first valid char loc after whitespace
		while ((*p!=' ')&&(*p!='\0')) p++;			// Skip non-whitespace
	}
	if (nargs==0) return;							// No command or parameter
	for (i=0; i<NCMD; i++)							// Lookup shell command
		if (strncmp(argv[0], shell[i].cmdstr, shell[i].cmdlen) == 0) break;
	if (i<NCMD)
		(*shell[i].cmd)();
	else											// Unknown command
	{
		for (i=0; i<NCMD; i++)						// Print help if no match
    {
      Serialx.println(shell[i].cmdsyn);
			Serialx.println(shell[i].help);
    }
	}
}

/*
 * Monitor process 
 * This function collects characters from stdin until CR
 * Then the command is send to a parser and executed.
 */
void mon_evaluate(void)
{
	static int i = 0;
	//int c = getchar_timeout_us(10L);				// NOTE: this is the only SDK way to read from stdin
	//if (c==PICO_ERROR_TIMEOUT) return;				// Early bail out
  if (Serialx.available() > 0) 
  {
	  int c = Serialx.read();
  	switch (c)
  	{
  	case CR:										// CR : need to parse command string
  		Serialx.print('\n');								// Echo character, assume terminal appends CR
  		mon_cmd[i] = '\0';							// Terminate command string		
  		if (i>0)									// something to parse?
  			mon_parse(mon_cmd);						// --> process command
  		i=0;										// reset index
  		Serialx.print("Pico> ");							// prompt
  		break;
  	case LF:
  		break;										// Ignore, assume CR as terminator
  	default:
  		if ((c<32)||(c>=128)) break;				// Only allow alfanumeric
  		Serialx.print((char)c);							// Echo character
  		mon_cmd[i] = (char)c;						// store in command string
  		if (i<CMD_LEN) i++;							// check range and increment
  		break;
  	}
  }

}
