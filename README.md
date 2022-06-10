# Arduino_uSDX_Pico_FFT_Proj
Hamradio SDR transceiver software   -  by Klaus Fensterseifer - PY2KLA



This project is based on  Arjante Marvelde / uSDR-pico, from https://github.com/ArjanteMarvelde/uSDR-pico
 . I strongly recommend you to take a look there before trying to follow this one.

My intention was to include a waterfall or panadapter to the uSDR-pico project, for this, I included an ILI9341 240x320, without touch, to the project, and also, changed the software to generate the waterfall.

Initially, I have used Visual Studio, but after some considerations, I ported all code to Arduino IDE. So, to compile and run this code you need the Arduino IDE installed for a Raspberry Pi Pico project.

I also, chose not to change the original software as much as possible, and focused on the waterfall implementation, mostly in the dsp.c.


Initial tests video:  https://youtu.be/0zGAnkRjizE


To implement the waterfall I have considered this:

- There are 3 ADC inputs: I, Q and MIC  (if we remove the VOX function, we could remove the MIC ADC during reception, this will increase the ADC frequency for I and Q, improving the frequencies we can see at the display - for now I will keep it like the original).
- The max ADC frequency is 500kHz, I have changed it to 480kHz (close to the original) to make the divisions "rounded".
- The ADC for audio reception has frequency of 16kHz (close to the original). I have tested higher frequencies, but the time became critical, without so much benefit.
- The max ADC frequency for each sample = 480kHz / 3 = 160kHz   (because there is only one internal ADC for 3 inputs).
- With 160kHz of samples, we can see 80kHz range after the FFT, but if we apply Hilbert to get the lower band and the upper band, we get two bands of 80kHz, one above and one below the center frequency.
- There is no time to process each sample at 160kHz and generate the "live" audio, so I use this method:
    Set the DMA to receive 10 samples of each ADC input (10 x 3 = 30) and generate a interrupt.
    So, we get 16kHz interrupts with 10 x 3 samples to deal. 
    For audio, we need only one at each interruption of 16kHz, but to improve the signal, I made an average from the last 10 samples to deliver to audio (this is also a low pass filter).
    For FFT, we need all samples, so they are copied to a FFT buffer for later use.
- There is also no time to process the samples and the receiver part at 16kHz, so I chose to split it, the interrupt and buffer/average part is done at Core1, and the audio original reception is in the Core0.
- Every 16kHz interrupt, after average the I, Q and MIC audio samples are passed to Core0.
- For FFT, when we have received 320 I and Q samples, it stops of filling the buffer and indicates to the Core1 main loop to process a new FFT and waterfall graphic.
- The original processes run at Core0, every 100ms.
- There is a digital low pass filter FIR implemented at the code (in the original too) that will give the passband we want for audio.
  This filter was calculated with the help of this site:  http://t-filter.engineerjs.com/
  The dificulty is that the number of taps could not be high (there is no much time to spend), so the filter must be chosen carefully.
- Block diagram at "Arduino uSDR Pico FFT.png".


Nyquist considerations:
If we sample each signal I, Q and MIC at 160kHz, it is necessary to have a hardware low pass filter for max 80kHz on each input (anti-aliasing filter).
If we deliver an audio signal at 16kHz (sample frequency), we need a hardware low pass filter for less than 8kHz at the output (the sample frequency will be present and need to be removed as it is an audio frequency).


Microcontroller RP2040 notes:
- Core0 and Core1 are too much connected and affect each other. This made me lose some painful hours...
- There are only 3 ADC ports available.
- There are some reports at internet about the low quality of the RP2040 ADC readings.


Arduino IDE setup and notes:
- I am using Arduino IDE version 1.8.19 in Linux/Ubuntu
- Lib used: TFT_eSPI by Bodmer
- There are some comments at beginning of  .ino  file.  I use them to "adjust" the library files to the project.
- Boards Manager:  Arduino Mbed OS RP2040 Boards. My version is 3.0.1 (If I update it, I will need to adjust the library files again, so I will leave it for later).
- Do not use EarlePhilhower library (it is just conflitant with Mbed)
- Board: "RaspberryPiPico"  >  Arduino Mbed OS RP2040 Boards  >  RaspberryPiPico
- The code files have cpp type, but the code itself is in C (cpp type is used to help in some compiler issues).

Hardware, changes and notes:
- Inclusion of ILI9341 on free pins, using SPI1, and removing the LCD display.
- Schematic diagram at "FFT_LCD_pico.png".
- I noticed that changing the signal in one ADC input, changes the other inputs signal through the resistors for setting half Vref. To solve this, I changed the circuit to have a separate resistor divider for each ADC input.
- Use input/output filters for Nyquist considerations (see above). 
- Obs.: at the initial test video, I used only the RC output filter shown in the schematic, and for input filter, only what is already inside of the Softrock RXTX Ensemble.


Last changes and notes:
Jun10 2022
- AGC uncommented and adapted to work.
- A visual scope was implemented to allow visualization of some internal variables.
- The signal level meter at the display does not change because it is fixed at the original code (the level depends on the software as well as the hardware).


To do list:
- There is some noise on ADC readings with nothing connected, so I will try some capacitors on Vref and each ADC input.
- Write to the display only when something changes.
- Tests: reception/transmission SSB...  menus...  switches/debounce...   display appearance
- Try to improve the suppression of not desired audio and waterfall bands

