#ifndef __USDX_SI5351_H__
#define __USDX_SI5351_H__

#ifdef __cplusplus
extern "C" {
#endif



#define SI5351_ADDR   0x60        // SI5351A I2C address: 0x60 for SI5351A-B-GT, Si5351A-B04771-GT, MS5351M; 0x62 for SI5351A-B-04486-GT; 0x6F for SI5351A-B02075-GT; see here for other variants: https://www.silabs.com/TimingUtility/timing-download-document.aspx?OPN=Si5351A-B02075-GT&OPNRevision=0&FileType=PublicAddendum
#define F_XTAL        27005000UL  // 27MHz SI5351 crystal


//TX CLK2
#define TX1RX0  0b11111011
#define TX1RX1  0b11111000
#define TX0RX1  0b11111100
#define TX0RX0  0b11111111


#define FAST   __attribute__((optimize("Ofast")))
enum ms_t { PLLA=0, PLLB=1, MSNA=-2, MSNB=-1, MS0=0, MS1=1, MS2=2, MS3=3, MS4=4, MS5=5 };
#define SI_CLK_OE  3




//***********************************************************************
//
//
//***********************************************************************
class SI5351 {
public:
  volatile int32_t _fout;
  volatile uint8_t _div;  // note: uint8_t asserts fout > 3.5MHz with R_DIV=1
  volatile uint16_t _msa128min512;
  volatile uint32_t _msb128;
  volatile uint8_t pll_regs[8];
  volatile uint32_t fxtal = F_XTAL;
  int16_t iqmsa; // to detect a need for a PLL reset
  //I2C i2c;
  
//***********************************************************************
  void FAST freq_calc_fast(int16_t df);  // note: relies on cached variables: _msb128, _msa128min512, _div, _fout, fxtal


//***********************************************************************
  void SendPLLRegisterBulk();

  
//***********************************************************************
  void SendRegister(uint8_t reg, uint8_t* data, uint8_t n);

//***********************************************************************
  void SendRegister(uint8_t reg, uint8_t val);


  
//***********************************************************************
  void ms(int8_t n, uint32_t div_nom, uint32_t div_denom, uint8_t pll = PLLA, uint8_t _int = 0, uint16_t phase = 0, uint8_t rdiv = 0);


//***********************************************************************
  void phase(int8_t n, uint32_t div_nom, uint32_t div_denom, uint16_t phase);

//***********************************************************************
  void reset();
  
//***********************************************************************
  void oe(uint8_t mask);

//***********************************************************************
  void freq(int32_t fout, uint16_t i, uint16_t q);

//***********************************************************************
  void freqb(uint32_t fout);
  

//***********************************************************************
  uint8_t RecvRegister(uint8_t reg);

//***********************************************************************
  void powerDown();

};




















#ifdef __cplusplus
}
#endif

#endif
