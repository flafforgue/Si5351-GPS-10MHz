// ----------------------------------------------------------------------------
//                                  Si5351 
// ----------------------------------------------------------------------------

#include "Si5351.h"
#include <Wire.h>

// ----------------------------------------------------------------------------
//                               Write data
// ----------------------------------------------------------------------------

void Si5351_write(uint8_t addr, uint8_t data) {
  Wire.beginTransmission(Si5351A_addr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

// ----------------------------------------------------------------------------
//                       Si5351 initialization routines
// ----------------------------------------------------------------------------

void Si5351_Init() {
  // Initialize Si5351_
  Si5351_write(XTAL_LOAD_CAP     ,0b11000000);  // Set crystal load to 10pF
  Si5351_write(CLK_ENABLE_CONTROL,0b00000000);  // Enable all outputs
  Si5351_write(CLK0_CONTROL      ,0b00001111);  // Set PLLA to CLK0, 8 mA output
  Si5351_write(CLK1_CONTROL      ,0b00001111);  // Set PLLA to CLK1, 8 mA output
  Si5351_write(CLK2_CONTROL      ,0b00101111);  // Set PLLB to CLK2, 8 mA output
  Si5351_write(PLL_RESET         ,0b10100000);  // Reset PLLA and PLLB

  // Set PLLA and PLLB to 900 MHz
  unsigned long  a, b, c, p1, p2, p3;

  a = 36;                                       // Derived from 900/25 MHz
  b = 0;                                        // Numerator
  c = 0xFFFFF;                                  // Denominator derived from max bits 2^20

  // Refer to Si5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  =  128 * a;
  p1 += (128 * b / c);
  p1 -=  512;

  // Write data to PLL registers
  Si5351_write(SYNTH_PLL_A,     0xFF);
  Si5351_write(SYNTH_PLL_A + 1, 0xFF);
  Si5351_write(SYNTH_PLL_A + 2,         (p1 & 0x00030000) >> 16);
  Si5351_write(SYNTH_PLL_A + 3,         (p1 & 0x0000FF00) >>  8);
  Si5351_write(SYNTH_PLL_A + 4,         (p1 & 0x000000FF)      );
  
  Si5351_write(SYNTH_PLL_A + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(SYNTH_PLL_A + 6,         (p2 & 0x0000FF00) >>  8);
  Si5351_write(SYNTH_PLL_A + 7,         (p2 & 0x000000FF)      );

  Si5351_write(SYNTH_PLL_B,     0xFF);
  Si5351_write(SYNTH_PLL_B + 1, 0xFF);
  Si5351_write(SYNTH_PLL_B + 2,         (p1 & 0x00030000) >> 16);
  Si5351_write(SYNTH_PLL_B + 3,         (p1 & 0x0000FF00) >>  8);
  Si5351_write(SYNTH_PLL_B + 4,         (p1 & 0x000000FF)      );
  
  Si5351_write(SYNTH_PLL_B + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(SYNTH_PLL_B + 6,         (p2 & 0x0000FF00) >>  8);
  Si5351_write(SYNTH_PLL_B + 7,         (p2 & 0x000000FF)      );
}

// ----------------------------------------------------------------------------
//                              Set Frequency
// ----------------------------------------------------------------------------

unsigned long Si5351_SetFreq(int synth, unsigned long  freq, unsigned char Rdiv) {
  unsigned long long CalcTemp;
  unsigned long  a, b, c, p1, p2, p3;

  c          =  0xFFFFF;                      // Denominator derived from max bits 2^20
  a          =  ((XtalFreq * 36)/ 1 ) / freq; // 36 is derived from 900/25 MHz

  double bj  = ((XtalFreq * 36) % c ) * c / freq;
  double err = abs(  ( b - int(b) ) / b );
    
  for (int j=1;j<10;j++) {
    unsigned long  cj   = 0xFFFFF - j;
                   bj   = ((XtalFreq * 36) % cj ) * cj / freq;
    double         errj = abs(  ( b - int(bj) ) / b );
    if ( errj < err ) {
      err = errj;
      c   = cj;
    }
  }
  
  CalcTemp =  round((XtalFreq * 36) / 1 ) % freq;
  CalcTemp *= c;
  CalcTemp /= freq ; 
  b        =  CalcTemp;                    // Calculated numerator

  // Refer to Si5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  =  128 * a;
  p1 += (128 * b / c);
  p1 -=  512;

  // Write data to multisynth registers
  Si5351_write(synth    , 0xFF );                             // 42  MS0_P3[15:8]
  Si5351_write(synth + 1, 0xFF );                             // 43  MS0_P3[7:0]
  Si5351_write(synth + 2, Rdiv |  (p1 & 0x00030000) >> 16);   // 44  R0_DIV[2:0]  MS0_P1[17:16]
  Si5351_write(synth + 3,         (p1 & 0x0000FF00) >> 8);    // 45  MS0_P1[15:8]
  Si5351_write(synth + 4,         (p1 & 0x000000FF));         // 46  MS0_P1[7:0]
  Si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));  // 47  MS0_P3[19:16] MS0_P2[19:16]
  Si5351_write(synth + 6,         (p2 & 0x0000FF00) >> 8);    // 48  MS0_P2[15:8]
  Si5351_write(synth + 7,         (p2 & 0x000000FF));         // 49  MS0_P2[7:0]   

  bj = a + double( b ) / double ( c );
  return (XtalFreq * 36) / bj ;
}

// ----------------------------------------------------------------------------
//                              --- E N D  ---
// ----------------------------------------------------------------------------
