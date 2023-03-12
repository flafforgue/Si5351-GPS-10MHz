
// ----------------------------------------------------------------------------
//                                  Si5351 
// ----------------------------------------------------------------------------

#include <arduino.h>

#if !defined Si5351_H
#define Si5351_H
  
// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------

#define XtalFreq          25000000

#define Si5351A_addr         (0x60)  // ADDR pin = low

#define CLK_ENABLE_CONTROL       3  // 0 enable   / 1 disable  :  D0 = Clk0 ... D7 = Clk7
#define CLK_ENABLE_CONTROL_PIN   9  // 0 Pin Ctrl / 1 No Pin   :  D0 = Clk0 ... D7 = Clk7
#define CLK0_CONTROL            16 
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183

#define RDIV_1                 0x00
#define RDIV_2                 0x10
#define RDIV_4                 0x20
#define RDIV_8                 0x30
#define RDIV_16                0x40
#define RDIV_32                0x50
#define RDIV_64                0x60
#define RDIV_128               0x70

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------

void Si5351_write(uint8_t addr, uint8_t data);
void Si5351_Init();
unsigned long  Si5351_SetFreq(int synth, unsigned long  freq, unsigned char Rdiv);

#endif

// ----------------------------------------------------------------------------
//                              --- E N D  ---
// ----------------------------------------------------------------------------
