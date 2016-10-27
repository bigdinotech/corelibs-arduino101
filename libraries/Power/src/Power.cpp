#include "Power.h"

Power PM;

Power::Power()
{
    
}

void Power::doze()
{
  //switch from external crystal oscillator to internal hybrid oscilator
  switchToHybridOscillator();
  
  //Set system clock to the RTC Crystal Oscillator
  uint32_t current_val = *(uint32_t*)CCU_SYS_CLK_CTL;
  *(uint32_t*)(CCU_SYS_CLK_CTL) = current_val & 0xFFFFFFFE;

  //Powerdown hybrid oscillator
  current_val = *(uint32_t*)(OSC0_CFG1);
  *(uint32_t*)(OSC0_CFG1) = current_val | 0x00000004; 
}

void Power::doze(int duration)
{
    doze();
    //wait some time here
    wakeFromDoze();
}

void Power::wakeFromDoze()
{
  //Powerup hybrid oscillator
  uint32_t current_val = *(uint32_t*)(OSC0_CFG1);
  *(uint32_t*)(OSC0_CFG1) = current_val & 0xFFFFFFFB;;
   
  //Set system clock to the Hybrid Oscillator
  current_val = *(uint32_t*)CCU_SYS_CLK_CTL;
  *(uint32_t*)(CCU_SYS_CLK_CTL) = current_val | 0x00000001;

  //switch back to the external crystal oiscillator
  void switchToCrystalOscillator();
}

void Power::switchToHybridOscillator()
{
    //read trim value from OTP
    uint32_t trimMask = *(uint16_t*)OSCTRIM_ADDR << 20;
    *(uint32_t*)(OSC0_CFG1) = 0x00000002 | trimMask;  //switch to internal oscillator using trim value from OTP
}

void Power::switchToCrystalOscillator()
{
    *(uint32_t*)(OSC0_CFG1) = 0x00070009;
}
