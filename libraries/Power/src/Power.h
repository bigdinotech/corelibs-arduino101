#define OSC0_CFG1       0xb0800008
#define CCU_SYS_CLK_CTL 0xb0800038
#define OSCTRIM_ADDR    0xffffe1f8

#define SLEEP_HOST_C0       0
#define SLEEP_HOST_C1       1
#define SLEEP_HOST_C2       2
#define SLEEP_HOST_C2_LP    3
#define SLEEP_SS_SS0        4
#define SLEEP_SS_SS1        5
#define SLEEP_SS_SS2        6
#define SLEEP_LPSS          7

#include <stdint.h>
#include <interrupt.h>

class Power
{
    public:
        Power();
        
        //puts the SoC into "doze" mode which lowers the system clock speed to 32k
        void doze();
        
        void doze(int duration);
        
        void wakeFromDoze();
        
        void sleep(int mode);
        
        void sleep(int mode, int duration);
        
        void deepSleep();
        
        void switchToHybridOscillator();
        
        void switchToCrystalOscillator();
        
    private:
};

extern Power PM;
