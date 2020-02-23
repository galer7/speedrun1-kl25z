#include "TFC.h"
#include "mbed.h"
 
//#define TERMINAL_PRINTF     PC.printf

int serialCamera() {
   
    uint32_t i = 0;
    for(i=0;i<128;i++)
    {
      if(i!=127) PC.printf("%d, ", TFC_LineScanImage0[i]);
      else PC.printf("%d", TFC_LineScanImage0[i]);
    }
}