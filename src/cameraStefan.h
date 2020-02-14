#include "TFC.h"
#include "mbed.h"
 
//#define TERMINAL_PRINTF     PC.printf

int cameraStefan() {
    //PC.baud(115200);
   
    uint32_t i = 0;
    while(1)
    {
        if(TFC_GetDIP_Switch()&0x01)
            return;
        PC.printf("[");
        for(i=0;i<128;i++)
            {
                    if(i!=127) PC.printf("%d, ", TFC_LineScanImage0[i]);
                    else PC.printf("%d", TFC_LineScanImage0[i]);
            }
        PC.printf("]\n");
        
    }
}