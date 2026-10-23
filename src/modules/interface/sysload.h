#ifndef __SYSLOAD_H__
#define __SYSLOAD_H__

void sysLoadInit();

// Called from vApplicationIdleHook to measure how long the idle task is starved
void sysLoadIdleHook(void);

#endif