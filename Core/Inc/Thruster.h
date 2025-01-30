#include "main.h"

#define delay_init 10000
#define kpercsec 0.5
void Thruster_Init(void);
void Thruster_Stop(void);
void Thruster_Set_Speed(uint16_t *speed);
void Thruster_Set_Perc(int *speed);
