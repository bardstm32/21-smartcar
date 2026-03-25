#ifndef __BUZZER_H
#define __BUZZER_H
/**/
#include "zf_common_headfile.h"
#define Buzzer (IO_P67)

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);

#endif 