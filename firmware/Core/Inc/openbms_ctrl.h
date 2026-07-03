#ifndef OPENBMS_CTRL_H
#define OPENBMS_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Ctrl_Init(void);
void Ctrl_Run(void);

void Ctrl_GetData(Control_Data_t *data);
void Ctrl_SetMode(uint16_t mode);

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_CTRL_H
