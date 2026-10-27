#ifndef OPENBMS_BALANCE_H
#define OPENBMS_BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void    Balance_Init(const Config_Data_t *config);              // keeps the config pointer, all cells off
uint8_t Balance_Run(bool allowed);                              // one pass, returns the cells to bleed (bit i = cell i), 0 when not allowed

#ifdef __cplusplus
}
#endif

#endif /* OPENBMS_BALANCE_H */
