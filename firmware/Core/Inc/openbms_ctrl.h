#ifndef OPENBMS_CTRL_H
#define OPENBMS_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

typedef enum 
{
    OPENBMS_OK = 0,
    OPENBMS_ERROR = -1,
} OpenBMS_Status_t;

typedef struct 
{
    uint32_t system_flags;
    uint16_t cell_count;
    bool initialized;
} OpenBMS_CtrlContext_t;

OpenBMS_Status_t OpenBMS_Ctrl_Init(void);
OpenBMS_Status_t OpenBMS_Ctrl_Run(void);

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_CTRL_H
