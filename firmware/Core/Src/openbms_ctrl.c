/**
  ******************************************************************************
  * @file           : openbms_ctrl.c
  * @brief          : OpenBMS Control Module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 OpenBatt.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "openbms_ctrl.h"
#include "openbms_comm.h"
#include "openbms_periph.h"

typedef enum
{
  LS_START,
  LS_CHECK_STATE,

} LearningState_t;

static LearningState_t  learning_state  = LS_START;

static void Run_Learning(void)
{
  switch(learning_state)
  {
    case LS_START:
    {
      // Disable FETs first
      Periph_SetFET(false);
      Periph_SetFET(false);
    }
    break;

    case LS_CHECK_STATE:
    {

    }
    break;
  }
}
void Ctrl_Init(void)
{
  Periph_Init();
  Comm_Init();
}
void Ctrl_Run(void)
{
  if(Perigh_GetLearningState())
  {
    Run_Learning();
  }
  else 
  {
  
  }
  

  Periph_Run();
  Comm_Run();
}
