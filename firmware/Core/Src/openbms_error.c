#include "openbms_error.h"

static volatile uint64_t error_flags = 0;

void Error_Set(OpenBMS_Error_t error)
{
  // The flags are 64 bits wide and this is also called from interrupts, so the
  // read-modify-write is done with interrupts masked
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  error_flags |= (1ULL << error);
  __set_PRIMASK(primask);
}
void Error_ClearAll(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  error_flags = 0;
  __set_PRIMASK(primask);
}
uint64_t Error_GetFlags(void)
{
  uint32_t primask = __get_PRIMASK();
  uint64_t flags;

  __disable_irq();
  flags = error_flags;
  __set_PRIMASK(primask);

  return flags;
}
bool Error_IsSet(OpenBMS_Error_t error)
{
  return (Error_GetFlags() & (1ULL << error)) != 0U;
}
