#ifndef OPENBMS_HPPC_EXTRACT_H
#define OPENBMS_HPPC_EXTRACT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// -------------------------------------------------------
// hppc_extract_status (System bank, read-only): state of the parameter extraction during an HPPC test
// -------------------------------------------------------
#define BD_HPPC_X_BUSY                      (1 << 8)        // test running, or fits and tables still being worked on
#define BD_HPPC_X_DONE                      (1 << 9)        // tables were written into the fuel gauge model
#define BD_HPPC_X_OVERRUN                   (1 << 10)       // a fit was not ready when the next rest started, that cycle was left out
#define BD_HPPC_X_FIT_WARN                  (1 << 11)       // a rest fit failed or is noisy, its cycle was interpolated from the neighbours
#define BD_HPPC_X_GAP_LO                    (1 << 12)       // lowest measured OCV is more than 0.15 V above ocv_v_lo_mv, low end extrapolated
#define BD_HPPC_X_GAP_HI                    (1 << 13)       // highest measured OCV is more than 0.15 V below ocv_v_hi_mv, high end extrapolated
#define BD_HPPC_X_FEW_CYCLES                (1 << 14)       // test ended with too few finished cycles, nothing written

// Parameter extraction during an HPPC test, the board side of hppc_pipeline.py.
// The sequencer (openbms_hppc.c) reports what it does, this module collects the
// data, fits the cells and writes the result into the fuel gauge model.

void     HppcX_Begin(uint8_t direction, uint8_t cell_count, uint32_t capacity_mah, uint16_t v_lo_mv, uint16_t v_hi_mv, uint32_t rest_ms);   // test starts, direction known
void     HppcX_RestStarted(void);                                  // FET went off: first rest, or a ramp has ended
void     HppcX_RampStarted(void);                                  // FET went on: a rest has ended
void     HppcX_Tick(const float *cell_mv, float current_ma, uint32_t dt_ms);   // once per loop tick while the sequencer is in REST or RAMP, dt_ms = time since the last pass
void     HppcX_Done(void);                                         // test ended by its own stop condition: finish the tables
void     HppcX_Abort(void);                                        // test aborted: drop everything collected
void     HppcX_Reset(void);                                        // new test: drop everything, including the status of the last test
void     HppcX_Run(void);                                          // background work, called every main loop pass
uint16_t HppcX_GetStatus(void);                                    // BD_HPPC_X_* bits

#ifdef __cplusplus
}
#endif

#endif /* OPENBMS_HPPC_EXTRACT_H */
