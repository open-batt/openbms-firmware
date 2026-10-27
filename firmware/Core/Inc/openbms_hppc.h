#ifndef OPENBMS_HPPC_H
#define OPENBMS_HPPC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// SOC checkpoints of a test (hppc_points in openbms_hppc.c), also the number of cycles the extraction stores
#define HPPC_POINT_COUNT                    (20)

// -------------------------------------------------------
// HPPC direction and status (System bank, read-only)
// hppc_status: low byte = state, high byte = stop reason
// -------------------------------------------------------
#define HPPC_DIR_UNKNOWN                 (0)             // not detected yet
#define HPPC_DIR_CHARGE                  (1)
#define HPPC_DIR_DISCHARGE               (2)

#define HPPC_STATE_IDLE                  (0)
#define HPPC_STATE_DETECT                (1)             // probing the current direction
#define HPPC_STATE_REST                  (2)
#define HPPC_STATE_RAMP                  (3)
#define HPPC_STATE_DONE                  (4)
#define HPPC_STATE_ABORTED               (5)

#define HPPC_STOP_NONE                   (0)             // still running, or not started
#define HPPC_STOP_CELL_MAX               (1)             // charging: a cell reached voltage_cell_max
#define HPPC_STOP_CELL_MIN               (2)             // discharging: a cell reached voltage_cell_min
#define HPPC_STOP_TERM_CURRENT           (3)             // charging: current fell to charging_term_current
#define HPPC_STOP_NO_CURRENT             (4)             // direction probe saw no current: no charger or load
#define HPPC_STOP_NO_CONFIG              (5)             // cell_capacity or cell_count not set, or ocv_v_hi_mv not above ocv_v_lo_mv
#define HPPC_STOP_HOST                   (6)             // the host switched back to normal run
#define HPPC_STOP_PROTECTION             (7)             // a protection tripped, the FET is forced off
#define HPPC_STOP_CURRENT_RANGE          (8)             // ramp current out of 0.5C to 2C: charger or load lost or wrong
#define HPPC_STOP_TEMPERATURE            (9)             // NTC fault, or pack temperature outside HPPC_TEMP_MIN_C to HPPC_TEMP_MAX_C (at the start, or for HPPC_TEMP_OUT_MS)

void Hppc_Run(bool *fet);                                         // one pass while the device is in HPPC run, fet = FET state wanted
void Hppc_Stop(bool *fet);                                        // the test has ended (or the host stops it): fet = off, a running test is aborted, the parameter extraction finishes
void Hppc_Abort(uint8_t reason, bool *fet);                       // test aborted from outside (HPPC_STOP_*), fet = off, the device stays in HPPC run until reboot
void Hppc_GetStatus(uint8_t *direction, uint16_t *status, uint16_t *extract);   // HPPC_DIR_*, the hppc_status value and the HX_STATUS_* bits

#ifdef __cplusplus
}
#endif

#endif /* OPENBMS_HPPC_H */
