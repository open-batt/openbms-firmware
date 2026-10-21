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
#define BD_HPPC_DIR_UNKNOWN                 (0)             // not detected yet
#define BD_HPPC_DIR_CHARGE                  (1)
#define BD_HPPC_DIR_DISCHARGE               (2)

#define BD_HPPC_STATE_IDLE                  (0)
#define BD_HPPC_STATE_DETECT                (1)             // probing the current direction
#define BD_HPPC_STATE_REST                  (2)
#define BD_HPPC_STATE_RAMP                  (3)
#define BD_HPPC_STATE_DONE                  (4)
#define BD_HPPC_STATE_ABORTED               (5)

#define BD_HPPC_STOP_NONE                   (0)             // still running, or not started
#define BD_HPPC_STOP_CELL_MAX               (1)             // charging: a cell reached charge_stop_cell_mv
#define BD_HPPC_STOP_CELL_MIN               (2)             // discharging: a cell reached voltage_cell_min
#define BD_HPPC_STOP_TERM_CURRENT           (3)             // charging: current fell to charging_term_current
#define BD_HPPC_STOP_NO_CURRENT             (4)             // direction probe saw no current: no charger or load
#define BD_HPPC_STOP_NO_CONFIG              (5)             // cell_capacity or cell_count not set, or ocv_v_hi_mv not above ocv_v_lo_mv
#define BD_HPPC_STOP_HOST                   (6)             // the host switched back to normal run
#define BD_HPPC_STOP_PROTECTION             (7)             // a protection tripped, the FET is forced off
#define BD_HPPC_STOP_CURRENT_RANGE          (8)             // ramp current out of 0.5C to 2C: charger or load lost or wrong

void Hppc_Run(bool *fet);                                         // one pass while the device is in HPPC run, fet = FET state wanted
bool Hppc_Stop(bool *fet);                                        // HPPC run is being left: fet = FET state wanted (off). true when it can be left, false while the parameter extraction is still finishing
void Hppc_Abort(uint8_t reason, bool *fet);                       // test aborted from outside (BD_HPPC_STOP_*), back to normal run, fet = off
void Hppc_GetStatus(uint8_t *direction, uint16_t *status, uint16_t *extract);   // BD_HPPC_DIR_*, the hppc_status value and the BD_HPPC_X_* bits

#ifdef __cplusplus
}
#endif

#endif /* OPENBMS_HPPC_H */
