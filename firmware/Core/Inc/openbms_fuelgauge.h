#ifndef OPENBMS_FUELGAUGE_H
#define OPENBMS_FUELGAUGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// -------------------------------------------------------------------------
// Estimator update rate
//
// The ADS131M08 DRDY interrupt runs at ~252 Hz, but pack_current_filtered and
// cell_voltage_filtered are single-pole IIR filtered with ADS_READ_FILT_COEF
// (0.166), which puts their cutoff near 7 Hz — so there is no signal content
// above that to capture. 20 Hz clears Nyquist comfortably while matching the
// log rate battery_cycler.py records at, which keeps firmware output directly
// comparable against soc_estimator.py replays of the same run. The cell
// dynamics themselves are far slower still (tau1 ~13-16 s, tau2 ~95-145 s).
// -------------------------------------------------------------------------
#define FG_UPDATE_PERIOD_MS             (50U)       // 20 Hz

// -------------------------------------------------------------------------
// 0xAC — LearningStatus register bit definitions (read-only)
//
// Only the bits the estimator itself drives are defined here; the remaining
// bits stay reserved for the SOH/capacity learning stage.
// -------------------------------------------------------------------------
#define FG_STATUS_SEEDED                (1 << 0)    // Bit 0 — state seeded from OCV, filter running
#define FG_STATUS_GATED                 (1 << 1)    // Bit 1 — last update had its correction gated
#define FG_STATUS_AT_REST               (1 << 2)    // Bit 2 — |I| below deadzone, SOC correction blocked

void FuelGauge_Init(void);
void FuelGauge_Run(void);

void FuelGauge_GetData(FuelGauge_Data_t *data);
void FuelGauge_SetData(const FuelGauge_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif // OPENBMS_FUELGAUGE_H
