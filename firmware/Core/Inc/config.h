#ifndef CONFIG_H
#define CONFIG_H

// -----------------------------------------------------------------------------
// Build options
// -----------------------------------------------------------------------------

// Simulation build. The host (python_scripts/simulation/bms_sim.py) simulates the battery and
// the load, and sends the cell voltages, pack current and time step with CC_SIM_SAMPLE.
// Each sample runs Ctrl_Run once on the simulated time, so the HPPC test, the parameter
// extraction, the fuel gauge and the protections all run on the host's data.
// The FET pins always stay off, fet_status still shows what the firmware asked for.
 #define OPENBMS_SIMULATION

#endif
