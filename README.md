# Scale_Car_Teensy

Teensy 4.1 firmware for a scale FCHEV (Fuel-Cell Hybrid EV) platform.

This project controls:
- motor torque through VESC (`VescUart`),
- fuel-cell/battery power sharing through dual MDAC droop outputs (SPI),
- battery charger current and enable control (I2C + GPIO),
- command/telemetry communication with Raspberry Pi over UDP Ethernet,
- safety state machine, watchdog, and basic fault handling.

## Hardware interfaces

- **UART (`Serial1`)**: VESC motor controller.
- **SPI**: two MDAC chips (`CS_MDAC_FC`, `CS_MDAC_BT`) for FC/BT droop gains.
- **I2C (`Wire`)**: charger programming (`BQ25690`, `REG_ICHG`).
- **Ethernet/UDP**: command in (port 5001), telemetry out (port 5000).
- **Encoder interrupts**: wheel speed estimation from `ENC_A`/`ENC_B`.
- **ADC inputs**: FC/BT current, charger current, FC/BT/bus voltages.

## Runtime flow

Main loop execution order:
1. `updateSensors()`
2. `computeDerivedSignals()`
3. `detectFaults()`
4. `checkPiWatchdog()`
5. `receiveCommands()`
6. state machine (`doState0/1/2/3/99`)
7. `sendTelemetry()` at ~50 Hz

## State machine

- **State 0 (Init)**: enable regulators, initialize MDAC/charger/ESC, then go to idle.
- **State 1 (Idle)**: motor current zero, wait for run command from Pi.
- **State 2 (Run)**: execute `motorControl()`, `powerBalance()`, `chargingControl()`.
- **State 3 (Finish)**: stop motor, disable charger, return to idle.
- **State 99 (Error)**: disable outputs and stay latched.

## Key functions

### `receiveCommands()`
Parses 22-byte UDP packet from Pi, validates sync/checksum, updates:
- `v_setpoint`
- `power_share_setpoint`
- `charge_goal`
- `mode_cmd`
Also updates watchdog timestamp and sets state transition flags.

### `sendTelemetry()`
Builds 54-byte UDP packet containing measured/derived signals, droop gains, charger/fault status, and checksum.

### `motorControl()` + `PI_Controller_Motor()`
Computes torque from speed error (`v_setpoint - v_actual`) and sends current command to VESC.

### `powerBalance()` + `PI_Controller_Power()`
Controls FC/BT split using measured currents.
PI output (`droopRatio`) is converted to FC/BT droop gains and sent via `setDroopMdac()`.

### `chargingControl()` + `setChargerTargetCurrentA()`
Enables/disables charger based on setpoint and status pin.
Programs charger current over I2C using register `REG_ICHG`.

### `updateWheelSpeed()` + encoder ISRs
Uses encoder counts over a moving time window to estimate flywheel speed and update `v_actual`.

## Safety features

- Overcurrent: `I_fc > LIMIT_I_FC_MAX`
- Battery undervoltage: `V_batt < LIMIT_V_BATT_MIN`
- Bus overvoltage: `V_bus > LIMIT_V_BUS_MAX`
- Pi watchdog timeout (`PI_TIMEOUT_MS`)

Any major fault pushes controller into **State 99**.

## Notes for calibration

Before deployment, calibrate:
- `SCALE_I`, `SCALE_V_FC`, `SCALE_V_BATT`, `SCALE_V_BUS`
- `motorConstant`
- PI gains (`Kp`, `Ki`)
- encoder counts-per-rev mapping to true vehicle speed

## Build/flash

Open `teensy_controller.ino` in Arduino IDE (with Teensyduino and required libraries):
- `VescUart`
- `NativeEthernet`

Select Teensy 4.1 and upload.
