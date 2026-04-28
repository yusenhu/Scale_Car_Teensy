#include <VescUart.h>
#include <SPI.h>
#include <Wire.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

VescUart vesc;
EthernetUDP Udp;

// ── Pin definitions ───────────────────────────────────────────────────────────
#define RX              0
#define TX              1
#define ENC_A           2
#define FC_REG_ENABLE   3
#define BT_REG_ENABLE   4
#define CHARGER_ENABLE  5
#define CHARGER_OK      6
#define ENC_ENABLE      7
#define ENC_B           8
#define MOSI            11
#define MISO            12
#define SCK             13
#define SDA             18
#define SCL             19
#define FC_VOLTAGE      24
#define BT_VOLTAGE      25
#define BUS_VOLTAGE     26
#define CS_MDAC_FC      36
#define CS_MDAC_BT      37
#define CHRG_CURRENT    39
#define FC_CURRENT      40
#define BT_CURRENT      41

// ── ADC calibration constants ─────────────────────────────────────────────────
#define ADC_VREF        3.3f
#define ADC_MAX         1023.0f
#define SCALE_I         (ADC_VREF / ADC_MAX / K_sns)   // raw -> amps
#define SCALE_V_FC      (15.0f / ADC_MAX)              // TODO: calibrate for your R divider (assumes 0-15V range)
#define SCALE_V_BATT    (10.0f / ADC_MAX)              // TODO: calibrate for your R divider (assumes 0-10V range)
#define SCALE_V_BUS     (18.0f / ADC_MAX)

// ── Safety limits ─────────────────────────────────────────────────────────────
#define FAULT_OC_FC     0x01   // I_fc overcurrent
#define FAULT_UV_BATT   0x02   // V_batt undervoltage
#define FAULT_OV_BUS    0x04   // V_bus overvoltage
#define FAULT_ERROR     0x80   // error state

#define LIMIT_I_FC_MAX   3.5f   // A  — H-20 max
#define LIMIT_V_BATT_MIN 6.2f   // V  — 2S LiPo cutoff
#define LIMIT_V_BUS_MAX  18.5f  // V  — bus overvoltage

// I2C Address
#define CHARGER_ADDR 0x6A   // 6.3.11 Serial Interfa
#define REG_ICHG     0x02   // 6.5.1 BQ25690 Registers
// ── State machine ─────────────────────────────────────────────────────────────
int mainState = 0;

// ── Physical constants ────────────────────────────────────────────────────────
const int CPR = 16;
const int MDAC_res = 4095;
const int32_t sampleTime = 50;    // us

const float tireRadius    = 1;    // inch
const float flyWheelRadius = 1;   // inch
const float K_sns = 0.4f;         // V/A current sensor gain
const float A_v   = 5.02f;        // static gain
const float k_eq  = 0.45f;        // ohm

// ── Sensor readings ───────────────────────────────────────────────────────────
float v_actual       = 0;
float current        = 0;
float targetMotorTorque = 0;
float P_fc_actual    = 0;
float P_batt_actual  = 0;

float I_fc           = 0;
float I_batt         = 0;
float V_fc           = 0;
float V_batt         = 0;
float V_bus          = 18.0f;   // SID 2026-04-21: updated default to 18V (nominal bus)
float I_charge       = 0;

float P_motor_actual     = 0;
float power_share_actual = 0;
float power_share_echo   = 0;
float droop_gain_FC_actual = 0;
float droop_gain_BT_actual = 0;

uint8_t charger_status = 0;
uint8_t fault_flags    = 0;

// ── Commands received from Pi ─────────────────────────────────────────────────
float v_setpoint           = 0;
float power_share_setpoint = 0.5f;
float charge_goal          = 0;
uint8_t mode_cmd           = 4;   // default SAFE

// ── State transition flags ────────────────────────────────────────────────────
bool changeToRun = false;
bool changeToFin = false;

// ── Encoder ───────────────────────────────────────────────────────────────────
volatile byte AfirstUp   = 0;
volatile byte BfirstUp   = 0;
volatile byte AfirstDown = 0;
volatile byte BfirstDown = 0;
volatile int encoderPos      = 0;
volatile int lastEncoderPos  = 0;
volatile byte pinA_read = 0;
volatile byte pinB_read = 0;
constexpr float ENCODER_COUNTS_PER_REV = 1024.0f;

// ── Network config ────────────────────────────────────────────────────────────
IPAddress pi_ip(192, 168, 1, 100);

// Pi bridge listens on 5000, Teensy listens on 5001
const int pi_port    = 5000;   // Pi receives telemetry on port 5000
const int local_port = 5001;   // Teensy receives commands on port 5001

const uint8_t SYNC_BYTE_TX = 0xAA;   // Teensy -> Pi header
const uint8_t SYNC_BYTE_RX = 0xBB;   // Pi -> Teensy header

uint16_t pkt_counter_T = 0;

// ── Safety watchdog ───────────────────────────────────────────────────────────
uint32_t last_rx_ms  = 0;
bool     pi_ever_connected = false;
const uint32_t PI_TIMEOUT_MS = 500;

// ── Motor constant ────────────────────────────────────────────────────────────
const float motorConstant  = 0.1f;    // TODO: tune this
const float maxChargeCurrentA = 5.0f;


// ═════════════════════════════════════════════════════════════════════════════
// SETUP
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    initEscUartPins();
    initMdacSpiPins();
    initBatteryChargerI2cPins();

    pinMode(RX,            INPUT);
    pinMode(TX,            OUTPUT);
    pinMode(ENC_A,         INPUT);
    pinMode(ENC_B,         INPUT);
    pinMode(ENC_ENABLE,    OUTPUT);
    pinMode(FC_REG_ENABLE, OUTPUT);
    pinMode(BT_REG_ENABLE, OUTPUT);
    pinMode(CHARGER_ENABLE,OUTPUT);
    pinMode(CHARGER_OK,    INPUT);
    pinMode(CS_MDAC_FC,    OUTPUT);
    pinMode(CS_MDAC_BT,    OUTPUT);

    digitalWrite(CS_MDAC_FC,    HIGH);
    digitalWrite(CS_MDAC_BT,    HIGH);
    digitalWrite(BT_REG_ENABLE, HIGH);
    digitalWrite(FC_REG_ENABLE, HIGH);
    digitalWrite(CHARGER_ENABLE,LOW);
    digitalWrite(ENC_ENABLE,    LOW);

    attachInterrupt(digitalPinToInterrupt(ENC_A), doEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), doEncoderB, CHANGE);

    byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
    IPAddress ip(192, 168, 1, 50);
    Ethernet.begin(mac, ip);
    Udp.begin(local_port);

    Serial.println("Teensy FCHEV ready | IP=192.168.1.50 | Listening on port 5001");
}


// ═════════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
    
    updateSensors();
    computeDerivedSignals();
    detectFaults();
    checkPiWatchdog();
    receiveCommands();

    switch (mainState) {
        case 0:  doState0();  break;
        case 1:  doState1();  break;
        case 2:  doState2();  break;
        case 3:  doState3();  break;
        case 99:
        default: doState99(); break;
    }

    // Send telemetry at ~50 Hz
    static uint32_t lastSend = 0;
    if (millis() - lastSend > 20) {
        sendTelemetry();
        //printToTerminal();
        lastSend = millis();
    }
}

void printToTerminal(){
    Serial.println("V_batt = " + String(V_batt));
    Serial.println("I_batt = " + String(I_batt));
    Serial.println("I_charge = " + String(I_charge));
    Serial.println("V_fc = " + String(V_fc));
    Serial.println("I_fc = " + String(I_fc));
    Serial.println("V_bus = " + String(V_bus));
    //println("" + );
}

// ═════════════════════════════════════════════════════════════════════════════
// SENSOR READING
// ═════════════════════════════════════════════════════════════════════════════
void updateSensors() {
    updateWheelSpeed();

    // Raw ADC is 0-1023, Vref=3.3V, K_sns=0.4 V/A
    I_fc     = analogRead(FC_CURRENT)   * SCALE_I;
    I_batt   = analogRead(BT_CURRENT)   * SCALE_I;
    I_charge = analogRead(CHRG_CURRENT) * SCALE_I;

    // IMPORTANT: SCALE_V_FC and SCALE_V_BATT depend on your resistor divider values
    // Measure R1 and R2 with a multimeter and recalculate:
    // SCALE = Vmax / ADC_MAX  where Vmax = Vref * (R1+R2)/R2
    V_fc   = analogRead(FC_VOLTAGE)   * SCALE_V_FC;
    V_batt = analogRead(BT_VOLTAGE) * SCALE_V_BATT;

    // V_bus: measure from boost regulator output
    // TODO: add V_bus ADC pin and calibration
    V_bus = analogRead(BUS_VOLTAGE) * SCALE_V_BUS;
}

void computeDerivedSignals() {
    float totalA = fabsf(I_fc) + fabsf(I_batt);
    if (totalA > 1e-6f) {
        power_share_actual = fabsf(I_fc) / totalA;
    }

    P_fc_actual    = V_fc   * I_fc;
    P_batt_actual  = V_batt * I_batt;
    P_motor_actual = V_bus  * current;
    power_share_echo = power_share_setpoint;
}

void detectFaults() {
    fault_flags = 0;

    if (I_fc   > LIMIT_I_FC_MAX)   fault_flags |= FAULT_OC_FC;
    if (V_batt < LIMIT_V_BATT_MIN) fault_flags |= FAULT_UV_BATT;
    if (V_bus  > LIMIT_V_BUS_MAX)  fault_flags |= FAULT_OV_BUS;
    if (mainState == 99)            fault_flags |= FAULT_ERROR;

    // If any fault detected, go to error state immediately
    if (fault_flags & (FAULT_OC_FC | FAULT_UV_BATT | FAULT_OV_BUS)) {
        mainState = 99;
        Serial.print("FAULT detected: 0x");
        Serial.println(fault_flags, HEX);
    }
}

void checkPiWatchdog() {
    if (!pi_ever_connected) return;   // don't trigger before first connection
    if (millis() - last_rx_ms > PI_TIMEOUT_MS) {
        mainState = 99;
        Serial.println("Pi timeout — entering error state");
    }
}


// ═════════════════════════════════════════════════════════════════════════════
// UDP COMMUNICATION
// ═════════════════════════════════════════════════════════════════════════════
void receiveCommands() {
    int packetSize = Udp.parsePacket();
    if (packetSize != 22) return;

    uint8_t buffer[22];
    Udp.read(buffer, 22);

    // Pi sends 0xBB as header, not 0xAA
    if (buffer[0] != SYNC_BYTE_RX) return;

    // Checksum over bytes 1-20 (skip header[0] and checksum[21])
    uint8_t checksum = 0;
    for (int i = 1; i < 21; i++) checksum ^= buffer[i];
    if (checksum != buffer[21]) {
        Serial.println("Checksum mismatch — packet dropped");
        return;
    }

    int idx = 1;

    uint32_t timestamp;
    memcpy(&timestamp, &buffer[idx], 4); idx += 4;

    uint16_t pkt_counter_Pi;
    memcpy(&pkt_counter_Pi, &buffer[idx], 2); idx += 2;

    memcpy(&v_setpoint,           &buffer[idx], 4); idx += 4;
    memcpy(&power_share_setpoint, &buffer[idx], 4); idx += 4;
    memcpy(&charge_goal,          &buffer[idx], 4); idx += 4;

    mode_cmd           = buffer[idx++];
    (void)buffer[idx++];   // droop_enable — received but not yet used

    last_rx_ms        = millis();
    pi_ever_connected = true;

    // MODE_HYBRID=0, MODE_FC_ONLY=1, MODE_BATT=2, MODE_CHARGE=3, MODE_SAFE=4
    if (mode_cmd <= 3 && mainState == 1) {
        changeToRun = true;    // any active mode -> transition to run
    }
    if (mode_cmd == 4 && mainState == 2) {
        changeToFin = true;    // SAFE mode -> finish and return to idle
    }
}

void sendTelemetry() {
    uint8_t packet[54];
    int idx = 0;

    packet[idx++] = SYNC_BYTE_TX;   // 0xAA

    uint32_t t = millis();
    memcpy(&packet[idx], &t,             4); idx += 4;
    memcpy(&packet[idx], &pkt_counter_T, 2); idx += 2;

    memcpy(&packet[idx], &v_actual,          4); idx += 4;
    memcpy(&packet[idx], &V_batt,            4); idx += 4;
    memcpy(&packet[idx], &I_batt,            4); idx += 4;
    memcpy(&packet[idx], &I_charge,          4); idx += 4;
    memcpy(&packet[idx], &V_fc,              4); idx += 4;
    memcpy(&packet[idx], &I_fc,              4); idx += 4;
    memcpy(&packet[idx], &V_bus,             4); idx += 4;
    memcpy(&packet[idx], &P_motor_actual,    4); idx += 4;
    memcpy(&packet[idx], &power_share_echo,  4); idx += 4;
    memcpy(&packet[idx], &power_share_actual,4); idx += 4;

    uint16_t fc_u16 = (uint16_t)(constrain(droop_gain_FC_actual, 0.0f, 1.0f) * 65535.0f);
    uint16_t bt_u16 = (uint16_t)(constrain(droop_gain_BT_actual, 0.0f, 1.0f) * 65535.0f);
    memcpy(&packet[idx], &fc_u16, 2); idx += 2;
    memcpy(&packet[idx], &bt_u16, 2); idx += 2;

    packet[idx++] = charger_status;
    packet[idx++] = fault_flags;

    // Checksum over bytes 1-52
    uint8_t checksum = 0;
    for (int i = 1; i < 53; i++) checksum ^= packet[i];
    packet[idx++] = checksum;

    Udp.beginPacket(pi_ip, pi_port);
    Udp.write(packet, 54);
    Udp.endPacket();

    pkt_counter_T++;
}


// ═════════════════════════════════════════════════════════════════════════════
// STATE MACHINE
// ═════════════════════════════════════════════════════════════════════════════
void doState0() {
    // Initialization
    digitalWrite(FC_REG_ENABLE, HIGH);
    digitalWrite(BT_REG_ENABLE, HIGH);
    initMdacOutputs();
    initBatteryCharger();
    initEsc();
    digitalWrite(CS_MDAC_FC, HIGH);
    digitalWrite(CS_MDAC_BT, HIGH);
    digitalWrite(ENC_ENABLE, HIGH);
    Serial.println("State 0 -> State 1 (IDLE)");
    mainState = 1;
}

void doState1() {
    // IDLE — wait for Pi to send a run command
    vesc.setCurrent(0);
    if (changeToRun) {
        changeToRun = false;
        Serial.println("State 1 -> State 2 (RUN)");
        mainState = 2;
    }
}

void doState2() {
    // RUN — full EMS control
    motorControl();
    powerBalance();
    chargingControl();
    if (changeToFin) {
        changeToFin = false;
        Serial.println("State 2 -> State 3 (FINISH)");
        mainState = 3;
    }
}

void doState3() {
    // Finishing — stop motor, disable charger, return to idle
    vesc.setCurrent(0);
    digitalWrite(CHARGER_ENABLE, LOW);
    Serial.println("State 3 -> State 1 (IDLE)");
    mainState = 1;
}

void doState99() {
    // Error — disable everything
    vesc.setCurrent(0);
    digitalWrite(FC_REG_ENABLE, LOW);
    digitalWrite(BT_REG_ENABLE, LOW);
    digitalWrite(CHARGER_ENABLE, LOW);
    // Stay in error state until power cycle
}


// ═════════════════════════════════════════════════════════════════════════════
// CONTROL FUNCTIONS
// ═════════════════════════════════════════════════════════════════════════════
void motorControl() {
    targetMotorTorque = PI_Controller_Motor(v_setpoint - v_actual);
    current = targetMotorTorque / motorConstant;
    vesc.setCurrent(current);
}

float PI_Controller_Motor(float error) {
    const float Kp = 1.0f;
    const float Ki = 1.0f;
    static float accumulatedError = 0;
    static uint32_t lastMicros = 0;

    uint32_t now = micros();
    uint32_t dtMicros = now - lastMicros;
    if (dtMicros < (uint32_t)sampleTime) return 0.0f;   // FIX 6
    lastMicros = now;

    accumulatedError += error * dtMicros * 1e-6f;
    return Kp * error + Ki * accumulatedError;
}

void powerBalance() {
    float totalA = fabsf(I_fc) + fabsf(I_batt);
    if (totalA < 1e-6f) return;

    float power_share_actual_local = fabsf(I_fc) / totalA;
    float shareError = power_share_setpoint - power_share_actual_local;
    float droopRatio = PI_Controller_Power(shareError);

    // Clamp to avoid divide by zero
    droopRatio = constrain(droopRatio, 0.01f, 0.99f);

    droop_gain_FC_actual = k_eq / droopRatio       / K_sns / A_v;
    droop_gain_BT_actual = k_eq / (1.0f - droopRatio) / K_sns / A_v;
    setDroopMdac(droop_gain_FC_actual, droop_gain_BT_actual);
}

float PI_Controller_Power(float error) {
    const float Kp = 1.0f;
    const float Ki = 1.0f;
    static float accumulatedError = 0;
    static uint32_t lastMicros = 0;

    uint32_t now = micros();
    uint32_t dtMicros = now - lastMicros;
    if (dtMicros < (uint32_t)sampleTime) return 0.0f;
    lastMicros = now;

    accumulatedError += error * dtMicros * 1e-6f;
    return Kp * error + Ki * accumulatedError;
}

void setDroopMdac(float fc_gain, float bt_gain) {
    uint16_t fcCode = (uint16_t)(constrain(fc_gain, 0.0f, 1.0f) * MDAC_res);
    uint16_t btCode = (uint16_t)(constrain(bt_gain, 0.0f, 1.0f) * MDAC_res);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_MDAC_FC, LOW);
    SPI.transfer16(fcCode);
    digitalWrite(CS_MDAC_FC, HIGH);
    digitalWrite(CS_MDAC_BT, LOW);
    SPI.transfer16(btCode);
    digitalWrite(CS_MDAC_BT, HIGH);
    SPI.endTransaction();
}

void chargingControl() {
    bool chargerReady = (digitalRead(CHARGER_OK) == HIGH);

    if (charge_goal <= 0.05f) {
        digitalWrite(CHARGER_ENABLE, LOW);
        setChargerTargetCurrentA(0.0f);
        return;
    }

    float cmdA = constrain(charge_goal, 0.0f, maxChargeCurrentA);
    if (chargerReady) {
        digitalWrite(CHARGER_ENABLE, HIGH);
        setChargerTargetCurrentA(cmdA);
    } else {
        digitalWrite(CHARGER_ENABLE, LOW);
        setChargerTargetCurrentA(0.0f);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// CHARGER I2C 
// ═════════════════════════════════════════════════════════════════════════════
void setChargerTargetCurrentA(float currentA) {
    currentA = constrain(currentA, 0.0f, 3.3f);

    uint16_t ichg_code = (uint16_t)(currentA / 0.02f + 0.5f); // ICHG step size = 20 mA (Rounding included)

    uint8_t lowByte  = ichg_code & 0xFF;
    uint8_t highByte = (ichg_code >> 8) & 0xFF;

    Wire.beginTransmission(CHARGER_ADDR);
    Wire.write(REG_ICHG);
    Wire.write(lowByte);
    Wire.write(highByte);
    Wire.endTransmission();
}

// ═════════════════════════════════════════════════════════════════════════════
// INIT HELPERS
// ═════════════════════════════════════════════════════════════════════════════
void initMdacSpiPins() {
    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.setSCK(13);
    SPI.begin();
    pinMode(CS_MDAC_FC, OUTPUT);
    pinMode(CS_MDAC_BT, OUTPUT);
    digitalWrite(CS_MDAC_FC, HIGH);
    digitalWrite(CS_MDAC_BT, HIGH);
}

void initBatteryChargerI2cPins() {
    Wire.setSDA(18);
    Wire.setSCL(19);
    Wire.begin();
}

void initEscUartPins() {
    Serial1.setRX(RX);
    Serial1.setTX(TX);
    Serial1.begin(115200);
    vesc.setSerialPort(&Serial1);
}

void initMdacOutputs() {
    setDroopMdac(k_eq / 0.5f / K_sns / A_v,
                 k_eq / 0.5f / K_sns / A_v);
}

void initBatteryCharger() {
    digitalWrite(CHARGER_ENABLE, LOW);
}

void initEsc() {
    vesc.setCurrent(0);
}


// ═════════════════════════════════════════════════════════════════════════════
// WHEEL SPEED (encoder)
// ═════════════════════════════════════════════════════════════════════════════
void updateWheelSpeed() {
    static uint32_t lastMicros = 0;
    static int32_t  index      = 0;

    const int averagingTime = 10000;
    const int arraySize = (int)ceil((float)averagingTime / sampleTime);
    static int posArr[200]  = {0};   // sized for averagingTime/sampleTime = 200
    static int timeArr[200] = {0};

    uint32_t now     = micros();
    uint32_t dtMicros = now - lastMicros;
    if (dtMicros < (uint32_t)sampleTime) return;
    lastMicros = now;

    noInterrupts();
    int32_t pos = encoderPos;
    interrupts();

    posArr[index]  = pos;
    timeArr[index] = now;
    if (index < arraySize - 1) index++;
    else index = 0;

    int dt  = now - timeArr[(index + 1) % arraySize];
    float dtSec = dt * 1e-6f;
    int dx  = pos - posArr[(index + 1) % arraySize];

    if (dtSec < 1e-6f) return;
    float flyWheelSpeedRpm = (dx / ENCODER_COUNTS_PER_REV) * (60.0f / dtSec);
    v_actual = flyWheelSpeedRpm * flyWheelRadius / 60.0f;
}


// ═════════════════════════════════════════════════════════════════════════════
// ENCODER ISRs
// ═════════════════════════════════════════════════════════════════════════════
void doEncoderA() {
    pinA_read = digitalRead(ENC_A);
    pinB_read = digitalRead(ENC_B);

    if ((pinA_read == 1) && (pinB_read == 1) && BfirstUp) {
        encoderPos--;
        AfirstUp = 0; BfirstUp = 0;
    } else if ((pinA_read == 1) && (pinB_read == 0)) {
        AfirstUp = 1;
    }

    if ((pinA_read == 0) && (pinB_read == 0) && BfirstDown) {
        encoderPos--;
        AfirstDown = 0; BfirstDown = 0;
    } else if ((pinA_read == 0) && (pinB_read == 1)) {
        AfirstDown = 1;
    }
}

void doEncoderB() {
    pinA_read = digitalRead(ENC_A);
    pinB_read = digitalRead(ENC_B);

    if ((pinA_read == 1) && (pinB_read == 1) && AfirstUp) {
        encoderPos++;
        AfirstUp = 0; BfirstUp = 0;
    } else if ((pinA_read == 0) && (pinB_read == 1)) {
        BfirstUp = 1;
    }

    if ((pinA_read == 0) && (pinB_read == 0) && AfirstDown) {
        encoderPos++;
        AfirstDown = 0; BfirstDown = 0;
    } else if ((pinA_read == 1) && (pinB_read == 0)) {
        BfirstDown = 1;
    }
}
