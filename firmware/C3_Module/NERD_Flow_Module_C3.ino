// ==================================================
// N.E.R.D. FLOW SLAVE C3 - V6
// ==================================================
//
// ESP32-C3 remote Modbus RTU flowmeter slave
// for N.E.R.D. Watermaker Controller.
//
// Architecture:
// Hall Flow Sensor -> ESP32-C3 -> RS485 Modbus RTU -> ESP32-S3 Main Controller
//
// --------------------------------------------------
// V6 CHANGES / NOTES
// --------------------------------------------------
//
// - Flow input moved from GPIO2 to GPIO3
//   to avoid random startup issues during boot.
//
// - Added startup stabilization delay before
//   attaching interrupt to avoid Hall sensor
//   activity during ESP32 boot sequence.
//
// - Flow interrupt now uses FALLING edge
//   instead of RISING.
//
// - GPIO configured as INPUT instead of
//   INPUT_PULLUP because Hall signal is already
//   conditioned with external resistor divider.
//
// - Added clean interrupt counter reset before
//   enabling interrupt.
//
// - Uses Hall flow sensor with external
//   5V -> 3.3V resistor divider:
//
//       R1 = 10k
//       R2 = 18k
//
// --------------------------------------------------
// Inspiration / References
// --------------------------------------------------
//
// Some flowmeter handling concepts inspired by:
//
// https://github.com/hoeken/brineomatic-firmware
//
// Specifically:
// - INPUT mode
// - FALLING edge interrupt handling
// - delayed/stabilized startup approach
//
// --------------------------------------------------
// Notes
// --------------------------------------------------
//
// Flow measurement is updated every 1 second
// and exposed via Modbus holding/input registers:
//
// reg 0 = Hz x100
// reg 1 = pulses in last window
// reg 2 = total pulses low word
// reg 3 = total pulses high word
//
// ==================================================

#include <Arduino.h>
#include <ModbusRTU.h>

// --------------------------------------------------
// CONFIG
// --------------------------------------------------
static const uint8_t FLOW_PIN = 3;

static const uint8_t UART_TX_PIN = 21;
static const uint8_t UART_RX_PIN = 20;
static const uint8_t RS485_DE_RE_PIN = 10;

static const uint8_t FLOW_SLAVE_ID = 20;
static const uint32_t RS485_BAUD = 9600;

static const unsigned long FLOW_UPDATE_MS = 1000UL;

// Debug
static const uint32_t SERIAL_BAUD = 115200;
static const bool DEBUG_SERIAL = true;

// --------------------------------------------------
// MODBUS
// --------------------------------------------------
static HardwareSerial RS485Serial(1);
static ModbusRTU mb;

// --------------------------------------------------
// FLOW STATE
// --------------------------------------------------
volatile uint32_t isrPulseCount = 0;

static unsigned long lastFlowCalcMs = 0;

static float currentHz = 0.0f;
static uint32_t windowPulses = 0;
static uint32_t totalPulses = 0;

// --------------------------------------------------
// MODBUS INPUT REGISTERS
// reg 0 = Hz x 100
// reg 1 = pulses in last window
// reg 2 = total pulses low word
// reg 3 = total pulses high word
// --------------------------------------------------
static uint16_t regHz_x100   = 0;
static uint16_t regWindow    = 0;
static uint16_t regTotalLow  = 0;
static uint16_t regTotalHigh = 0;

// --------------------------------------------------
// ISR
// --------------------------------------------------
void IRAM_ATTR flowPulseISR()
{
  isrPulseCount++;
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------
void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("=================================");
  Serial.println("ESP32-C3 Flow Pulse Modbus Slave");
  Serial.println("=================================");
  Serial.print("Flow pin: ");
  Serial.println(FLOW_PIN);
  Serial.print("RS485 RX pin: ");
  Serial.println(UART_RX_PIN);
  Serial.print("RS485 TX pin: ");
  Serial.println(UART_TX_PIN);
  Serial.print("RS485 DE/RE pin: ");
  Serial.println(RS485_DE_RE_PIN);
  Serial.print("Modbus Slave ID: ");
  Serial.println(FLOW_SLAVE_ID);
  Serial.print("RS485 baud: ");
  Serial.println(RS485_BAUD);
  Serial.println("---------------------------------");

  // --------------------------------------------------
  // FLOW INPUT INIT
  // --------------------------------------------------

  pinMode(FLOW_PIN, INPUT);

  // Ignore any Hall sensor activity during boot/startup
  delay(5000);

  // Read once to settle GPIO input state
  (void)digitalRead(FLOW_PIN);

  // Start with a clean pulse counter
  noInterrupts();
  isrPulseCount = 0;
  interrupts();

  // Attach interrupt only after stabilization
  attachInterrupt(
      digitalPinToInterrupt(FLOW_PIN),
      flowPulseISR,
      FALLING
  );

  // --------------------------------------------------
  // RS485
  // --------------------------------------------------

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);   // receive mode

  RS485Serial.begin(
      RS485_BAUD,
      SERIAL_8N1,
      UART_RX_PIN,
      UART_TX_PIN
  );

  mb.begin(&RS485Serial, RS485_DE_RE_PIN);
  mb.slave(FLOW_SLAVE_ID);

  // --------------------------------------------------
  // MODBUS INPUT REGISTERS
  // --------------------------------------------------

  mb.addIreg(0, 0);
  mb.addIreg(1, 0);
  mb.addIreg(2, 0);
  mb.addIreg(3, 0);

  lastFlowCalcMs = millis();

  Serial.println("Setup complete. Waiting for pulses...");
  Serial.println();
}
// --------------------------------------------------
// LOOP
// --------------------------------------------------
void loop()
{
  mb.task();

  unsigned long now = millis();

  if ((now - lastFlowCalcMs) >= FLOW_UPDATE_MS) {
    unsigned long elapsedMs = now - lastFlowCalcMs;
    lastFlowCalcMs = now;

    noInterrupts();
    uint32_t pulses = isrPulseCount;
    isrPulseCount = 0;
    interrupts();

    windowPulses = pulses;
    totalPulses += pulses;

    currentHz = ((float)pulses * 1000.0f) / (float)elapsedMs;

    regHz_x100   = (uint16_t)(currentHz * 100.0f + 0.5f);
    regWindow    = (uint16_t)(windowPulses & 0xFFFF);
    regTotalLow  = (uint16_t)(totalPulses & 0xFFFF);
    regTotalHigh = (uint16_t)((totalPulses >> 16) & 0xFFFF);

    mb.Ireg(0, regHz_x100);
    mb.Ireg(1, regWindow);
    mb.Ireg(2, regTotalLow);
    mb.Ireg(3, regTotalHigh);

    if (DEBUG_SERIAL) {
      Serial.print("Elapsed ms: ");
      Serial.print(elapsedMs);

      Serial.print(" | Window pulses: ");
      Serial.print(windowPulses);

      Serial.print(" | Hz: ");
      Serial.print(currentHz, 2);

      Serial.print(" | Hz x100: ");
      Serial.print(regHz_x100);

      Serial.print(" | Total pulses: ");
      Serial.print(totalPulses);

      Serial.print(" | Regs: [0]=");
      Serial.print(regHz_x100);

      Serial.print(" [1]=");
      Serial.print(regWindow);

      Serial.print(" [2]=");
      Serial.print(regTotalLow);

      Serial.print(" [3]=");
      Serial.println(regTotalHigh);
    }
  }
}