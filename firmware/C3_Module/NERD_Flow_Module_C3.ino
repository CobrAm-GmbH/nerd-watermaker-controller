#include <Arduino.h>
#include <ModbusRTU.h>

// --------------------------------------------------
// CONFIG
// --------------------------------------------------
static const uint8_t FLOW_PIN = 2;

static const uint8_t UART_TX_PIN = 21;
static const uint8_t UART_RX_PIN = 20;
static const uint8_t RS485_DE_RE_PIN = 10;

static const uint8_t FLOW_SLAVE_ID = 20;
static const uint32_t RS485_BAUD = 9600;

static const unsigned long FLOW_UPDATE_MS = 1000UL;

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
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowPulseISR, RISING);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);   // receive mode

  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  mb.begin(&RS485Serial, RS485_DE_RE_PIN);
  mb.slave(FLOW_SLAVE_ID);

  mb.addIreg(0, 0);
  mb.addIreg(1, 0);
  mb.addIreg(2, 0);
  mb.addIreg(3, 0);

  lastFlowCalcMs = millis();
}

// --------------------------------------------------
// LOOP
// --------------------------------------------------
void loop()
{
  mb.task();

  unsigned long now = millis();

  if ((now - lastFlowCalcMs) >= FLOW_UPDATE_MS) {
    lastFlowCalcMs = now;

    noInterrupts();
    uint32_t pulses = isrPulseCount;
    isrPulseCount = 0;
    interrupts();

    windowPulses = pulses;
    totalPulses += pulses;

    currentHz = (float)pulses;   // 1 second window => pulses/sec = Hz

    regHz_x100   = (uint16_t)(currentHz * 100.0f + 0.5f);
    regWindow    = (uint16_t)(windowPulses & 0xFFFF);
    regTotalLow  = (uint16_t)(totalPulses & 0xFFFF);
    regTotalHigh = (uint16_t)((totalPulses >> 16) & 0xFFFF);

    mb.Ireg(0, regHz_x100);
    mb.Ireg(1, regWindow);
    mb.Ireg(2, regTotalLow);
    mb.Ireg(3, regTotalHigh);
  }
}