#include "Arduino.h"
#include "pinDefinitions.h"

/* wiring_analog variables definition */
/* Flag to indicate whether the ADC config has been changed from the default one */
bool isAdcConfigChanged = false;

// /* 
//  * Configuration used for all the active ADC channels, it is initialized with the mbed default values
//  * When it is changed, all the ADC channels are reconfigured accordingly 
//  */
// analogin_config_t adcCurrentConfig = {
//     .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
//     .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
//     .gain       = NRF_SAADC_GAIN1_4,
//     .reference  = NRF_SAADC_REFERENCE_VDD4,
//     .acq_time   = NRF_SAADC_ACQTIME_10US,
//     .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
//     .burst      = NRF_SAADC_BURST_DISABLED,
//     .pin_p      = NRF_SAADC_INPUT_DISABLED,
//     .pin_n      = NRF_SAADC_INPUT_DISABLED
// };

void analogReference(uint8_t mode)
{
  nrf_saadc_reference_t reference = NRF_SAADC_REFERENCE_VDD4;
  nrf_saadc_gain_t gain = NRF_SAADC_GAIN1_4;
  if (mode == AR_VDD) {
    reference = NRF_SAADC_REFERENCE_VDD4;
    gain = NRF_SAADC_GAIN1_4;
  } else if (mode == AR_INTERNAL) {
    reference = NRF_SAADC_REFERENCE_INTERNAL;
    gain = NRF_SAADC_GAIN1;
  } else if (mode == AR_INTERNAL1V2) {
    reference = NRF_SAADC_REFERENCE_INTERNAL;
    gain = NRF_SAADC_GAIN1_2;
  } else if (mode == AR_INTERNAL2V4) {
    reference = NRF_SAADC_REFERENCE_INTERNAL;
    gain = NRF_SAADC_GAIN1_4;
  }
  adcCurrentConfig.reference = reference;
  adcCurrentConfig.gain = gain;
  analogUpdate();
}

void analogAcquisitionTime(uint8_t time)
{
  nrf_saadc_acqtime_t acqTime = NRF_SAADC_ACQTIME_10US;
  if (time == AT_3_US) {
    acqTime = NRF_SAADC_ACQTIME_3US;
  } else if (time == AT_5_US) {
    acqTime = NRF_SAADC_ACQTIME_5US;
  } else if (time == AT_10_US) {
    acqTime = NRF_SAADC_ACQTIME_10US;
  } else if (time == AT_15_US) {
    acqTime = NRF_SAADC_ACQTIME_15US;
  } else if (time == AT_20_US) {
    acqTime = NRF_SAADC_ACQTIME_20US;
  } else if (time == AT_40_US) {
    acqTime = NRF_SAADC_ACQTIME_40US;
  }
  adcCurrentConfig.acq_time = acqTime;
  analogUpdate();
}

AnalogPinDescription g_AAnalogPinDescription[] = {
    // A0 - A5
  { P0_30, NULL },    // A0
  { P0_29, NULL },    // A1
  { P0_5,  NULL },    // A2
  { P0_31, NULL },    // A3
  { P0_2,  NULL },    // A4
  { P0_28, NULL }     // A5
};

PinDescription g_APinDescription[] = {
  // Pinout
  { P0_24, NULL, NULL, NULL },     // D0/RX
  { P0_25, NULL, NULL, NULL },     // D1/TX
  { P1_10, NULL, NULL, NULL },     // D2/FREE
  { P0_6,  NULL, NULL, NULL },     // D3/SDA
  { P0_26, NULL, NULL, NULL },     // D4/SCL
  { P1_1,  NULL, NULL, NULL },     // D5/GPIO6
  { P0_4,  NULL, NULL, NULL },     // D6/GPIO5
  { P0_20, NULL, NULL, NULL },     // D7/PICO
  { P0_21, NULL, NULL, NULL },     // D8/POCI
  { P0_16, NULL, NULL, NULL },     // D9/GPIO4
  { P0_15, NULL, NULL, NULL },     // D10/GPIO3
  { P0_14, NULL, NULL, NULL },     // D11/GPIO2
  { P0_13, NULL, NULL, NULL },     // D12/GPIO1
  { P0_27, NULL, NULL, NULL },     // D13/GPIO0

  // A0 - A7
  { P0_30, NULL, NULL, NULL },     // A0
  { P0_29, NULL, NULL, NULL },     // A1
  { P0_5,  NULL, NULL, NULL },     // A2
  { P0_31, NULL, NULL, NULL },     // A3
  { P0_2,  NULL, NULL, NULL },     // A4
  { P0_28, NULL, NULL, NULL },     // A5

  // LEDs
  { P1_8,  NULL, NULL, NULL },     // LED RGB DIN
  { P1_14, NULL, NULL, NULL },     // LED STAT

  // ISM IMU Interrupts
  { P1_12, NULL, NULL, NULL },     // IMU_INT1
  { P1_13, NULL, NULL, NULL },     // IMU_INT2

  // Battery Fuel Guage Interrupts
  { P1_11, NULL, NULL, NULL },     // !BATT_ALRT!

  // SD Card
  { P0_17, NULL, NULL, NULL },     // SD_!CS!
  { P1_3,  NULL, NULL, NULL },     // SD_DET
  { P0_19, NULL, NULL, NULL },     // SCK

  // QWIIC Power Enable
  { P1_15, NULL, NULL, NULL },     // QWIIC_!PEN!

};

extern "C" {
  unsigned int PINCOUNT_fn() {
    return (sizeof(g_APinDescription) / sizeof(g_APinDescription[0]));
  }
}

#include "nrf_rtc.h"
#include "nrf_uarte.h"
#include "nrf_uart.h"

void initVariant() {
//   // turn power LED on
//   pinMode(LED_PWR, OUTPUT);
//   digitalWrite(LED_PWR, HIGH);

//   // Errata Nano33BLE - I2C pullup is controlled by the SWO pin.
//   // Configure the TRACEMUX to disable routing SWO signal to pin.
//   NRF_CLOCK->TRACECONFIG = 0;

  // FIXME: bootloader enables interrupt on COMPARE[0], which we don't handle
  // Disable it here to avoid getting stuck when OVERFLOW irq is triggered
  nrf_rtc_event_disable(NRF_RTC1, NRF_RTC_INT_COMPARE0_MASK);
  nrf_rtc_int_disable(NRF_RTC1, NRF_RTC_INT_COMPARE0_MASK);

  // Enable Qwiic Power by default
  pinMode(PIN_QWIIC_PWR_EN, OUTPUT);

  digitalWrite(PIN_SD_CS, HIGH);

  // Disable UARTE0 which is initially enabled by the bootloader
  nrf_uarte_task_trigger(NRF_UARTE0, NRF_UARTE_TASK_STOPRX); 
  while (!nrf_uarte_event_check(NRF_UARTE0, NRF_UARTE_EVENT_RXTO)) ; 
  NRF_UARTE0->ENABLE = 0; 
  NRF_UART0->ENABLE = 0; 

  NRF_PWM_Type* PWM[] = {
    NRF_PWM0, NRF_PWM1, NRF_PWM2
#ifdef NRF_PWM3
    ,NRF_PWM3
#endif
  };

  for (unsigned int i = 0; i < (sizeof(PWM)/sizeof(PWM[0])); i++) {
    PWM[i]->ENABLE = 0;
    PWM[i]->PSEL.OUT[0] = 0xFFFFFFFFUL;
  } 
}

#ifdef SERIAL_CDC

static void utox8(uint32_t val, uint8_t* s) {
  for (int i = 0; i < 16; i=i+2) {
    int d = val & 0XF;
    val = (val >> 4);

    s[15 - i -1] = d > 9 ? 'A' + d - 10 : '0' + d;
    s[15 - i] = '\0';
  }
}

uint8_t getUniqueSerialNumber(uint8_t* name) {
  #define SERIAL_NUMBER_WORD_0  NRF_FICR->DEVICEADDR[1]
  #define SERIAL_NUMBER_WORD_1  NRF_FICR->DEVICEADDR[0]

  utox8(SERIAL_NUMBER_WORD_0, &name[0]);
  utox8(SERIAL_NUMBER_WORD_1, &name[16]);

  return 32;
}

void _ontouch1200bps_() {
  __disable_irq();
  NRF_POWER->GPREGRET = DFU_MAGIC_SERIAL_ONLY_RESET;
  NVIC_SystemReset();
}

#endif
