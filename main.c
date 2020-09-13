/*********  Copyright (c) 2018-2020, a5021 ************************************/

#if defined(__clang__)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wuninitialized"
#endif

#include "nrf.h"

#if defined(__clang__)
  #pragma clang diagnostic pop
#endif


#ifdef USE_UART
  #include <stdio.h>
  #define UART_TX_PIN                     6
#endif

#define NRF_FREQ_CHANNEL                  99
#define TX_PERIOD                         60 

#define SDA_PIN                           26
#define SCL_PIN                           27

#define NRF_TWIMx                         NRF_TWIM1
#define NRF_TWIx                          NRF_TWI1
#define NRF_TIMERy                        NRF_TIMER2

#define WAIT_FOR_EVENT(EVT)               while((EVT) == 0); EVT = 0

#define TWI_ERROR                         0
#define TWI_OK                            !TWI_ERROR

#define CRCINIT0                          0x0UL
#define CRCPOLY0                          0x0UL
#define CRCINIT8                          0xFFUL
#define CRCPOLY8                          0x107UL
#define CRCINIT16                         0xFFFFUL
#define CRCPOLY16                         0x11021UL
/* width=24 poly=0x00065b init=0x555555 refin=true refout=true xorout=0x000000 check=0xc25a56 residue=0x000000 name="CRC-24/BLE" */
#define CRCINIT24                         0x555555UL
#define CRCPOLY24                         0x00065bUL

/***         allowed value for `BITS' is: 0, 8, 16, 24                                      */
#define NRF_RADIO_SET_CRC(BITS)           NRF_RADIO->CRCCNF = BITS / 8;                      \
                                          NRF_RADIO->CRCINIT = CRCINIT ## BITS;              \
                                          NRF_RADIO->CRCPOLY = CRCPOLY ## BITS

#define BME280_I2C_ADDR_PRIM              0x76

#define BME280_CHIP_ID                    0x60
#define BME280_RESET_CMD                  0xB6

#define BME280_FORCED_MODE                0x01

#define BME280_FILTER_POS                 0x02
#define BME280_SENSOR_MODE_POS            0x00
#define BME280_CTRL_HUM_POS               0x00
#define BME280_CTRL_PRESS_POS             0x02
#define BME280_CTRL_TEMP_POS              0x05

#define BME280_NO_OVERSAMPLING            0x00
#define BME280_OVERSAMPLING_1X            0x01
#define BME280_OVERSAMPLING_2X            0x02
#define BME280_OVERSAMPLING_4X            0x03
#define BME280_OVERSAMPLING_8X            0x04
#define BME280_OVERSAMPLING_16X           0x05

#define BME280_FILTER_COEFF_OFF           0x00
#define BME280_FILTER_COEFF_2             0x01
#define BME280_FILTER_COEFF_4             0x02
#define BME280_FILTER_COEFF_8             0x03
#define BME280_FILTER_COEFF_16            0x04

typedef enum {
  CHIP_ID_REG                           = 0xD0,
  RESET_REG                             = 0xE0,
  TEMP_PRESS_CALIB_DATA_REG             = 0x88,
  HUMIDITY_CALIB_DATA_REG               = 0xE1,
  CONFIG_REG                            = 0xF5,
  CTRL_HUM_REG                          = 0xF2,
  CTRL_MEAS_REG                         = 0xF4,
  DATA_REG                              = 0xF7
} bme280_reg_addr_t;

typedef enum {
  ID_LEN                                = 1,
  TEMP_PRESS_CALIB_DATA_LEN             = 26,
  HUMIDITY_CALIB_DATA_LEN               = 7,
  P_T_H_DATA_LEN                        = 8
} bme280_len_t;

#if defined(__clang__)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wpadded"
#endif

typedef struct {
  uint16_t T1;
  int16_t  T2;
  int16_t  T3;
  uint16_t P1;
  int16_t  P2;
  int16_t  P3;
  int16_t  P4;
  int16_t  P5;
  int16_t  P6;
  int16_t  P7;
  int16_t  P8;
  int16_t  P9;
  uint8_t  H1;
  int16_t  H2;
  uint8_t  H3;
  int16_t  H4;
  int16_t  H5;
  int8_t   H6;
  int32_t  t_fine;
} bme280_calib_data_t ;

#if defined(__clang__)
  #pragma clang diagnostic pop
#endif


#define PRESS_EXP(D) ((uint32_t)D[0] << 12) | ((uint32_t)D[1] << 4) | (D[2] >> 4)
#define TEMP_EXP(D)  ((uint32_t)D[3] << 12) | ((uint32_t)D[4] << 4) | (D[5] >> 4)
#define HUM_EXP(D)   ((uint32_t)D[6] << 8)  | D[7]

static unsigned char i2c_status;

__STATIC_INLINE void init_radio(uint8_t freq, uint8_t *payload) {

  NRF_RADIO->FREQUENCY = freq;                    /* Set RF channel                          */
  NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit;    /* Set data rate and modulation            */
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm; /* Set TX power                        */

  NRF_RADIO->PCNF0 = (                            /* Packet configuration register 0         */
    (6 << RADIO_PCNF0_LFLEN_Pos)              |   /* 6 bits for LENGTH field                 */
    (3 << RADIO_PCNF0_S1LEN_Pos)                  /* 3 bits for PKT ID & NO_ACK field        */
  );

  NRF_RADIO->PCNF1 = (                            /* Packet configuration register 1         */
    (RADIO_PCNF1_ENDIAN_Msk)                  |   /* Most significant bit on air first       */
    (2 << RADIO_PCNF1_BALEN_Pos)              |   /* Base address length in number of bytes  */
    (32 << RADIO_PCNF1_MAXLEN_Pos)                /* Max payload size in bytes               */
  );

        /***   Address consists of one byte PREFIX0 + n bytes BASE0  ***/
        /***   where n is a value of BALEN field in PCNF1 register   ***/

  NRF_RADIO->BASE0 = 0xE7E7E7E7;                  /* Base address 0                          */
  NRF_RADIO->PREFIX0 = 0xE7;                      /* Prefixes bytes for logical addresses    */
  NRF_RADIO->RXADDRESSES = 0x01;                  /* Receive address select                  */

  NRF_RADIO_SET_CRC(16);                          /* Set 16 bit CRC mode                     */

  NRF_RADIO->PACKETPTR = (uint32_t) payload;      /* Set TX buffer pointer                   */
  NRF_RADIO->TASKS_TXEN = 1;                      /* Enable RADIO in TX mode                 */
}


__STATIC_INLINE void init_twi(uint8_t twi_addr) {
  NRF_TWIx->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos; /* Enable I2C       */

  NRF_TWIx->ADDRESS = twi_addr;                   /* Set I2C slave device (sensor) address   */
  NRF_TWIx->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K400;  /* Set 400kHz I2C mode                */
  NRF_TWIx->PSELSDA = SDA_PIN;                    /* Define SDA pin                          */
  NRF_TWIx->PSELSCL = SCL_PIN;                    /* Define SCL pin                          */
}


__STATIC_INLINE void init_rtc(void) {
  NRF_RTC2->PRESCALER = 1;                        /* freq = 32768 / 2                        */
  NRF_RTC2->CC[0] = 16384 * TX_PERIOD;            /* sleep period between data sendings      */
  NRF_RTC2->CC[1] = 135;                          /* short sleep while data converting       */
  NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk | RTC_INTENSET_COMPARE1_Msk;
  NRF_RTC2->EVENTS_COMPARE[1] =                   /* reset compare 1 event flag              */
  NRF_RTC2->EVENTS_COMPARE[0] = 0;                /* reset compare 0 event flag              */
  NVIC_ClearPendingIRQ(RTC2_IRQn);
  // do not use NVIC_EnableIRQ(RTC2_IRQn); !!!
}


#ifdef USE_UART
  void __STATIC_INLINE init_uart(void) {
    NRF_UART0->PSELTXD       = UART_TX_PIN;
    NRF_UART0->BAUDRATE      = UART_BAUDRATE_BAUDRATE_Baud9600;
    NRF_UART0->ENABLE        = UART_ENABLE_ENABLE_Enabled;
    NRF_UART0->EVENTS_TXDRDY = 0x0UL;
    NRF_UART0->TASKS_STARTTX = 0x1UL;
  }

  #define UART_PUTC(C)	                              \
    NRF_UART0->TXD = C;                               \
    WAIT_FOR_EVENT(NRF_UART0->EVENTS_TXDRDY)

  uint8_t __STATIC_INLINE uart_puts(char *s) {
    while (*s != 0) {
      UART_PUTC(*s++);
    }
    return 1;
  }

  #define PRINTF(...) for(char _[100]; snprintf(_, sizeof(_), __VA_ARGS__), uart_puts(_), 0;)
#endif
    

#define TWI_CHECK_ERR() if (NRF_TWIx->EVENTS_ERROR) { \
  NRF_TWIx->TASKS_STOP = 1;                           \
  WAIT_FOR_EVENT(NRF_TWIx->EVENTS_STOPPED);           \
  i2c_status |= NRF_TWIx->ERRORSRC;                   \
  return TWI_ERROR;                                   \
}

#define TWI_WAIT(FLAG) do {while(!FLAG) TWI_CHECK_ERR();} while(0)


#define TWI_XFER(T, E)                                \
  NRF_TWIx->TASKS_##T = 1;                            \
  TWI_WAIT(NRF_TWIx->EVENTS_##E)


__STATIC_INLINE uint8_t twi_read(uint8_t r, uint8_t *d, uint8_t len) {

  NRF_TWIx->EVENTS_TXDSENT  =
  NRF_TWIx->EVENTS_RXDREADY = 0;

  NRF_TWIx->TXD = r;
  TWI_XFER(STARTTX, TXDSENT);

  if (len == 1) {

    NRF_TWIx->SHORTS = TWI_SHORTS_BB_STOP_Enabled << TWI_SHORTS_BB_STOP_Pos;
    TWI_XFER(STARTRX, RXDREADY);
    *d = (uint8_t)NRF_TWIx->RXD;

  } else {

    NRF_TWIx->SHORTS = TWI_SHORTS_BB_SUSPEND_Enabled << TWI_SHORTS_BB_SUSPEND_Pos;
    NRF_TWIx->TASKS_STARTRX = 1;

    do {
      if (--len == 0) {
        NRF_TWIx->SHORTS = TWI_SHORTS_BB_STOP_Enabled << TWI_SHORTS_BB_STOP_Pos;
      }
      NRF_TWIx->EVENTS_RXDREADY = 0;
      TWI_XFER(RESUME, RXDREADY);

      *d++ = (uint8_t)NRF_TWIx->RXD;
    } while (len);
  }
  WAIT_FOR_EVENT(NRF_TWIx->EVENTS_STOPPED);
  NRF_TWIx->SHORTS = 0;
  return TWI_OK;
}


__STATIC_INLINE uint8_t twi_write(uint8_t r, uint8_t d) {

  NRF_TWIx->TXD = r;
  NRF_TWIx->EVENTS_TXDSENT  = 0;
  TWI_XFER(STARTTX, TXDSENT);
  NRF_TWIx->EVENTS_TXDSENT = 0;
  NRF_TWIx->TXD = d;
  TWI_WAIT(NRF_TWIx->EVENTS_TXDSENT);
  NRF_TWIx->TASKS_STOP = 1;
  WAIT_FOR_EVENT(NRF_TWIx->EVENTS_STOPPED);
  return TWI_OK;
}


__STATIC_INLINE uint8_t bme280_read(const bme280_reg_addr_t r, uint8_t* d, bme280_len_t len) {
  return twi_read((uint8_t)r, d, (uint8_t)len);
}


__STATIC_INLINE uint8_t bme280_write(const bme280_reg_addr_t r, uint8_t d) {
  return twi_write((uint8_t)r, d);
}


__STATIC_INLINE int32_t compensate_temperature(uint32_t u_temp, bme280_calib_data_t *c) {
  int32_t var1, var2, temperature;
  int32_t temperature_min = -4000;
  int32_t temperature_max = 8500;

  var1 = (int32_t)(u_temp / 8) - (int32_t)c->T1 * 2;
  var1 = (var1 * ((int32_t)c->T2)) / 2048;
  var2 = (int32_t)(u_temp / 16) - ((int32_t)c->T1);
  var2 = (((var2 * var2) / 4096) * ((int32_t)c->T3)) / 16384;
  c->t_fine = var1 + var2;
  temperature = (c->t_fine * 5 + 128) / 256;

  if (temperature < temperature_min)
    temperature = temperature_min;
  else if (temperature > temperature_max)
    temperature = temperature_max;

  return temperature;
}


__STATIC_INLINE uint32_t compensate_pressure(uint32_t u_press, bme280_calib_data_t *c) {

  #define pressure_min 3000000UL
  #define pressure_max 11000000UL

  int64_t var1, var2, var3;
  uint32_t pressure;

  var1 = ((int64_t)c->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)c->P6;
  var2 = var2 + ((var1 * (int64_t)c->P5) * 131072);
  var2 = var2 + (((int64_t)c->P4) * 34359738368);
  var1 = ((var1 * var1 * (int64_t)c->P3) / 256) + ((var1 * ((int64_t)c->P2) * 4096));
  var3 = ((int64_t)1) * 140737488355328;
  var1 = (var3 + var1) * ((int64_t)c->P1) / 8589934592;

  /* To avoid divide by zero exception */
  if (var1 != 0) {
    int64_t var4 = 1048576 - u_press;
    var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
    var1 = (((int64_t)c->P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
    var2 = (((int64_t)c->P8) * var4) / 524288;
    var4 = ((var4 + var1 + var2) / 256) + (((int64_t)c->P7) * 16);
    pressure = (uint32_t)(((var4 / 2) * 100) / 128);

    if (pressure < pressure_min)
      pressure = pressure_min;
    else if (pressure > pressure_max)
      pressure = pressure_max;
  } else {
    pressure = pressure_min;
  }
  return pressure;
}


__STATIC_INLINE uint32_t compensate_humidity(uint32_t u_hum,  bme280_calib_data_t *c) {
  int32_t var1, var2, var3, var4, var5;

  var1 = c->t_fine - ((int32_t)76800);
  var2 = (int32_t)(u_hum * 16384);
  var3 = (int32_t)(((int32_t)c->H4) * 1048576);
  var4 = ((int32_t)c->H5) * var1;
  var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
  var2 = (var1 * ((int32_t)c->H6)) / 1024;
  var3 = (var1 * ((int32_t)c->H3)) / 2048;
  var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
  var2 = ((var4 * ((int32_t)c->H2)) + 8192) / 16384;
  var3 = var5 * var2;
  var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
  var5 = var3 - ((var4 * ((int32_t)c->H1)) / 16);
  var5 = (var5 < 0 ? 0 : var5);
  var5 = (var5 > 419430400 ? 419430400 : var5);
  uint32_t humidity = (uint32_t)(var5 / 4096);

  if (humidity > 102400)
    humidity = 102400;

  return humidity;
}


__STATIC_INLINE void sleep(void) {
  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NRF_POWER->TASKS_LOWPWR = 1;
  __WFE();
}


__STATIC_INLINE void init_clock(void) {
  /* Start 32 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART    = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

  /* Start low frequency crystal oscillator */
  NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_LFCLKSTART    = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
}


__STATIC_INLINE void init_adc(void) {
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;

  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDD;
  NRF_SAADC->CH[0].CONFIG = (
    SAADC_CH_CONFIG_TACQ_15us << SAADC_CH_CONFIG_TACQ_Pos     |
    SAADC_CH_CONFIG_BURST_Enabled << SAADC_CH_CONFIG_BURST_Pos
  );

  NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over64x;

  NRF_SAADC->RESULT.MAXCNT = 1;

  NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;

  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;  /* Start calibration */
  while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0);
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy <<SAADC_STATUS_STATUS_Pos));
}


__STATIC_INLINE uint32_t measure_vdd(void) {

  volatile uint16_t res;

  NRF_SAADC->RESULT.PTR = (uint32_t) &res;

  NRF_SAADC->EVENTS_DONE = 0;

  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;

  NRF_SAADC->TASKS_START = 1;              // Start the SAADC
  while (NRF_SAADC->EVENTS_STARTED == 0);  // Wait for STARTED event
  NRF_SAADC->EVENTS_STARTED = 0;           // Reset event flag

  NRF_SAADC->TASKS_SAMPLE = 1;
  while (NRF_SAADC->EVENTS_END == 0);
  NRF_SAADC->EVENTS_END = 0;

  // Disable SAADC
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos;

  return res * 3600 / 4095;
}


#define MASK_SIGN           (0x00000200UL)
#define MASK_SIGN_EXTENSION (0xFFFFFC00UL)
#define READ_TEMP()  (((unsigned)NRF_TEMP->TEMP & MASK_SIGN) != 0) ? (signed)((unsigned)NRF_TEMP->TEMP | MASK_SIGN_EXTENSION) : (NRF_TEMP->TEMP)


int main(void) {

  struct {
    uint8_t  l;
    uint8_t  i;
    uint32_t p : 24;
    int8_t   t0: 8;
    int32_t  t : 16;
    uint32_t h : 16;
    uint16_t v : 16;
  } __attribute__((packed)) payload_buf = {.l = 10, .i = 6};

  init_clock();
  init_adc();
  init_twi(BME280_I2C_ADDR_PRIM);
  init_rtc();

#ifdef USE_UART
  init_uart();
#endif

  SCB->SCR |= SCB_SCR_SEVONPEND_Msk;
  __SEV();
  __WFE();

  bme280_calib_data_t c_data;
  uint8_t buf[TEMP_PRESS_CALIB_DATA_LEN];

  #if defined(__clang__)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wcomma"
  #endif

  while(
    ! bme280_read(CHIP_ID_REG, buf, ID_LEN)                                                ||
    ! (buf[0] == BME280_CHIP_ID)                                                           ||
    ! bme280_write(RESET_REG, BME280_RESET_CMD)                                            ||
    (
      NRF_TWIx->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos ,
      NRF_RTC2->TASKS_START = 1                                              ,

      sleep()                                                                ,

      NRF_RTC2->EVENTS_COMPARE[1] = 0                                        ,
      NRF_RTC2->TASKS_STOP = 1                                               ,
      NRF_RTC2->TASKS_CLEAR = 1                                              ,
      NRF_TWIx->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos  ,
      0
    )                                                                                      ||
    ! bme280_read(TEMP_PRESS_CALIB_DATA_REG, (uint8_t*)&c_data, TEMP_PRESS_CALIB_DATA_LEN) ||
    ! bme280_read(HUMIDITY_CALIB_DATA_REG, buf, HUMIDITY_CALIB_DATA_LEN)                   ||
    (
      c_data.H2 = (int16_t)((int16_t)buf[1] << 8) | buf[0]                   ,
      c_data.H3 = buf[2]                                                     ,
      c_data.H4 = ((int16_t)buf[3] * 16) | (buf[4] & 0x0F)                   ,
      c_data.H5 = ((int16_t)buf[5] * 16) | (buf[4] >> 4)                     ,
      c_data.H6 = (int8_t)buf[6]                                             ,
      0
    )                                                                                      ||
    ! bme280_write(CONFIG_REG, BME280_FILTER_COEFF_OFF << BME280_FILTER_POS)               ||
    ! bme280_write(CTRL_HUM_REG, BME280_OVERSAMPLING_1X << BME280_CTRL_HUM_POS)
  );

  #if defined(__clang__)
    #pragma clang diagnostic pop
  #endif

  init_radio(NRF_FREQ_CHANNEL, (uint8_t*)&payload_buf);

  NRF_RTC2->TASKS_START = 1;

  /* Workaround for PAN_028 rev2.0A anomaly 31 - TEMP: Temperature offset value has to be manually loaded to the TEMP module */
  *(uint32_t *) 0x4000C504 = 0;

  while (1) {

    bme280_write(CTRL_MEAS_REG, (BME280_OVERSAMPLING_1X << BME280_CTRL_TEMP_POS) | (BME280_OVERSAMPLING_1X << BME280_CTRL_PRESS_POS) | BME280_FORCED_MODE);

    sleep();

    NRF_RTC2->EVENTS_COMPARE[1] = 0;
    bme280_read(DATA_REG, buf, P_T_H_DATA_LEN);
    NRF_TWIx->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;

    NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */

    /* Busy wait while temperature measurement is not finished */
    while (NRF_TEMP->EVENTS_DATARDY == 0);
    NRF_TEMP->EVENTS_DATARDY = 0;

    /* Workaround for PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register. */
    payload_buf.t0 = (int8_t)(READ_TEMP() / 4);

    /* Workaround for PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs. */
    NRF_TEMP->TASKS_STOP = 1; /* Stop the temperature measurement. */

    payload_buf.t = compensate_temperature(TEMP_EXP(buf), &c_data);
    payload_buf.p = compensate_pressure(PRESS_EXP(buf), &c_data);
    payload_buf.h = compensate_humidity(HUM_EXP(buf),  &c_data) / 10;
    payload_buf.i = ((((payload_buf.i >> 1) + 1) << 1) & 0x06) | 1;

    payload_buf.v = (uint16_t) measure_vdd();

#ifdef USE_UART
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled;
    NRF_UART0->TASKS_STARTTX = 1;
    PRINTF("%d.%02uC,\t\t %u.%02u Pa / %u.%02u mmHg,\t %u.%02u%%\t%u\t0x%02X\r\n", payload_buf.t / 100, (unsigned) payload_buf.t % 100, payload_buf.p / 100, payload_buf.p % 100, payload_buf.p / 13332, payload_buf.p % 13332 * 100 / 13332, payload_buf.h / 1000, payload_buf.h % 1000, payload_buf.i, NRF_CLOCK->HFCLKSTAT);
#endif

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;

    WAIT_FOR_EVENT(NRF_RADIO->EVENTS_END);

#ifdef USE_UART
    static unsigned a_cnt;
    PRINTF("TX Loop count = %u  \n", ++a_cnt);
    NRF_UART0->TASKS_STOPTX = 1;
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Disabled;
#endif

    NRF_RADIO->TASKS_DISABLE = 1;
    WAIT_FOR_EVENT(NRF_RADIO->EVENTS_DISABLED);

    NRF_CLOCK->TASKS_HFCLKSTOP = 1;
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

    sleep();

    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

    NRF_TWIx->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    NRF_RTC2->TASKS_CLEAR = 1;
  }
}
