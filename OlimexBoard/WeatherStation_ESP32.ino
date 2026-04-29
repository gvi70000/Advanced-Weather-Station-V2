/**
 * @file WeatherStation_ESP32.ino
 * @brief Olimex ESP32-POE2 (WROVER-E-N4R8) – Weather Station Gateway
 * @details Bridges an STM32 sensor array and a SparkFun LG290P RTK GNSS module
 * to an Ethernet network via MQTT and NTRIP.
 * @date 2026-04-28
 *
 * Fixes applied (2026-04-17):
 *  FIX-1  processSTMbyte()  – Clear static `prev` after sync detected.
 *  FIX-2  ntripLoop()       – ntripHdrBuf[] moved to file scope, zeroed on connect.
 *  FIX-3  mqttCallback()    – Retry loop when PIN_STM_IS_TX is HIGH.
 *  FIX-4  ntripLoop()       – Reset ntripLastGGA on new connection.
 *  FIX-5  gpsHardwareReset()– Post-reset delay extended to 2500 ms.
 *  FIX-6  packWindHeight()  – Centralised IEEE-754 memcpy helper.
 *  FIX-7  pub() float       – Guard against NaN/Inf before dtostrf().
 *  FIX-8  publishGPSStatus()– Null/empty check on NMEA sentence pointer.
 *  FIX-9  mqtt buffer       – setBufferSize increased to 512 bytes.
 *  FIX-10 OTA reconnect     – Reset otaStarted on Ethernet disconnect.
 *
 * Protocol alignment with STM32 main.c (2026-04-28):
 *  PROTO-1  Defined UART_FRAME_SOF_B0/B1, CAL_SOF_B0/B1, CAL_FRAME_SIZE,
 *           CAL_PAYLOAD_SIZE, Cal_Payload_t — previously undefined/missing.
 *  PROTO-2  Removed broken WIND_SOF/CMD_SOF raw-byte approach. All ESP→STM
 *           frames now use UART_TX_Frame_t: [0x23][FrameType][Wind_Input_t(16)][CRC16(2)].
 *  PROTO-3  sendWindInputToSTM() rewritten with FrameType = UART_FRAME_CAL.
 *  PROTO-4  sendCmdToSTM() rewritten: FrameType encodes command type,
 *           Height field carries the float value (matching STM HAL_UART_RxCpltCallback).
 *  PROTO-5  AS3935 interrupt type decoded from DistanceEstimation bits [7:4].
 *           New MQTT topic weather/LightningType publishes INT_NH/INT_D/INT_L.
 *           Existing weather/DistanceEstimation masked to bits [5:0].
 */

#ifndef ETH_PHY_TYPE
  #define ETH_PHY_TYPE   ETH_PHY_LAN8720
  #define ETH_PHY_ADDR   0
  #define ETH_PHY_MDC    23
  #define ETH_PHY_MDIO   18
  #define ETH_PHY_POWER  12
  #define ETH_CLK_MODE   ETH_CLOCK_GPIO0_OUT
#endif

#include <ETH.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <SparkFun_LG290P_GNSS.h>
#include "mbedtls/base64.h"
#include <math.h>   // isfinite()

// ==========================================================================
// USER CONFIGURATION
// ==========================================================================

static const char* MQTT_SERVER    = "10.0.1.6";
static const uint16_t MQTT_PORT   = 1883;
static const char* MQTT_CLIENT_ID = "WeatherStation";
static const char* MQTT_USER      = "";
static const char* MQTT_PASS      = "";
static const char* OTA_HOSTNAME   = "WeatherStation";
static const char* OTA_PASSWORD   = "ota1234";

static const char* NTRIP_HOST               = "";
static const uint16_t NTRIP_PORT            = 2101;
static const char* NTRIP_MOUNTPOINT         = "";
static const char* NTRIP_USER               = "";
static const char* NTRIP_PASS               = "";
static const uint32_t NTRIP_GGA_INTERVAL_MS = 10000UL;

// ==========================================================================
// MQTT TOPICS
// ==========================================================================
/** @name MQTT Publication Topics */
///@{
static const char* TOPIC_TEMP     = "weather/Temperature";
static const char* TOPIC_RH       = "weather/RH";
static const char* TOPIC_PRES     = "weather/Pressure";
static const char* TOPIC_DP       = "weather/DewPoint";
static const char* TOPIC_SP       = "weather/SoundSpeed";
static const char* TOPIC_UVT      = "weather/UVTemperature";
static const char* TOPIC_UVA      = "weather/UVA";
static const char* TOPIC_UVB      = "weather/UVB";
static const char* TOPIC_UVC      = "weather/UVC";
static const char* TOPIC_LIGHT_C  = "weather/ClearChannel";
static const char* TOPIC_LIGHT_R  = "weather/RedChannel";
static const char* TOPIC_LIGHT_G  = "weather/GreenChannel";
static const char* TOPIC_LIGHT_B  = "weather/BlueChannel";
static const char* TOPIC_LIGHT_FS = "weather/FullSpectrum";
static const char* TOPIC_LIGHT_IR = "weather/Infrared";
static const char* TOPIC_LIGHT_VI = "weather/Visible";
static const char* TOPIC_LIGHT_LX = "weather/Lux";
static const char* TOPIC_LIGHT_IR = "weather/TCS_IR";
static const char* TOPIC_LIGHT_CT = "weather/CCT";
static const char* TOPIC_LIGHT_IX = "weather/Irradiance";
static const char* TOPIC_AQI      = "weather/AQI";
static const char* TOPIC_TVOC     = "weather/TVOC";
static const char* TOPIC_ECO      = "weather/eCO2";
static const char* TOPIC_WS       = "weather/WindSpeed";
static const char* TOPIC_WD       = "weather/WindDirection";
static const char* TOPIC_LE       = "weather/LightningEnergy";
static const char* TOPIC_LD       = "weather/DistanceEstimation"; /* bits [5:0] only */
static const char* TOPIC_LT       = "weather/LightningType";      /* INT_NH/INT_D/INT_L */
static const char* TOPIC_WIND_H   = "weather/Wind/Height";
static const char* TOPIC_WIND_D01 = "weather/Wind/PathD01";
static const char* TOPIC_WIND_D12 = "weather/Wind/PathD12";
static const char* TOPIC_WIND_D20 = "weather/Wind/PathD20";
static const char* TOPIC_STATUS   = "weather/status";
///@}

/** @name MQTT Command Topics */
///@{
static const char* CMD_STM_RESET   = "weather/cmd/Reset";
static const char* CMD_TEMP_CALIB  = "weather/cmd/TempCalib";
static const char* CMD_RH_CALIB    = "weather/cmd/RHCalib";
static const char* CMD_WIND_HEIGHT = "weather/cmd/Wind/Height";
static const char* CMD_WIND_D01    = "weather/cmd/Wind/PathD01";
static const char* CMD_WIND_D12    = "weather/cmd/Wind/PathD12";
static const char* CMD_WIND_D20    = "weather/cmd/Wind/PathD20";
static const char* CMD_GPS_HWRESET = "weather/cmd/GPS/HWReset";
static const char* SUB_CMD_WILDCARD = "weather/cmd/#";
///@}

// ==========================================================================
// PIN DEFINITIONS
// ==========================================================================
#define PIN_STM_RX    36  /**< UART RX from STM32 (10k pull-up)       */
#define PIN_STM_TX     4  /**< UART TX to STM32                        */
#define PIN_STM_IS_TX 35  /**< Input: HIGH while STM32 is transmitting */
#define PIN_GPS_RX    34  /**< UART RX from LG290P                     */
#define PIN_GPS_TX    32  /**< UART TX to LG290P                       */
#define PIN_GPS_RST    5  /**< Hardware reset for LG290P (10k pull-up) */

// ==========================================================================
// PROTOCOL CONSTANTS  (must match STM32 main.c exactly)
// ==========================================================================

/* ---- STM32 → ESP  frame SOF bytes ----------------------------------------
 * Sensor data frame:   [0x23][0x4E][Payload(78)][CRC16(2)]  = 82 bytes
 *   SOF pair = {UART_FRAME_SOF_B0, UART_FRAME_SOF_B1}
 *   UART_FRAME_SOF_B1 = 0x4E = sizeof(UART_Payload_t) = 78 — the Length byte
 *   IS the second SOF byte, which is how processSTMbyte() detects frame type 1.
 *
 * Calibration frame:   [0x23][0x24][0x0C][D01(4)][D12(4)][D20(4)][CRC16(2)] = 17 bytes
 *   SOF pair = {CAL_SOF_B0, CAL_SOF_B1}
 * ----------------------------------------------------------------------- */
#define UART_FRAME_SOF_B0  0x23   /* '#' — common first byte for all STM→ESP frames */
#define UART_FRAME_SOF_B1  0x4E   /* Length = 78 = sizeof(UART_Payload_t)            */
#define CAL_SOF_B0         0x23   /* '#'                                              */
#define CAL_SOF_B1         0x24   /* '$' = UART_FRAME_CAL                             */

/* ---- ESP → STM  frame constants ------------------------------------------
 * All commands share the same 20-byte layout:
 *   [0x23][FrameType(1)][Wind_Input_t(16)][CRC16(2)]
 * CRC covers FrameType + Wind_Input_t = 17 bytes.
 * ----------------------------------------------------------------------- */
#define UART_FRAME_START   0x23   /* '#'  first byte of every ESP→STM frame   */
#define UART_FRAME_CAL     0x24   /* '$'  boot config: Height + D01/D12/D20   */
#define UART_FRAME_TMP     0x25   /* '%'  temp reference (Height = float °C)  */
#define UART_FRAME_RH      0x26   /* '&'  RH reference   (Height = float %)   */
#define UART_FRAME_RST     0x27   /* '\'' system reset   (payload ignored)    */
#define UART_FRAME_CAL_REQ 0x28   /* '('  run wind calibration                */

/* ---- Calibration readback frame (STM→ESP, type 2) ------------------------ */
#define CAL_PAYLOAD_SIZE   12     /* 3 × uint32_t (D01, D12, D20)             */
#define CAL_FRAME_SIZE     17     /* SOF(2)+Length(1)+payload(12)+CRC(2)      */

/* ---- AS3935 interrupt type packed into DistanceEstimation bits [7:4] ----- */
#define AS3935_INT_NONE  0x00
#define AS3935_INT_NH    0x01   /* noise-level-too-high                       */
#define AS3935_INT_D     0x04   /* disturber detected                         */
#define AS3935_INT_L     0x08   /* lightning strike                           */
#define AS3935_DECODE_INT(b)   (((b) >> 4) & 0x0F)
#define AS3935_DECODE_DIST(b)  ((b) & 0x3F)

// ==========================================================================
// DATA STRUCTURES
// ==========================================================================

/* ---- Wire structs — must match STM32 UART_Payload_t byte-for-byte -------- */
struct Wind_EnvData_t     { float    Temperature, RH, Pressure, DewPoint, SoundSpeed; } __attribute__((packed));
struct Wind_t             { float    Speed, Direction; }                                __attribute__((packed));
struct AS3935_LightningData_t { uint32_t LightningEnergy; uint8_t DistanceEstimation; } __attribute__((packed));
struct AS7331_DataOut_t   { uint16_t TEMP_C100, UVA, UVB, UVC; }                      __attribute__((packed));
struct ENS160_Data_t      { uint8_t  AQI; uint16_t TVOC, eCO2; }                      __attribute__((packed));
struct TCS34003_RGBC_Data_t { uint16_t ClearChannel, RedChannel, GreenChannel, BlueChannel; } __attribute__((packed));
struct TCS34003_IR_Data_t   { uint16_t IR; }                                                   __attribute__((packed));
struct TCS34003_LightData_t {
    TCS34003_RGBC_Data_t RGBC;   /*  8 B: Clear, Red, Green, Blue (u16 x4) */
    TCS34003_IR_Data_t   IR;     /*  2 B: Infrared channel                 */
    float CCT;                   /*  4 B: Correlated Colour Temperature (K) */
    float Lux;                   /*  4 B: Illuminance (lux)                 */
    float Irradiance;            /*  4 B: Irradiance (uW/cm2)               */
} __attribute__((packed));       /* = 22 B total                            */
struct TSL25911_LightData_t { uint16_t FullSpectrum, Infrared, Visible; float Lux; }  __attribute__((packed));

/**
 * @struct UART_Payload_t
 * @brief 64-byte sensor data payload received from STM32.
 *        Must match STM32 UART_Payload_t exactly (verified by _Static_assert on STM side).
 */
struct UART_Payload_t {
    Wind_EnvData_t        Env;         /* 20 B */
    Wind_t                Wind;        /*  8 B */
    AS3935_LightningData_t Lightning;  /*  5 B */
    AS7331_DataOut_t      UV;          /*  8 B */
    ENS160_Data_t         AirQuality;  /*  5 B */
    TCS34003_LightData_t  LightRGB;    /* 22 B */
    TSL25911_LightData_t  Light;       /* 10 B */
} __attribute__((packed));             /* = 64 B total */

static_assert(sizeof(UART_Payload_t) == 78, "UART_Payload_t size mismatch with STM32");

/**
 * @struct UART_Frame_t
 * @brief Full sensor data frame received from STM32 (68 bytes).
 *        Wire: [0x23][0x40][Payload(64)][CRC16(2)]
 */
struct UART_Frame_t {
    uint8_t        Start;    /* 0x23 */
    uint8_t        Length;   /* 0x40 = 64 */
    UART_Payload_t Payload;
    uint16_t       CR;       /* CRC16 over Payload only */
} __attribute__((packed));

static constexpr size_t FRAME_SIZE = sizeof(UART_Frame_t);  /* 68 */
static_assert(FRAME_SIZE == 82, "UART_Frame_t size mismatch");

/**
 * @struct Cal_Payload_t
 * @brief Calibration payload (12 bytes) inside the STM→ESP calibration frame.
 */
struct Cal_Payload_t {
    uint32_t PathLenD01;
    uint32_t PathLenD12;
    uint32_t PathLenD20;
} __attribute__((packed));

static_assert(sizeof(Cal_Payload_t) == CAL_PAYLOAD_SIZE, "Cal_Payload_t size mismatch");

/**
 * @struct Wind_Input_t
 * @brief Configuration payload shared by all ESP→STM command frames (16 bytes).
 *
 * Height is stored as the raw IEEE-754 bit pattern packed via memcpy.
 * The canonical float lives in g_windHeightFloat — always use packWindHeight().
 * For UART_FRAME_TMP / UART_FRAME_RH the Height field carries the reference
 * float value; other fields are ignored by the STM for those frame types.
 */
struct Wind_Input_t {
    uint32_t Height;      /**< IEEE-754 float bits of altitude (m)  */
    uint32_t PathLenD01;  /**< Acoustic path 0→1 (µm)               */
    uint32_t PathLenD12;  /**< Acoustic path 1→2 (µm)               */
    uint32_t PathLenD20;  /**< Acoustic path 2→0 (µm)               */
} __attribute__((packed));

static constexpr size_t WIND_INPUT_SIZE = sizeof(Wind_Input_t);  /* 16 */

/**
 * @struct UART_TX_Frame_t
 * @brief ESP→STM command frame (20 bytes).
 *        Wire: [0x23][FrameType(1)][Wind_Input_t(16)][CRC16(2)]
 *        CRC covers FrameType + Wind_Input_t = 17 bytes.
 */
struct UART_TX_Frame_t {
    uint8_t      Start;      /* UART_FRAME_START = 0x23          */
    uint8_t      FrameType;  /* 0x24–0x28 — selects STM action   */
    Wind_Input_t Payload;    /* 16 bytes                          */
    uint16_t     CR;         /* CRC16 over FrameType + Payload    */
} __attribute__((packed));

static constexpr size_t CMD_FRAME_SIZE = sizeof(UART_TX_Frame_t);  /* 20 */
static_assert(CMD_FRAME_SIZE == 20, "UART_TX_Frame_t size mismatch");

/* STM TX busy-wait constants.
 * STM sensor frame = 68 bytes @ 115200 baud = 5.9 ms.
 * Wait up to 10 x 2 ms = 20 ms before giving up.
 * Since STM DMA RX is fixed 20-byte circular, commands received during STM TX
 * are not lost — they land correctly in the DMA buffer. The wait is a courtesy
 * to avoid sending during the STM TX completion ISR window.                  */
#define STM_TX_WAIT_RETRIES  10
#define STM_TX_WAIT_DELAY_MS 2

// ==========================================================================
// GLOBALS
// ==========================================================================
static bool eth_connected = false;
static bool otaStarted    = false;

NetworkClient ethClient;
PubSubClient mqtt(ethClient);
NetworkClient ntripClient;

static bool     ntripStreamActive = false;
static uint32_t ntripLastAttempt  = 0;
static uint32_t ntripLastGGA      = 0;
static char     latestGGA[128]    = {0};

// FIX-2: File-scope NTRIP header scan buffer, zeroed on each new connection.
static uint8_t ntripHdrBuf[4] = {0};

LG290P myGNSS;

// g_windInput.Height is the IEEE-754 bit pattern of g_windHeightFloat.
// Never assign g_windInput.Height directly — always use packWindHeight().
static Wind_Input_t g_windInput = {
    .Height     = 0,           // set properly in setup() via packWindHeight()
    .PathLenD01 = 316228UL,
    .PathLenD12 = 316228UL,
    .PathLenD20 = 316228UL
};
static float g_windHeightFloat = 0.0f;

static Preferences prefs;
#define NVS_NS     "windcfg"
#define NVS_HEIGHT "height"
#define NVS_D01    "d01"
#define NVS_D12    "d12"
#define NVS_D20    "d20"

// RX state machine (processSTMbyte)
static uint8_t  rxBuf[FRAME_SIZE];   // largest expected frame is the sensor frame (68 B)
static uint16_t rxLen     = 0;
static bool     syncFound = false;
static uint8_t  frameType = 0;       // 1 = sensor data, 2 = calibration readback

// ==========================================================================
// HELPERS
// ==========================================================================

/**
 * @brief Modbus CRC16 — matches STM32 implementation exactly.
 */
static uint16_t CRC16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xA001u) : (crc >> 1);
        }
    }
    return crc;
}

/**
 * @brief FIX-6: Pack g_windHeightFloat into g_windInput.Height as IEEE-754 bits.
 *        Call whenever g_windHeightFloat changes.
 */
static inline void packWindHeight() {
    memcpy(&g_windInput.Height, &g_windHeightFloat, sizeof(float));
}

/**
 * @brief Load wind calibration and height from NVS.
 */
static void nvsLoad() {
    prefs.begin(NVS_NS, true);
    g_windHeightFloat      = prefs.getFloat(NVS_HEIGHT, 0.0f);
    g_windInput.PathLenD01 = prefs.getUInt(NVS_D01, g_windInput.PathLenD01);
    g_windInput.PathLenD12 = prefs.getUInt(NVS_D12, g_windInput.PathLenD12);
    g_windInput.PathLenD20 = prefs.getUInt(NVS_D20, g_windInput.PathLenD20);
    packWindHeight();  // FIX-6
    prefs.end();
    Serial.printf("[NVS] H=%.1fm D01=%lu D12=%lu D20=%lu µm\n",
                  g_windHeightFloat,
                  (unsigned long)g_windInput.PathLenD01,
                  (unsigned long)g_windInput.PathLenD12,
                  (unsigned long)g_windInput.PathLenD20);
}

/**
 * @brief Save wind calibration and height to NVS.
 */
static void nvsSave() {
    prefs.begin(NVS_NS, false);
    prefs.putFloat(NVS_HEIGHT, g_windHeightFloat);
    prefs.putUInt(NVS_D01, g_windInput.PathLenD01);
    prefs.putUInt(NVS_D12, g_windInput.PathLenD12);
    prefs.putUInt(NVS_D20, g_windInput.PathLenD20);
    prefs.end();
}

/** @name MQTT Publisher Overloads */
///@{
static void pub(const char* topic, const char* value, bool retain = false) {
    if (mqtt.connected()) mqtt.publish(topic, value, retain);
}
/** @brief FIX-7: Skip NaN/Inf. */
static void pub(const char* topic, float v, uint8_t dec = 2) {
    if (!isfinite(v)) return;
    char buf[24]; dtostrf(v, 1, dec, buf); pub(topic, buf);
}
static void pub(const char* topic, uint32_t v) { char buf[16]; snprintf(buf, sizeof(buf), "%lu", (unsigned long)v); pub(topic, buf); }
static void pub(const char* topic, uint16_t v) { pub(topic, (uint32_t)v); }
static void pub(const char* topic, uint8_t  v) { pub(topic, (uint32_t)v); }
///@}

/**
 * @brief Publish current wind configuration to MQTT.
 */
static void publishWindInputs() {
    pub(TOPIC_WIND_H,   g_windHeightFloat, 2);
    pub(TOPIC_WIND_D01, g_windInput.PathLenD01);
    pub(TOPIC_WIND_D12, g_windInput.PathLenD12);
    pub(TOPIC_WIND_D20, g_windInput.PathLenD20);
}

/**
 * @brief Wait for STM32 to finish transmitting its sensor frame.
 *
 * PIN_STM_IS_TX (PA11 on STM, GPIO35 on ESP) is asserted HIGH by STM_START_TX
 * at the start of HAL_UART_Transmit_DMA and deasserted LOW by STM_STOP_TX in
 * HAL_UART_TxCpltCallback when the 68-byte DMA TX completes (~5.9 ms).
 *
 * The STM DMA RX is fixed 20-byte circular — it accepts the ESP command frame
 * independently of STM TX state, so this wait is not strictly required for
 * correctness.  It prevents the ESP sending during the STM TX completion ISR
 * window which could cause a brief processing spike on the STM.
 *
 * If the STM remains busy beyond the timeout (20 ms >> 5.9 ms frame time),
 * we send anyway rather than silently dropping the command.
 */
static void waitForSTMBusFree() {
    for (uint8_t i = 0; i < STM_TX_WAIT_RETRIES; i++) {
        if (digitalRead(PIN_STM_IS_TX) == LOW) return;
        delay(STM_TX_WAIT_DELAY_MS);
    }
    /* Timeout — send anyway; STM DMA RX will still capture the frame */
}

/**
 * @brief Build and send a UART_TX_Frame_t to the STM32.
 * @details Internal helper used by sendWindInputToSTM() and sendCmdToSTM().
 *          Wire: [0x23][FrameType][Wind_Input_t(16)][CRC16(2)] = 20 bytes.
 *          CRC covers FrameType + Wind_Input_t = 17 bytes.
 */
static void sendFrameToSTM(uint8_t frameType, const Wind_Input_t& payload) {
    waitForSTMBusFree();
    UART_TX_Frame_t f;
    f.Start     = UART_FRAME_START;
    f.FrameType = frameType;
    f.Payload   = payload;
    f.CR        = CRC16(reinterpret_cast<const uint8_t*>(&f.FrameType),
                        1 + WIND_INPUT_SIZE);
    Serial1.write(reinterpret_cast<const uint8_t*>(&f), CMD_FRAME_SIZE);
    Serial1.flush();
}

/**
 * @brief Send full wind configuration to STM32 (boot sync + MQTT updates).
 * @details FrameType = UART_FRAME_CAL (0x24).
 *          STM copies the entire Wind_Input_t into calibData.
 */
static void sendWindInputToSTM() {
    packWindHeight();  // FIX-6: ensure Height field is in sync
    sendFrameToSTM(UART_FRAME_CAL, g_windInput);
}

/**
 * @brief Send a command with optional float value to the STM32.
 * @details FrameType selects the command. The float value is packed into the
 *          Height field via memcpy (IEEE-754). Other Wind_Input_t fields are
 *          zeroed and ignored by the STM for UART_FRAME_TMP / RH / RST / CAL_REQ.
 * @param frameType  UART_FRAME_TMP / RH / RST / CAL_REQ
 * @param value      Float value (used for TMP and RH; ignored for RST/CAL_REQ)
 */
static void sendCmdToSTM(uint8_t frameType, float value = 0.0f) {
    Wind_Input_t payload = {};
    memcpy(&payload.Height, &value, sizeof(float));
    sendFrameToSTM(frameType, payload);
}

/**
 * @brief FIX-5: Hardware reset of LG290P GNSS module.
 */
static void gpsHardwareReset() {
    digitalWrite(PIN_GPS_RST, LOW);  delay(200);
    digitalWrite(PIN_GPS_RST, HIGH); delay(2500);  // FIX-5: was 1000 ms
}

// ==========================================================================
// MQTT CALLBACK
// ==========================================================================

/**
 * @brief Handle incoming MQTT commands and dispatch to STM32 or GPS.
 */
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char val[64] = {0};
    uint16_t n = (length < sizeof(val) - 1) ? (uint16_t)length : (uint16_t)(sizeof(val) - 1);
    memcpy(val, payload, n);
    val[n] = '\0';

    char* endPtr;

    if (strcmp(topic, CMD_STM_RESET) == 0) {
        sendCmdToSTM(UART_FRAME_RST);
    }
    else if (strcmp(topic, CMD_TEMP_CALIB) == 0) {
        float v = strtof(val, &endPtr);
        if (endPtr != val) sendCmdToSTM(UART_FRAME_TMP, v);
    }
    else if (strcmp(topic, CMD_RH_CALIB) == 0) {
        float v = strtof(val, &endPtr);
        if (endPtr != val) sendCmdToSTM(UART_FRAME_RH, v);
    }
    else if (strcmp(topic, CMD_WIND_HEIGHT) == 0) {
        float v = strtof(val, &endPtr);
        if (endPtr != val) {
            g_windHeightFloat = v;
            nvsSave(); publishWindInputs(); sendWindInputToSTM();
        }
    }
    else if (strcmp(topic, CMD_WIND_D01) == 0) {
        uint32_t v = strtoul(val, &endPtr, 10);
        if (endPtr != val) { g_windInput.PathLenD01 = v; nvsSave(); publishWindInputs(); sendWindInputToSTM(); }
    }
    else if (strcmp(topic, CMD_WIND_D12) == 0) {
        uint32_t v = strtoul(val, &endPtr, 10);
        if (endPtr != val) { g_windInput.PathLenD12 = v; nvsSave(); publishWindInputs(); sendWindInputToSTM(); }
    }
    else if (strcmp(topic, CMD_WIND_D20) == 0) {
        uint32_t v = strtoul(val, &endPtr, 10);
        if (endPtr != val) { g_windInput.PathLenD20 = v; nvsSave(); publishWindInputs(); sendWindInputToSTM(); }
    }
    else if (strcmp(topic, CMD_GPS_HWRESET) == 0) {
        gpsHardwareReset();
        if (myGNSS.begin(Serial2, "LG290P")) myGNSS.setMessageRate("GGA", 1);
    }
}

/**
 * @brief Ethernet event handler.
 * @note FIX-10: Reset otaStarted on disconnect.
 */
static void onEthEvent(arduino_event_id_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:        ETH.setHostname(OTA_HOSTNAME); break;
        case ARDUINO_EVENT_ETH_GOT_IP:       eth_connected = true;  break;
        case ARDUINO_EVENT_ETH_DISCONNECTED: eth_connected = false; otaStarted = false; break;
        case ARDUINO_EVENT_ETH_STOP:         eth_connected = false; otaStarted = false; break;
        default: break;
    }
}

/**
 * @brief MQTT reconnection with subscription and boot-time wind config sync.
 */
static void mqttReconnect() {
    if (!eth_connected) return;
    static uint32_t lastAttempt = 0;
    if (millis() - lastAttempt < 5000UL) return;
    lastAttempt = millis();

    bool ok = (strlen(MQTT_USER) > 0)
        ? mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, TOPIC_STATUS, 1, true, "offline")
        : mqtt.connect(MQTT_CLIENT_ID, nullptr, nullptr,     TOPIC_STATUS, 1, true, "offline");

    if (ok) {
        pub(TOPIC_STATUS, "online", true);
        mqtt.subscribe(SUB_CMD_WILDCARD, 1);
        publishWindInputs();
        sendWindInputToSTM();   // send stored config to STM on every reconnect
    }
}

// ==========================================================================
// PAYLOAD PUBLISHER
// ==========================================================================

/**
 * @brief Decode AS3935 interrupt type name from packed DistanceEstimation byte.
 */
static const char* as3935IntTypeName(uint8_t intType) {
    switch (intType) {
        case AS3935_INT_L:  return "LIGHTNING";
        case AS3935_INT_D:  return "DISTURBER";
        case AS3935_INT_NH: return "NOISE_HIGH";
        default:            return "NONE";
    }
}

/**
 * @brief Publish all sensor data from the STM32 payload to MQTT.
 * @details AS3935: DistanceEstimation bits [7:4] carry the interrupt type
 *          (packed by STM main.c using AS3935_ENCODE_DIST macro).
 *          Publishes interrupt type to TOPIC_LT and distance (bits [5:0])
 *          to TOPIC_LD.  For non-lightning events (INT_D, INT_NH) the
 *          energy is 0 and the distance code is 0x3F (out of range).
 */
static void publishPayload(const UART_Payload_t& p) {
    /* Environmental */
    pub(TOPIC_TEMP, p.Env.Temperature, 2);
    pub(TOPIC_RH,   p.Env.RH,          2);
    pub(TOPIC_PRES, p.Env.Pressure,    2);
    pub(TOPIC_DP,   p.Env.DewPoint,    2);
    pub(TOPIC_SP,   p.Env.SoundSpeed,  2);

    /* Wind */
    pub(TOPIC_WS, p.Wind.Speed,     3);
    pub(TOPIC_WD, p.Wind.Direction, 1);

    /* AS3935 lightning — decode packed DistanceEstimation byte */
    uint8_t intType  = AS3935_DECODE_INT(p.Lightning.DistanceEstimation);
    uint8_t distCode = AS3935_DECODE_DIST(p.Lightning.DistanceEstimation);
    pub(TOPIC_LT, as3935IntTypeName(intType));
    pub(TOPIC_LE, p.Lightning.LightningEnergy);
    pub(TOPIC_LD, (uint8_t)distCode);

    /* UV (AS7331) */
    pub(TOPIC_UVT, (uint16_t)p.UV.TEMP_C100);
    pub(TOPIC_UVA, p.UV.UVA);
    pub(TOPIC_UVB, p.UV.UVB);
    pub(TOPIC_UVC, p.UV.UVC);

    /* Air quality (ENS160) */
    pub(TOPIC_AQI,  p.AirQuality.AQI);
    pub(TOPIC_TVOC, p.AirQuality.TVOC);
    pub(TOPIC_ECO,  p.AirQuality.eCO2);

    /* Colour (TCS34003 — RGBC only) */
    pub(TOPIC_LIGHT_C, p.LightRGB.ClearChannel);
    pub(TOPIC_LIGHT_R, p.LightRGB.RedChannel);
    pub(TOPIC_LIGHT_G, p.LightRGB.GreenChannel);
    pub(TOPIC_LIGHT_B, p.LightRGB.BlueChannel);

    /* Ambient light (TSL25911) */
    pub(TOPIC_LIGHT_FS, p.Light.FullSpectrum);
    pub(TOPIC_LIGHT_IR, p.Light.Infrared);
    pub(TOPIC_LIGHT_VI, p.Light.Visible);
    pub(TOPIC_LIGHT_LX, p.Light.Lux, 2);   /* TSL25911 broadband lux */
}

/**
 * @brief Handle calibration readback from STM32 (after Wind_CalibrateReflector).
 * @details Updates g_windInput path lengths, saves to NVS, publishes to MQTT.
 *          Does NOT update Height — that is GPS-owned by the ESP.
 */
static void handleCalibration(const Cal_Payload_t& cal) {
    g_windInput.PathLenD01 = cal.PathLenD01;
    g_windInput.PathLenD12 = cal.PathLenD12;
    g_windInput.PathLenD20 = cal.PathLenD20;
    nvsSave();
    publishWindInputs();
    Serial.printf("[CAL] D01=%lu D12=%lu D20=%lu µm\n",
                  (unsigned long)cal.PathLenD01,
                  (unsigned long)cal.PathLenD12,
                  (unsigned long)cal.PathLenD20);
}

// ==========================================================================
// STM32 UART PARSER
// ==========================================================================

/**
 * @brief FIX-1: Byte-at-a-time state-machine parser for STM32 UART frames.
 *
 * Detects two frame types by their two-byte SOF sequences:
 *   frameType 1 — sensor data:   {0x23, 0x40}  → FRAME_SIZE   = 68 bytes
 *   frameType 2 — calibration:   {0x23, 0x24}  → CAL_FRAME_SIZE = 17 bytes
 *
 * For frameType 1: CRC is checked over Payload (64 bytes); payload published.
 * For frameType 2: Length byte (rxBuf[2]) and CRC are verified; cal dispatched.
 */
static void processSTMbyte(uint8_t b) {
    if (!syncFound) {
        static uint8_t prev = 0;
        if (prev == UART_FRAME_SOF_B0 && b == UART_FRAME_SOF_B1) {
            rxBuf[0] = prev; rxBuf[1] = b; rxLen = 2; frameType = 1; syncFound = true;
            prev = 0;  // FIX-1
            return;
        }
        if (prev == CAL_SOF_B0 && b == CAL_SOF_B1) {
            rxBuf[0] = prev; rxBuf[1] = b; rxLen = 2; frameType = 2; syncFound = true;
            prev = 0;  // FIX-1
            return;
        }
        prev = b;
        return;
    }

    size_t target = (frameType == 1) ? FRAME_SIZE : CAL_FRAME_SIZE;
    if (rxLen < target) rxBuf[rxLen++] = b;

    if (rxLen == target) {
        syncFound = false; rxLen = 0;

        if (frameType == 1) {
            /* Sensor data frame */
            UART_Frame_t frame;
            memcpy(&frame, rxBuf, FRAME_SIZE);
            if (frame.Start != UART_FRAME_SOF_B0) return;
            uint16_t calc = CRC16(reinterpret_cast<const uint8_t*>(&frame.Payload),
                                  sizeof(UART_Payload_t));
            if (calc == frame.CR) publishPayload(frame.Payload);
        } else {
            /* Calibration readback frame
             * rxBuf layout: [0x23][0x24][lenByte][D01(4)][D12(4)][D20(4)][CRC_L][CRC_H]
             * CRC covers lenByte + payload = 1 + lenByte bytes                          */
            uint8_t  lenByte = rxBuf[2];
            uint16_t calcCRC = CRC16(&rxBuf[2], 1u + lenByte);
            uint16_t rxCRC   = (uint16_t)rxBuf[3 + lenByte] |
                               ((uint16_t)rxBuf[3 + lenByte + 1] << 8);
            if (lenByte == (uint8_t)CAL_PAYLOAD_SIZE && calcCRC == rxCRC) {
                Cal_Payload_t cal;
                memcpy(&cal, &rxBuf[3], CAL_PAYLOAD_SIZE);
                handleCalibration(cal);
            }
        }
    }
}

// ==========================================================================
// NTRIP ROVER
// ==========================================================================

/**
 * @brief Connect to the NTRIP caster and forward RTCM3 corrections to GNSS.
 * @note FIX-2: ntripHdrBuf at file scope, zeroed on each new connection.
 * @note FIX-4: ntripLastGGA reset on new connection.
 */
static void ntripLoop() {
    if (strlen(NTRIP_HOST) == 0 || !eth_connected) return;

    if (!ntripClient.connected()) {
        ntripStreamActive = false;
        if (millis() - ntripLastAttempt < 10000UL) return;
        ntripLastAttempt = millis();
        if (!ntripClient.connect(NTRIP_HOST, NTRIP_PORT)) return;

        memset(ntripHdrBuf, 0, sizeof(ntripHdrBuf));  // FIX-2
        ntripLastGGA = millis() - NTRIP_GGA_INTERVAL_MS;  // FIX-4

        char authHdr[192] = {0};
        if (strlen(NTRIP_USER) > 0) {
            char cred[64];
            snprintf(cred, sizeof(cred), "%s:%s", NTRIP_USER, NTRIP_PASS);
            unsigned char enc[128]; size_t olen = 0;
            mbedtls_base64_encode(enc, sizeof(enc), &olen,
                                  (const unsigned char*)cred, strlen(cred));
            snprintf(authHdr, sizeof(authHdr), "Authorization: Basic %s\r\n", enc);
        }
        char ggaHdr[144] = {0};
        if (latestGGA[0] != '\0')
            snprintf(ggaHdr, sizeof(ggaHdr), "Ntrip-GGA: %s\r\n", latestGGA);

        char req[512];
        snprintf(req, sizeof(req),
                 "GET /%s HTTP/1.0\r\nHost: %s\r\nNtrip-Version: Ntrip/1.0\r\n"
                 "User-Agent: NTRIPClient/ESP32\r\n%s%sConnection: close\r\n\r\n",
                 NTRIP_MOUNTPOINT, NTRIP_HOST, authHdr, ggaHdr);
        ntripClient.print(req);
    }

    if (ntripClient.available()) {
        if (!ntripStreamActive) {
            while (ntripClient.available() && !ntripStreamActive) {
                uint8_t b = ntripClient.read();
                memmove(ntripHdrBuf, ntripHdrBuf + 1, 3);
                ntripHdrBuf[3] = b;
                if (ntripHdrBuf[0]=='\r' && ntripHdrBuf[1]=='\n' &&
                    ntripHdrBuf[2]=='\r' && ntripHdrBuf[3]=='\n')
                    ntripStreamActive = true;
            }
        }
        if (ntripStreamActive) {
            uint8_t buf[128];
            int avail = ntripClient.available();
            while (avail > 0) {
                int toRead = min(avail, (int)sizeof(buf));
                int got    = ntripClient.readBytes(buf, toRead);
                if (got > 0) Serial2.write(buf, got);
                avail = ntripClient.available();
            }
        }
    }

    if (ntripStreamActive && latestGGA[0] != '\0' &&
        (millis() - ntripLastGGA >= NTRIP_GGA_INTERVAL_MS)) {
        ntripLastGGA = millis();
        ntripClient.print(latestGGA);
        ntripClient.print("\r\n");
    }
}

// ==========================================================================
// OTA & SETUP
// ==========================================================================

static void otaSetup() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    nvsLoad();  // also calls packWindHeight() internally

    pinMode(PIN_STM_IS_TX, INPUT);
    pinMode(PIN_GPS_RST, OUTPUT);
    gpsHardwareReset();

    Serial1.begin(115200, SERIAL_8N1, PIN_STM_RX, PIN_STM_TX);
    Serial2.begin(460800, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    if (myGNSS.begin(Serial2, "LG290P")) myGNSS.setMessageRate("GGA", 1);

    Network.onEvent(onEthEvent);
    ETH.begin();

    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setBufferSize(512);  // FIX-9
    mqtt.setKeepAlive(30);
    mqtt.setCallback(mqttCallback);

    otaSetup();
}

void loop() {
    if (eth_connected && !otaStarted) {
        ArduinoOTA.begin();
        otaStarted = true;
    }
    if (otaStarted) ArduinoOTA.handle();

    if (eth_connected) {
        if (!mqtt.connected()) mqttReconnect();
        else mqtt.loop();
    }

    while (Serial1.available())
        processSTMbyte(static_cast<uint8_t>(Serial1.read()));

    myGNSS.update();
    ntripLoop();
}
