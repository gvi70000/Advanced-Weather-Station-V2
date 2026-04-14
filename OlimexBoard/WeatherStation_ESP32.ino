/**
 * @file WeatherStation_ESP32.ino
 * @brief Olimex ESP32-POE2 (WROVER-E-N4R8) – Weather Station Gateway
 *
 * Functions
 * ---------
 *  1. Receives binary UART_Frame_t packets from STM32G431 (Serial1) and
 *     publishes every field as a plain scalar string on its own MQTT topic
 *     via PoE Ethernet.
 *  2. Receives NMEA / PQTM messages from a Quectel LG290P RTK GNSS module
 *     (Serial2) via the SparkFun LG290P library and publishes position,
 *     fix quality, and RTK status as individual MQTT topics.
 *  3. Handshake lines between ESP and STM32:
 *       GPIO35 (input)  <- STM PA11  HIGH while STM is TRANSMITTING to ESP
 *       GPIO13 (output) -> STM PA12  driven LOW while ESP is TRANSMITTING to STM
 *                         (2.2 kOhm pull-up on STM side; idle = HIGH)
 *  4. ArduinoOTA: firmware can be pushed wirelessly from Arduino IDE / CLI
 *     once the board has an IP address.  OTA password is set below.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * HARDWARE PIN MAP  (ESP32 GPIO numbers)
 * ──────────────────────────────────────────────────────────────────────────
 *  GPIO36 (input-only)  <- STM PA9  Tx  (STM->ESP UART data)   Serial1 RX
 *                                                               10 kOhm pull-up
 *  GPIO4                -> STM PA10 Rx  (ESP->STM UART data)   Serial1 TX
 *  GPIO35 (input-only)  <- STM PA11    HIGH = STM transmitting  sense pin
 *  GPIO13               -> STM PA12 EXTI  LOW = ESP transmitting
 *                                               2.2 kOhm pull-up, idle HIGH
 *
 *  GPIO34 (input-only)  <- LG290P Tx  (GPS->ESP)               Serial2 RX
 *  GPIO32               -> LG290P Rx  (ESP->GPS, RTCM/cmds)    Serial2 TX
 *  GPIO5                -> LG290P RESET_N   10 kOhm pull-up, active-LOW
 *                          P2 connector: VCC, GND, Tx, Rx, RST, 1PPS
 *
 * ──────────────────────────────────────────────────────────────────────────
 * IMPORTANT NOTES
 * ──────────────────────────────────────────────────────────────────────────
 *  - GPIO16 / GPIO17 are used by the WROVER PSRAM – do NOT assign them.
 *  - GPIO0  is used by the LAN8720 Ethernet clock  – do NOT assign it.
 *  - GPIO35 maps to the Olimex battery-voltage sense pin (SENS_BAT_E1
 *    solder jumper).  If the jumper is closed, UART-level use is still safe
 *    (input-only) but the ADC reading will reflect UART levels, not battery.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * DEPENDENCIES  (install via Arduino Library Manager)
 * ──────────────────────────────────────────────────────────────────────────
 *  - SparkFun LG290P Quadband RTK GNSS Arduino Library  (>= v3.0.0)
 *  - PubSubClient  by Nick O'Leary                       (>= 2.8)
 *
 * ARDUINO IDE SETTINGS
 *  Board      : ESP32 Dev Module  (or "Olimex ESP32-POE2" if listed)
 *  PSRAM      : Enabled           <- mandatory for WROVER
 *  Flash size : 4 MB
 *  Partition  : Default 4MB with spiffs  (OTA needs two app partitions)
 *
 * ──────────────────────────────────────────────────────────────────────────
 * MIT Licence – adapt freely.
 * ──────────────────────────────────────────────────────────────────────────
 */

// ==========================================================================
// ETH – PHY settings MUST be defined before #include <ETH.h>
// ==========================================================================
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
#include <PubSubClient.h>
#include <SparkFun_LG290P_GNSS.h>

// ==========================================================================
// USER CONFIGURATION  – edit this section only
// ==========================================================================

// --- MQTT broker ---
static const char*    MQTT_SERVER    = "10.0.1.6";
static const uint16_t MQTT_PORT      = 1883;
static const char*    MQTT_CLIENT_ID = "WeatherStation";
static const char*    MQTT_USER      = "";   // leave empty if not required
static const char*    MQTT_PASS      = "";

// --- OTA ---
static const char*    OTA_HOSTNAME   = "WeatherStation";
static const char*    OTA_PASSWORD   = "ota1234";   // change before deployment!

// --- NTRIP (fill in to stream RTCM3 RTK corrections to the LG290P) ---
static const char*    NTRIP_HOST       = "";   // e.g. "ntrip.example.com"
static const uint16_t NTRIP_PORT       = 2101;
static const char*    NTRIP_MOUNTPOINT = "";   // e.g. "NEAREST0"
static const char*    NTRIP_USER       = "";
static const char*    NTRIP_PASS       = "";

// ==========================================================================
// MQTT TOPICS  – one plain scalar value per topic
// ==========================================================================

// Environmental
static const char* TOPIC_TEMP      = "weather/Temperature";       // float  degrees C
static const char* TOPIC_RH        = "weather/RH";                // float  %
static const char* TOPIC_PRES      = "weather/Pressure";          // float  hPa
static const char* TOPIC_SP        = "weather/SoundSpeed";        // float  m/s
static const char* TOPIC_DP        = "weather/DewPoint";          // float  degrees C

// UV (AS7331)
static const char* TOPIC_UVT       = "weather/UVTemperature";     // uint16 (degrees C x 100)
static const char* TOPIC_UVA       = "weather/UVA";               // uint16 counts
static const char* TOPIC_UVB       = "weather/UVB";               // uint16 counts
static const char* TOPIC_UVC       = "weather/UVC";               // uint16 counts

// Colour light (TCS34003)
static const char* TOPIC_LIGHT_C   = "weather/ClearChannel";      // uint16
static const char* TOPIC_LIGHT_R   = "weather/RedChannel";        // uint16
static const char* TOPIC_LIGHT_G   = "weather/GreenChannel";      // uint16
static const char* TOPIC_LIGHT_B   = "weather/BlueChannel";       // uint16

// Broad-spectrum light (TSL25911)
static const char* TOPIC_LIGHT_FS  = "weather/FullSpectrum";      // uint16
static const char* TOPIC_LIGHT_IR  = "weather/Infrared";          // uint16
static const char* TOPIC_LIGHT_VI  = "weather/Visible";           // uint16
static const char* TOPIC_LIGHT_LX  = "weather/Lux";               // float

// Air quality (ENS160)
static const char* TOPIC_AQI       = "weather/AQI";               // uint8  1-5
static const char* TOPIC_TVOC      = "weather/TVOC";              // uint16 ppb
static const char* TOPIC_ECO       = "weather/eCO2";              // uint16 ppm

// Wind (PGA460)
static const char* TOPIC_WS        = "weather/WindSpeed";         // float  m/s
static const char* TOPIC_WD        = "weather/WindDirection";     // float  degrees

// Lightning (AS3935)
static const char* TOPIC_LE        = "weather/LightningEnergy";   // uint32
static const char* TOPIC_LD        = "weather/DistanceEstimation";// uint8

// Housekeeping
static const char* TOPIC_STATUS    = "weather/status";            // "online" / "offline"

// ==========================================================================
// PIN DEFINITIONS
// ==========================================================================

// STM32 UART  (Serial1)
#define PIN_STM_RX      36   // GPIO36  input-only  <- STM PA9  Tx  (10 kOhm pull-up)
#define PIN_STM_TX       4   // GPIO4               -> STM PA10 Rx

// STM32 handshake
#define PIN_STM_IS_TX   35   // GPIO35  input-only  <- STM PA11
                             //   HIGH while STM is transmitting to ESP
#define PIN_ESP_IS_TX   13   // GPIO13  output      -> STM PA12 EXTI
                             //   driven LOW while ESP is transmitting to STM
                             //   2.2 kOhm pull-up on STM side; idle = HIGH

// LG290P GNSS  (Serial2)
#define PIN_GPS_RX      34   // GPIO34  input-only  <- LG290P Tx
#define PIN_GPS_TX      32   // GPIO32              -> LG290P Rx
#define PIN_GPS_RST      5   // GPIO5               -> LG290P RESET_N (10 kOhm pull-up, active-LOW)

// ==========================================================================
// FRAME STRUCTURES  – must match the STM32 packed structs byte-for-byte
//
// NOTE: ENS160_Data_t  has no Raw[5] field here  (STM does not transmit it).
//       TCS34003_LightData_t has no FullArray[8]  (same reason).
//       Both structs are exact mirrors of what the STM sends in UART_Payload_t.
// ==========================================================================

#define UART_FRAME_START  0xAA55u
#define UART_FRAME_SOF_B0 0x55u   // little-endian byte 0 of 0xAA55
#define UART_FRAME_SOF_B1 0xAAu   // little-endian byte 1

struct Wind_EnvData_t {
    float Temperature;   // degrees C
    float RH;            // %
    float Pressure;      // hPa
    float SoundSpeed;    // m/s
} __attribute__((packed));

struct Wind_t {
    float Speed;         // m/s
    float Direction;     // degrees FROM  (0/360=N, 90=E, 180=S, 270=W)
} __attribute__((packed));

struct AS3935_LightningData_t {
    uint32_t LightningEnergy;    // 20-bit raw energy
    uint8_t  DistanceEstimation; // distance code
} __attribute__((packed));

struct AS7331_DataOut_t {
    uint16_t TEMP_C100;  // die temperature x 100
    uint16_t UVA;
    uint16_t UVB;
    uint16_t UVC;
} __attribute__((packed));

struct ENS160_Data_t {
    uint8_t  AQI;        // Air Quality Index 1-5
    uint16_t TVOC;       // ppb
    uint16_t eCO2;       // ppm
} __attribute__((packed));

struct TCS34003_LightData_t {
    uint16_t ClearChannel;
    uint16_t RedChannel;
    uint16_t GreenChannel;
    uint16_t BlueChannel;
} __attribute__((packed));

struct TSL25911_LightData_t {
    uint16_t FullSpectrum;
    uint16_t Infrared;
    uint16_t Visible;
    float    Lux;
} __attribute__((packed));

struct UART_Payload_t {
    Wind_EnvData_t         Env;
    float                  DewPoint;
    AS3935_LightningData_t Lightning;
    AS7331_DataOut_t       UV;
    ENS160_Data_t          AirQuality;
    TCS34003_LightData_t   LightRGB;
    TSL25911_LightData_t   Light;
    Wind_t                 Wind;
} __attribute__((packed));

struct UART_Frame_t {
    uint16_t       Start;    // 0xAA55
    uint16_t       Length;   // sizeof(UART_Payload_t)
    UART_Payload_t Payload;
    uint16_t       CR;       // CRC-16
} __attribute__((packed));

static constexpr size_t FRAME_SIZE = sizeof(UART_Frame_t);

// ==========================================================================
// CRC-16  (Modbus / CRC16-IBM  polynomial 0xA001)
// Must match the STM32 CRC16() implementation exactly.
// ==========================================================================
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

// ==========================================================================
// GLOBALS
// ==========================================================================

static bool eth_connected = false;
static bool otaStarted    = false;   // ArduinoOTA.begin() deferred until IP ready

WiFiClient   ethClient;
PubSubClient mqtt(ethClient);

WiFiClient      ntripClient;
static bool     ntripStreamActive = false;
static uint32_t ntripLastAttempt  = 0;

LG290P myGNSS;

// STM32 frame receiver state machine
static uint8_t  rxBuf[FRAME_SIZE];
static uint16_t rxLen     = 0;
static bool     syncFound = false;

// ==========================================================================
// MQTT PUBLISH HELPERS
// Each overload formats its value type and calls mqtt.publish().
// All are no-ops when the broker is not connected.
// ==========================================================================
static void pub(const char* topic, const char* value, bool retain = false) {
    if (mqtt.connected()) {
        mqtt.publish(topic, value, retain);
    }
}

static void pub(const char* topic, float v, uint8_t dec = 2) {
    char buf[24];
    dtostrf(v, 1, dec, buf);
    pub(topic, buf);
}

static void pub(const char* topic, double v, uint8_t dec = 8) {
    char buf[32];
    dtostrf(v, 1, dec, buf);
    pub(topic, buf);
}

static void pub(const char* topic, uint32_t v) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)v);
    pub(topic, buf);
}

static void pub(const char* topic, uint16_t v) { pub(topic, (uint32_t)v); }
static void pub(const char* topic, uint8_t  v) { pub(topic, (uint32_t)v); }

// ==========================================================================
// ETHERNET EVENT HANDLER
// ==========================================================================
static void onEthEvent(arduino_event_id_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            Serial.println("[ETH] Started");
            ETH.setHostname(OTA_HOSTNAME);
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("[ETH] Cable connected");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.print("[ETH] IP address: ");
            Serial.println(ETH.localIP());
            eth_connected = true;
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("[ETH] Disconnected");
            eth_connected = false;
            break;
        case ARDUINO_EVENT_ETH_STOP:
            Serial.println("[ETH] Stopped");
            eth_connected = false;
            break;
        default:
            break;
    }
}

// ==========================================================================
// MQTT – reconnect with 5 s throttle
// ==========================================================================
static void mqttReconnect() {
    if (!eth_connected) return;

    static uint32_t lastAttempt = 0;
    if (millis() - lastAttempt < 5000UL) return;
    lastAttempt = millis();

    Serial.printf("[MQTT] Connecting to %s:%u ...", MQTT_SERVER, MQTT_PORT);

    bool ok = (strlen(MQTT_USER) > 0)
        ? mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS,
                       TOPIC_STATUS, 1, true, "offline")
        : mqtt.connect(MQTT_CLIENT_ID, nullptr, nullptr,
                       TOPIC_STATUS, 1, true, "offline");

    if (ok) {
        Serial.println(" connected");
        pub(TOPIC_STATUS, "online", true);
    } else {
        Serial.printf(" failed (rc=%d)\n", mqtt.state());
    }
}

// ==========================================================================
// PUBLISH STM32 PAYLOAD – one MQTT message per sensor field
// ==========================================================================
static void publishPayload(const UART_Payload_t& p) {
    // Environmental
    pub(TOPIC_TEMP, p.Env.Temperature,  2);
    pub(TOPIC_RH,   p.Env.RH,           2);
    pub(TOPIC_PRES, p.Env.Pressure,     2);
    pub(TOPIC_SP,   p.Env.SoundSpeed,   2);
    pub(TOPIC_DP,   p.DewPoint,         2);

    // UV
    pub(TOPIC_UVT, p.UV.TEMP_C100);
    pub(TOPIC_UVA, p.UV.UVA);
    pub(TOPIC_UVB, p.UV.UVB);
    pub(TOPIC_UVC, p.UV.UVC);

    // Colour light
    pub(TOPIC_LIGHT_C, p.LightRGB.ClearChannel);
    pub(TOPIC_LIGHT_R, p.LightRGB.RedChannel);
    pub(TOPIC_LIGHT_G, p.LightRGB.GreenChannel);
    pub(TOPIC_LIGHT_B, p.LightRGB.BlueChannel);

    // Broad-spectrum / lux
    pub(TOPIC_LIGHT_FS, p.Light.FullSpectrum);
    pub(TOPIC_LIGHT_IR, p.Light.Infrared);
    pub(TOPIC_LIGHT_VI, p.Light.Visible);
    pub(TOPIC_LIGHT_LX, p.Light.Lux, 2);

    // Air quality
    pub(TOPIC_AQI,  p.AirQuality.AQI);
    pub(TOPIC_TVOC, p.AirQuality.TVOC);
    pub(TOPIC_ECO,  p.AirQuality.eCO2);

    // Wind
    pub(TOPIC_WS, p.Wind.Speed,     3);
    pub(TOPIC_WD, p.Wind.Direction, 1);

    // Lightning
    pub(TOPIC_LE, p.Lightning.LightningEnergy);
    pub(TOPIC_LD, p.Lightning.DistanceEstimation);
}

// ==========================================================================
// STM32 FRAME RECEIVER
// Scans incoming bytes for the 0x55 0xAA SOF, accumulates exactly FRAME_SIZE
// bytes, validates CRC, and dispatches to publishPayload().
// ==========================================================================
static void processSTMbyte(uint8_t b) {
    if (!syncFound) {
        static uint8_t prev = 0;
        if (prev == UART_FRAME_SOF_B0 && b == UART_FRAME_SOF_B1) {
            rxBuf[0] = prev;
            rxBuf[1] = b;
            rxLen     = 2;
            syncFound = true;
        }
        prev = b;
        return;
    }

    if (rxLen < FRAME_SIZE) {
        rxBuf[rxLen++] = b;
    }

    if (rxLen == FRAME_SIZE) {
        syncFound = false;
        rxLen     = 0;

        UART_Frame_t frame;
        memcpy(&frame, rxBuf, FRAME_SIZE);

        if (frame.Start != UART_FRAME_START) {
            Serial.println("[STM] Bad SOF – frame discarded");
            return;
        }

        uint16_t calc = CRC16(
            reinterpret_cast<const uint8_t*>(&frame.Payload),
            sizeof(UART_Payload_t));

        if (calc != frame.CR) {
            Serial.printf("[STM] CRC fail: rx=0x%04X calc=0x%04X\n",
                          frame.CR, calc);
            return;
        }

        publishPayload(frame.Payload);
    }
}

// ==========================================================================
// SEND TO STM  (stub – for future downstream commands, e.g. remote reset)
//
// Protocol:
//   1. Check GPIO35: if HIGH the STM is currently transmitting – skip.
//   2. Drive GPIO13 LOW  -> STM EXTI on PA12 triggers (ESP is transmitting).
//   3. Write data, flush.
//   4. Release GPIO13 HIGH (restored by the 2.2 kOhm pull-up on STM side).
// ==========================================================================
static void sendToSTM(const uint8_t* data, size_t len) {
    if (digitalRead(PIN_STM_IS_TX) == HIGH) {
        // STM is currently transmitting – avoid bus collision
        Serial.println("[STM-TX] STM busy, transmission skipped");
        return;
    }
    digitalWrite(PIN_ESP_IS_TX, LOW);   // assert: ESP is transmitting
    Serial1.write(data, len);
    Serial1.flush();
    digitalWrite(PIN_ESP_IS_TX, HIGH);  // deassert: back to idle
}

// ==========================================================================
// NTRIP CLIENT
// Connects to a caster, strips the HTTP header, then forwards every RTCM3
// byte to the LG290P via Serial2 to enable RTK Fixed mode.
// Auto-reconnects after 10 s if the TCP link drops.
// ==========================================================================
static void ntripLoop() {
    if (strlen(NTRIP_HOST) == 0) return;  // not configured – skip
    if (!eth_connected)           return;

    if (!ntripClient.connected()) {
        ntripStreamActive = false;
        if (millis() - ntripLastAttempt < 10000UL) return;
        ntripLastAttempt = millis();

        Serial.printf("[NTRIP] Connecting to %s:%u\n", NTRIP_HOST, NTRIP_PORT);

        if (!ntripClient.connect(NTRIP_HOST, NTRIP_PORT)) {
            Serial.println("[NTRIP] TCP connection failed");
            return;
        }

        // Build NTRIP/1.0 GET request with optional Basic auth
        String auth = "";
        if (strlen(NTRIP_USER) > 0) {
            static const char b64[] =
                "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            String cred = String(NTRIP_USER) + ":" + String(NTRIP_PASS);
            const uint8_t* in = reinterpret_cast<const uint8_t*>(cred.c_str());
            size_t n = cred.length();
            String enc = "";
            for (size_t i = 0; i < n; i += 3) {
                uint32_t v = (uint32_t)in[i] << 16;
                if (i + 1 < n) v |= (uint32_t)in[i + 1] << 8;
                if (i + 2 < n) v |= (uint32_t)in[i + 2];
                enc += b64[(v >> 18) & 0x3F];
                enc += b64[(v >> 12) & 0x3F];
                enc += (i + 1 < n) ? b64[(v >>  6) & 0x3F] : '=';
                enc += (i + 2 < n) ? b64[(v      ) & 0x3F] : '=';
            }
            auth = "Authorization: Basic " + enc + "\r\n";
        }

        String req = String("GET /") + NTRIP_MOUNTPOINT + " HTTP/1.0\r\n"
                   + "Host: " + NTRIP_HOST + "\r\n"
                   + "Ntrip-Version: Ntrip/1.0\r\n"
                   + "User-Agent: NTRIPClient/ESP32\r\n"
                   + auth
                   + "Connection: close\r\n\r\n";
        ntripClient.print(req);
        Serial.println("[NTRIP] Request sent – waiting for ICY 200 OK");
    }

    // Forward RTCM bytes; discard the HTTP response header first
    while (ntripClient.available()) {
        uint8_t b = ntripClient.read();
        if (!ntripStreamActive) {
            // Look for the CR LF CR LF end-of-header sequence
            static uint8_t hdr[4] = {0, 0, 0, 0};
            memmove(hdr, hdr + 1, 3);
            hdr[3] = b;
            if (hdr[0] == '\r' && hdr[1] == '\n' &&
                hdr[2] == '\r' && hdr[3] == '\n') {
                ntripStreamActive = true;
                Serial.println("[NTRIP] RTCM3 stream active – forwarding to LG290P");
            }
        } else {
            Serial2.write(b);   // correction bytes -> LG290P UART
        }
    }
}

// ==========================================================================
// OTA SETUP  (called once from setup; ArduinoOTA.begin() deferred to loop)
// ==========================================================================
static void otaSetup() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("[OTA] Start – updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Update complete");
    });
    ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
        Serial.printf("[OTA] %u%%\r", done * 100 / total);
    });
    ArduinoOTA.onError([](ota_error_t err) {
        Serial.printf("[OTA] Error[%u]: ", err);
        switch (err) {
            case OTA_AUTH_ERROR:    Serial.println("Auth Failed");    break;
            case OTA_BEGIN_ERROR:   Serial.println("Begin Failed");   break;
            case OTA_CONNECT_ERROR: Serial.println("Connect Failed"); break;
            case OTA_RECEIVE_ERROR: Serial.println("Receive Failed"); break;
            case OTA_END_ERROR:     Serial.println("End Failed");     break;
        }
    });
    // ArduinoOTA.begin() is called in loop() once eth_connected becomes true
}

// ==========================================================================
// SETUP
// ==========================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);   // allow USB-serial to enumerate before printing
    Serial.println("\n[WS] Weather Station ESP32 – booting");

    // --- STM32 handshake pins ---
    pinMode(PIN_STM_IS_TX, INPUT);
    pinMode(PIN_ESP_IS_TX, OUTPUT);
    digitalWrite(PIN_ESP_IS_TX, HIGH);   // idle HIGH (deasserted)

    // --- LG290P hardware reset ---
    pinMode(PIN_GPS_RST, OUTPUT);
    digitalWrite(PIN_GPS_RST, HIGH);     // deasserted (normal operation)
    delay(10);
    digitalWrite(PIN_GPS_RST, LOW);      // assert reset  (active-LOW)
    delay(200);
    digitalWrite(PIN_GPS_RST, HIGH);     // release reset
    delay(1000);                         // wait for LG290P boot cycle
    Serial.println("[WS] LG290P reset done");

    // --- STM32 UART ---
    Serial1.begin(115200, SERIAL_8N1, PIN_STM_RX, PIN_STM_TX);
    Serial.println("[WS] Serial1 (STM32) ready at 115200 baud");

    // --- LG290P GNSS UART ---
    Serial2.begin(460800, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    if (myGNSS.begin(Serial2)) {
        Serial.println("[GPS] LG290P detected");
        myGNSS.setNMEAOutputRate(1);   // 1 Hz NMEA sentences
        // Library auto-parses PQTM RTK-status messages
    } else {
        Serial.println("[GPS] LG290P NOT detected – check wiring and baud rate");
    }

    // --- Ethernet ---
    Network.onEvent(onEthEvent);
    ETH.begin();
    Serial.println("[WS] Ethernet initialising");

    // --- MQTT ---
    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setBufferSize(256);
    mqtt.setKeepAlive(30);

    // --- OTA callbacks ---
    otaSetup();

    Serial.printf("[WS] Frame size: %u bytes\n", (unsigned)FRAME_SIZE);
    Serial.println("[WS] Setup complete");
}

// ==========================================================================
// LOOP
// ==========================================================================
void loop() {

    // --- OTA: start service once we have an IP address ---
    if (eth_connected && !otaStarted) {
        ArduinoOTA.begin();
        otaStarted = true;
        Serial.println("[OTA] Service started – ready for wireless upload");
    }
    if (otaStarted) {
        ArduinoOTA.handle();
    }

    // --- MQTT keep-alive ---
    if (eth_connected) {
        if (!mqtt.connected()) {
            mqttReconnect();
        } else {
            mqtt.loop();
        }
    }

    // --- STM32 frame receive ---
    while (Serial1.available()) {
        processSTMbyte(static_cast<uint8_t>(Serial1.read()));
    }

    // --- GPS: feed parser; publish whenever a new fix is ready ---
    myGNSS.update();

    // --- NTRIP: forward RTCM3 corrections to LG290P ---
    ntripLoop();
}
