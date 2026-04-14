/**
 * @file WeatherStation_ESP32.ino
 * @brief Olimex ESP32-POE2 (WROVER-E-N4R8) – Weather Station Gateway
 *
 * Functions
 * ---------
 *  1. Receives binary UART_Frame_t packets from STM32G431 (UART1) and
 *     publishes all fields as JSON to an MQTT broker via PoE Ethernet.
 *  2. Receives NMEA / PQTM messages from a Quectel LG290P RTK GNSS module
 *     (UART2) via the SparkFun LG290P library and publishes position,
 *     fix quality, and RTK status to a separate MQTT topic.
 *  3. Monitors PA11 (STM RX-busy) and PA12 (STM TX-busy) to allow future
 *     upstream commands to the STM (currently a stub – see Q3 answer below).
 *
 * ──────────────────────────────────────────────────────────────────────────
 * HARDWARE PIN MAP  (ESP32 GPIO numbers)
 * ──────────────────────────────────────────────────────────────────────────
 *  GPIO36 (input-only)  ← STM PA9  Tx  (STM→ESP UART RX)       Serial1 RX
 *  GPIO4                → STM PA10 Rx  (ESP→STM UART TX)       Serial1 TX
 *  GPIO35 (input-only)  ← STM PA11     "STM is transmitting"   sense pin
 *  GPIO13 HIGH Default  → STM PA12 INT "ESP is transmitting"   signal pin
 *
 *  GPI34  (input-only)  ← LG290P Tx  (GPS→ESP)                 Serial2 RX
 *  GPIO32				 → LG290P Rx  (GPS→ESP)					Serial2 TX
 * ──────────────────────────────────────────────────────────────────────────
 *  STM32: The STM only listens for upstream commands via PA12 EXTI.  In this
 *  firmware there are no control messages sent to the STM; PA12 / GPIO33 is
 *  driven HIGH while Serial1.write() is active (stub provided: sendToSTM()).
 *
 *  LG290P: RTCM3 correction bytes from an NTRIP caster should be forwarded
 *  to the GPS via Serial2 to achieve RTK Fix.  A lightweight NTRIP client
 *  stub (NtripClient) is included; fill in NTRIP_HOST / PORT / MOUNTPOINT /
 *  NTRIP_USER / NTRIP_PASS and call ntripLoop() from loop().
 *
 * ──────────────────────────────────────────────────────────────────────────
 * DEPENDENCIES (install via Arduino Library Manager)
 * ──────────────────────────────────────────────────────────────────────────
 *  • SparkFun LG290P Quadband RTK GNSS Arduino Library  (>= v3.0.0)
 *  • PubSubClient  by Nick O'Leary                       (>= 2.8)
 *  • ArduinoJson   by Benoit Blanchon                    (>= 7.x)
 *
 * ARDUINO IDE SETTINGS
 *  Board  : ESP32 Dev Module   (or "Olimex ESP32-POE2" if available)
 *  PSRAM  : Enabled            ← mandatory for WROVER
 *  Flash  : 4MB
 *  Part.  : Default (or your custom scheme)
 *
 * ──────────────────────────────────────────────────────────────────────────
 * MIT Licence – adapt freely.
 * ──────────────────────────────────────────────────────────────────────────
 * https://www.centipede-rtk.org/projects
 * https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station/all
 * https://gpsd.gitlab.io/gpsd/ppp-howto.html
 *
 *  https://www.ngs.noaa.gov/OPUS/
 *  https://webapp.csrs-scrs.nrcan-rncan.gc.ca/geod/tools-outils/ppp.php?locale=en
 *  https://www.unoosa.org/documents/pdf/icg/2018/ait-gnss/16_PPP.pdf
 *
 */

// ==========================================================================
// ETH – must be defined BEFORE #include <ETH.h>
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
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SparkFun_LG290P_GNSS.h>   // SparkFun LG290P library

// ==========================================================================
// USER CONFIGURATION
// ==========================================================================

// --- MQTT broker ---
static const char* MQTT_SERVER   = "10.0.1.6";
static const uint16_t MQTT_PORT  = 1883;
static const char* MQTT_CLIENT_ID = "WeatherStation";
// Optional broker credentials (leave empty strings if not needed)
static const char* MQTT_USER     = "";
static const char* MQTT_PASS     = "";

// --- MQTT topics ---
static const char* TOPIC_ENV        = "weather/env";
static const char* TOPIC_WIND       = "weather/wind";
static const char* TOPIC_LIGHTNING  = "weather/lightning";
static const char* TOPIC_UV         = "weather/uv";
static const char* TOPIC_AIR        = "weather/air";
static const char* TOPIC_LIGHT_RGB  = "weather/light/rgb";
static const char* TOPIC_LIGHT_LUX  = "weather/light/lux";
static const char* TOPIC_GPS        = "weather/gps";
static const char* TOPIC_STATUS     = "weather/status";

// --- NTRIP client (fill in to get RTK corrections to GPS) ---
static const char* NTRIP_HOST       = "";       // e.g. "ntrip.kartverket.no"
static const uint16_t NTRIP_PORT    = 2101;
static const char* NTRIP_MOUNTPOINT = "";       // e.g. "HFKR0"
static const char* NTRIP_USER       = "";
static const char* NTRIP_PASS       = "";

// ==========================================================================
// PIN DEFINITIONS
// ==========================================================================

// --- STM32 UART (Serial1) ---
#define PIN_STM_RX   36   // GPIO36 input-only  ← STM PA09 Tx 10k PullUp
#define PIN_STM_TX    4   // GPIO4              → STM PA10 Rx

// --- STM32 handshake signals ---
#define PIN_STM_IS_TX   35  // GPI35 input-only – HIGH when STM is receiving. Map to Battery. Jumper Open.
#define PIN_ESP_IS_TX   13  // GPIO13 output     – drive HIGH while ESP Tx-ing Has 2.2k PullUp

// --- LG290P GNSS UART (Serial2) ---
#define PIN_GPS_RX   34   // GPI39   ← LG290P Tx
#define PIN_GPS_TX   32   // GPIO32  → LG290P Rx
#define PIN_GPS_RST  5    // GPIO5   → LG290P RESET_N 10k PullUp
// Use P2 connector: VCC, GND, Tx, Rx, RST, 1PPS

// ==========================================================================
// FRAME DEFINITIONS  (must match STM32 structs exactly)
// ==========================================================================

#define UART_FRAME_START  0xAA55
#define UART_FRAME_SOF_B0 0x55   // little-endian low byte of 0xAA55
#define UART_FRAME_SOF_B1 0xAA   // high byte

// Environmental / wind data mirrored from stm32 Wind_EnvData_t
struct Wind_EnvData_t {
    float Temperature;   // °C
    float RH;            // %
    float Pressure;      // hPa
    float SoundSpeed;    // m/s
} __attribute__((packed));

struct Wind_t {
    float Speed;         // m/s
    float Direction;     // degrees
} __attribute__((packed));

struct AS3935_LightningData_t {
    uint32_t LightningEnergy;      // 20-bit raw
    uint8_t  DistanceEstimation;   // code
} __attribute__((packed));

struct AS7331_DataOut_t {
    uint16_t TEMP_C100;  // °C * 100
    uint16_t UVA;
    uint16_t UVB;
    uint16_t UVC;
} __attribute__((packed));

struct ENS160_Data_t {
    uint8_t  AQI;    // 1..5
    uint16_t TVOC;   // ppb
    uint16_t eCO2;   // ppm
    uint8_t  Raw[5];
} __attribute__((packed));

struct TCS34003_LightData_t {
    uint16_t ClearChannel;
    uint16_t RedChannel;
    uint16_t GreenChannel;
    uint16_t BlueChannel;
    uint8_t  FullArray[8];
} __attribute__((packed));

struct TSL25911_LightData_t {
    uint16_t FullSpectrum;
    uint16_t Infrared;
    uint16_t Visible;
    float    Lux;
} __attribute__((packed));

struct UART_Payload_t {
    Wind_EnvData_t       Env;
    float                DewPoint;
    AS3935_LightningData_t Lightning;
    AS7331_DataOut_t     UV;
    ENS160_Data_t        AirQuality;
    TCS34003_LightData_t LightRGB;
    TSL25911_LightData_t Light;
    Wind_t               Wind;
} __attribute__((packed));

struct UART_Frame_t {
    uint16_t Start;    // 0xAA55
    uint16_t Length;   // sizeof(UART_Payload_t)
    UART_Payload_t Payload;
    uint16_t CR;       // CRC16
} __attribute__((packed));

static constexpr size_t FRAME_SIZE = sizeof(UART_Frame_t);

// ==========================================================================
// CRC-16 (Modbus/CRC16-IBM – same algorithm as STM side)
// ==========================================================================
static uint16_t CRC16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

// ==========================================================================
// GLOBALS
// ==========================================================================

static bool eth_connected = false;

// Networking
WiFiClient   ethClient;
WiFiClient   ntripEthClient;
PubSubClient mqtt(ethClient);

// GPS
LG290P myGNSS;

// STM receive buffer
static uint8_t  rxBuf[FRAME_SIZE];
static uint16_t rxLen = 0;
static bool     syncFound = false;

// ==========================================================================
// ETHERNET EVENT HANDLER
// ==========================================================================
void onEvent(arduino_event_id_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            Serial.println("[ETH] Started");
            ETH.setHostname("WeatherStation");
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("[ETH] Connected");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.print("[ETH] IP: ");
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
// MQTT
// ==========================================================================
static void mqttReconnect() {
    if (!eth_connected) return;
    static uint32_t lastAttempt = 0;
    if (millis() - lastAttempt < 5000) return;   // throttle retries
    lastAttempt = millis();

    Serial.print("[MQTT] Connecting to ");
    Serial.print(MQTT_SERVER);
    Serial.print("...");

    bool ok = (strlen(MQTT_USER) > 0)
        ? mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS,
                       TOPIC_STATUS, 1, true, "offline")
        : mqtt.connect(MQTT_CLIENT_ID, nullptr, nullptr,
                       TOPIC_STATUS, 1, true, "offline");

    if (ok) {
        Serial.println(" connected");
        mqtt.publish(TOPIC_STATUS, "online", true);
    } else {
        Serial.print(" failed, rc=");
        Serial.println(mqtt.state());
    }
}

static void mqttPublish(const char* topic, const String& payload) {
    if (!mqtt.connected()) return;
    mqtt.publish(topic, payload.c_str());
}

// ==========================================================================
// PUBLISH STM PAYLOAD
// ==========================================================================
static void publishPayload(const UART_Payload_t& p) {
    // --- Environmental ---
    {
        JsonDocument doc;
        doc["temp_c"]       = serialized(String(p.Env.Temperature, 2));
        doc["rh_pct"]       = serialized(String(p.Env.RH, 2));
        doc["pressure_hpa"] = serialized(String(p.Env.Pressure, 2));
        doc["sound_ms"]     = serialized(String(p.Env.SoundSpeed, 2));
        doc["dew_c"]        = serialized(String(p.DewPoint, 2));
        String out;
        serializeJson(doc, out);
        mqttPublish(TOPIC_ENV, out);
    }
    // --- Wind ---
    {
        JsonDocument doc;
        doc["speed_ms"]   = serialized(String(p.Wind.Speed, 3));
        doc["dir_deg"]    = serialized(String(p.Wind.Direction, 1));
        String out;
        serializeJson(doc, out);
        mqttPublish(TOPIC_WIND, out);
    }
    // --- Lightning ---
    {
        JsonDocument doc;
        doc["energy"]   = p.Lightning.LightningEnergy;
        doc["dist_code"]= p.Lightning.DistanceEstimation;
        String out;
        serializeJson(doc, out);
        mqttPublish(TOPIC_LIGHTNING, out);
    }
    // --- UV ---
    {
        JsonDocument doc;
        doc["temp_x100"] = p.UV.TEMP_C100;
        doc["uva"]       = p.UV.UVA;
        doc["uvb"]       = p.UV.UVB;
        doc["uvc"]       = p.UV.UVC;
        String out;
        serializeJson(doc, out);
        mqttPublish(TOPIC_UV, out);
    }
    // --- Air Quality ---
    {
        JsonDocument doc;
        doc["aqi"]  = p.AirQuality.AQI;
        doc["tvoc"] = p.AirQuality.TVOC;
        doc["eco2"] = p.AirQuality.eCO2;
        String out;
        serializeJson(doc, out);
        mqttPublish(TOPIC_AIR, out);
    }
    // --- Light RGB ---
    {
        JsonDocument doc;
        doc["clear"] = p.LightRGB.ClearChannel;
        doc["red"]   = p.LightRGB.RedChannel;
        doc["green"] = p.LightRGB.GreenChannel;
        doc["blue"]  = p.LightRGB.BlueChannel;
        String out;
        serializeJson(doc, out);
        mqttPublish(TOPIC_LIGHT_RGB, out);
    }
    // --- Light Lux ---
    {
        JsonDocument doc;
        doc["full"]    = p.Light.FullSpectrum;
        doc["ir"]      = p.Light.Infrared;
        doc["visible"] = p.Light.Visible;
        doc["lux"]     = serialized(String(p.Light.Lux, 2));
        String out;
        serializeJson(doc, out);
        mqttPublish(TOPIC_LIGHT_LUX, out);
    }
}

// ==========================================================================
// PUBLISH GPS
// ==========================================================================
static void publishGPS() {
    if (!myGNSS.isNewDataAvailable()) return;
    // Latitude / longitude / altitude are available after the parser
    // calls myGNSS.update() in the main loop.
    double lat    = myGNSS.getLatitude();
    double lon    = myGNSS.getLongitude();
    double alt    = myGNSS.getAltitudeMSL();    // metres above MSL
    float  hdop   = myGNSS.getHorizontalDOP();
    uint8_t sats  = myGNSS.getSatellitesInView();
    uint8_t fix   = myGNSS.getFixType();
    uint8_t rtk   = myGNSS.getRTKStatus();       // 0=none,1=float,2=fixed

    JsonDocument doc;
    doc["lat"]  = serialized(String(lat,  8));
    doc["lon"]  = serialized(String(lon,  8));
    doc["alt"]  = serialized(String(alt,  3));
    doc["hdop"] = serialized(String(hdop, 2));
    doc["sats"] = sats;
    doc["fix"]  = fix;
    doc["rtk"]  = rtk;     // 2 = RTK Fixed – centimetre accuracy
    String out;
    serializeJson(doc, out);
    mqttPublish(TOPIC_GPS, out);
}

// ==========================================================================
// STM32 FRAME RECEIVER
// Scans Serial1 byte-by-byte for the 0x55, 0xAA sync word, then accumulates
// exactly FRAME_SIZE bytes and validates the CRC.
// ==========================================================================
static void processSTMbyte(uint8_t b) {
    if (!syncFound) {
        // Shift a 2-byte window looking for SOF = 0x55 0xAA (little-endian)
        static uint8_t prev = 0;
        if (prev == UART_FRAME_SOF_B0 && b == UART_FRAME_SOF_B1) {
            // Found sync – place both bytes at the start of rxBuf
            rxBuf[0] = prev;
            rxBuf[1] = b;
            rxLen     = 2;
            syncFound = true;
        }
        prev = b;
        return;
    }

    // Accumulate remaining bytes
    if (rxLen < FRAME_SIZE) {
        rxBuf[rxLen++] = b;
    }

    if (rxLen == FRAME_SIZE) {
        syncFound = false;
        rxLen     = 0;

        // Overlay struct
        UART_Frame_t frame;
        memcpy(&frame, rxBuf, FRAME_SIZE);

        // Validate SOF
        if (frame.Start != UART_FRAME_START) {
            Serial.println("[STM] Bad SOF");
            return;
        }

        // Validate CRC
        uint16_t calc = CRC16(reinterpret_cast<const uint8_t*>(&frame.Payload),
                              sizeof(UART_Payload_t));
        if (calc != frame.CR) {
            Serial.printf("[STM] CRC mismatch: got 0x%04X, expected 0x%04X\n",
                          frame.CR, calc);
            return;
        }

        publishPayload(frame.Payload);
    }
}

// ==========================================================================
// SEND TO STM  (stub – for future downstream commands)
// Asserts GPIO13 LOW to signal "ESP is transmitting" before writing,
// then clears it.  The STM watches this on PA12 EXTI.
// ==========================================================================
static void sendToSTM(const uint8_t* data, size_t len) {
    // Only transmit if STM is not currently receiving something else
    if (digitalRead(PIN_STM_IS_TX) == HIGH) {
        Serial.println("[STM-TX] STM busy, skipped");
        return;
    }
    digitalWrite(PIN_ESP_IS_TX, LOW);
    Serial1.write(data, len);
    Serial1.flush();
    digitalWrite(PIN_ESP_IS_TX, HIGH);
}

// ==========================================================================
// NTRIP CLIENT  (lightweight – feeds RTCM3 corrections to Serial2 / GPS)
// Fill in the server credentials at the top of this file.
// ==========================================================================
static bool ntripConnected = false;
static uint32_t ntripLastAttempt = 0;

static void ntripLoop() {
    if (strlen(NTRIP_HOST) == 0) return;   // not configured
    if (!eth_connected) return;

    if (!ntripEthClient.connected()) {
        ntripConnected = false;
        if (millis() - ntripLastAttempt < 10000) return;
        ntripLastAttempt = millis();

        Serial.print("[NTRIP] Connecting to ");
        Serial.print(NTRIP_HOST);
        Serial.print(":");
        Serial.println(NTRIP_PORT);

        if (!ntripEthClient.connect(NTRIP_HOST, NTRIP_PORT)) {
            Serial.println("[NTRIP] Connection failed");
            return;
        }

        // Send NTRIP 1.0 request
        String auth = "";
        if (strlen(NTRIP_USER) > 0) {
            // Base64-encode user:pass  (simple manual encoding for small strings)
            String cred = String(NTRIP_USER) + ":" + String(NTRIP_PASS);
            // Use Arduino base64 if available, otherwise inline:
            // For brevity we use the raw bytes — replace with a proper
            // base64 library if credentials contain non-ASCII characters.
            auth = "Authorization: Basic ";
            // Simple ASCII base64 table
            static const char b64[] =
                "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            const uint8_t* in = reinterpret_cast<const uint8_t*>(cred.c_str());
            size_t n = cred.length();
            for (size_t i = 0; i < n; i += 3) {
                uint32_t val = (uint32_t)in[i] << 16;
                if (i+1 < n) val |= (uint32_t)in[i+1] << 8;
                if (i+2 < n) val |= (uint32_t)in[i+2];
                auth += b64[(val >> 18) & 0x3F];
                auth += b64[(val >> 12) & 0x3F];
                auth += (i+1 < n) ? b64[(val >>  6) & 0x3F] : '=';
                auth += (i+2 < n) ? b64[(val      ) & 0x3F] : '=';
            }
            auth += "\r\n";
        }

        String request = String("GET /") + NTRIP_MOUNTPOINT + " HTTP/1.0\r\n"
                       + "Host: " + NTRIP_HOST + "\r\n"
                       + "Ntrip-Version: Ntrip/1.0\r\n"
                       + "User-Agent: NTRIPClient/ESP32\r\n"
                       + auth
                       + "Connection: close\r\n\r\n";
        ntripEthClient.print(request);
        Serial.println("[NTRIP] Request sent, waiting for ICY 200 OK…");
    }

    // Forward any available RTCM bytes to the GPS
    while (ntripEthClient.available()) {
        uint8_t b = ntripEthClient.read();
        if (!ntripConnected) {
            // Discard HTTP header bytes until we see \r\n\r\n
            static uint8_t hdr[4] = {0};
            memmove(hdr, hdr+1, 3);
            hdr[3] = b;
            if (hdr[0]=='\r' && hdr[1]=='\n' && hdr[2]=='\r' && hdr[3]=='\n') {
                ntripConnected = true;
                Serial.println("[NTRIP] RTCM stream active");
            }
        } else {
            Serial2.write(b);   // forward correction bytes to LG290P
        }
    }
}

// ==========================================================================
// SETUP
// ==========================================================================
void setup() {
    Serial.begin(115200);
    delay(1000); // Let ESP do the Boot
    Serial.println("\n[WS] Weather Station ESP32 starting…");

    // --- Handshake pins ---
    pinMode(PIN_STM_IS_RX, INPUT);
    pinMode(PIN_ESP_IS_TX, OUTPUT);
	pinMode(PIN_GPS_RST, OUTPUT);
    digitalWrite(PIN_ESP_IS_TX, HIGH);
	digitalWrite(PIN_GPS_RST, HIGH);

    // Optional Hardware Reset: Ensure the LG290P starts fresh for RTK.
    digitalWrite(PIN_GPS_RST, LOW);  // Trigger Reset (Negative Logic) 
    delay(200);                      // Hold for 200ms
    digitalWrite(PIN_GPS_RST, HIGH); // Release Reset
    delay(1000); // Wait for LG290P to finish its internal boot cycle
    // --- STM32 UART ---
    // 115200 8N1 to match STM huart1 (adjust baud if STM side differs)
    Serial1.begin(115200, SERIAL_8N1, PIN_STM_RX, PIN_STM_TX);
    Serial.println("[WS] Serial1 (STM) ready");

    // --- LG290P GNSS UART ---
    // Default LG290P baud is 460800; if your module was reconfigured, change this.
    Serial2.begin(460800, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    if (myGNSS.begin(Serial2)) {
        Serial.println("[GPS] LG290P connected");
        // Request 1 Hz GGA + RMC output on UART1 of the LG290P
        myGNSS.setNMEAOutputRate(1);  // 1 Hz
        // Enable RTK status output (PQTM messages)
        // The SparkFun library automatically parses these.
    } else {
        Serial.println("[GPS] LG290P not detected – check wiring / baud rate");
    }

    // --- Ethernet ---
    Network.onEvent(onEvent);
    ETH.begin();
    Serial.println("[WS] Ethernet init…");

    // --- MQTT ---
    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setBufferSize(512);   // increase if JSON payloads exceed 256 bytes
    mqtt.setKeepAlive(30);
}

// ==========================================================================
// LOOP
// ==========================================================================
void loop() {
    // ---- MQTT keep-alive ----
    if (eth_connected) {
        if (!mqtt.connected()) {
            mqttReconnect();
        } else {
            mqtt.loop();
        }
    }

    // ---- STM32 UART receive ----
    while (Serial1.available()) {
        processSTMbyte(static_cast<uint8_t>(Serial1.read()));
    }

    // ---- GPS update ----
    // myGNSS.update() feeds bytes from Serial2 into the SparkFun parser.
    // Call it as often as possible to avoid UART buffer overflow at 460800.
    myGNSS.update();
    publishGPS();

    // ---- NTRIP corrections ----
    ntripLoop();
}
