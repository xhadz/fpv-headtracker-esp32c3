#if defined(ESP32)
#pragma message "ESP32 stuff happening!"
#else
#pragma message "ESP8266 stuff happening!"
#endif

// ===== Logging Level =====
#define LOG_LEVEL LOG_LEVEL_DEBUG
#include "logging.h"

// ======================================================
// 1. Includes
// ======================================================

#include <terseCRSF.h>
#include "msp.h"
#include "msptypes.h"
#include "fake_vrx_fake_trainer.h"
#include <math.h>
#include "application.h"
#include "types.h"
#include "../uv.lock/lock.h"   // Private secrets — not committed to git

// ======================================================
// 2. Platform Selection (ESP32 / ESP8266)
// ======================================================

#if defined(ESP32)
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif

enum RadioMode
{
    MODE_RX_ONLY = 0,
    MODE_TX_ONLY = 1,
    MODE_BOTH    = 2
};

// ======================================================
// 3. Configuration (Defines, UID, WiFi, Logging)
// ======================================================

// TON UID — value defined in uv.lock/lock.h (not committed)
uint8_t UID[6] = SECRET_UID_BYTES;

RadioMode radioMode = MODE_BOTH;

const unsigned long CONFIG_WINDOW_MS = 5000;

// ===== AP Wifi Config — values defined in uv.lock/lock.h (not committed) =====
const char* ssid     = SECRET_WIFI_SSID;
const char* password = SECRET_WIFI_PASSWORD;

// ===== Config for Channel output =====
const uint8_t NUM_CHANNELS = 16;
#define CHANNEL_MIN 172
#define CHANNEL_MAX 1811
#define WAVE_SPEED 0.05
float wavePhase = 0.0;
#define STEP_INTERVAL 100

// ===== Reglages head tracking =====
#define HT_LIMIT_DEG   100.0f   // plage de tete +/- (degres)
#define PAN_GAIN       1.0f     // sensibilite pan  (1.0 = normal, >1 plus sensible)
#define TILT_GAIN      1.0f     // sensibilite tilt
#define PAN_REVERSE    false    // inverse le sens du pan
#define TILT_REVERSE   false    // inverse le sens du tilt

// ===== Bouton (recentrage = appui court, hold = appui long) =====
#define RECENTER_PIN   3        // poussoir entre GPIO3 et GND
#define LONGPRESS_MS   500      // duree pour declencher le hold

// ======================================================
// BNO085 Head Tracking  (broches C3 : SDA=8, SCL=9)
// ======================================================
#include <Adafruit_BNO08x.h>
#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9
Adafruit_BNO08x bno(-1);
sh2_SensorValue_t htEvent;

// Quaternion de reference (le "centre" capture au recentrage)
float refQr = 1, refQi = 0, refQj = 0, refQk = 0;
bool haveRef = false;

// Etat hold
bool holdActive = false;
float holdPan = 0, holdTilt = 0;   // valeurs figees pendant le hold (en degres)

void initBNO085() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    delay(100);
    bool ok = false;
    for (int i = 0; i < 10; i++) {
        if (bno.begin_I2C(0x4B, &Wire) || bno.begin_I2C(0x4A, &Wire)) { ok = true; break; }
        LOG_ERROR("BNO085 retry...");
        delay(200);
    }
    if (ok) {
        bno.enableReport(SH2_ARVR_STABILIZED_RV, 5000);
        LOG_INFO("BNO085 OK");
    } else {
        LOG_ERROR("BNO085 absent - on continue sans");
    }
}

// Normalise un angle dans [-180, 180] en gerant le wrap-around
float normalizeAngle(float deg) {
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

uint16_t angleToChannel(float deg) {
    if (deg < -HT_LIMIT_DEG) deg = -HT_LIMIT_DEG;
    if (deg >  HT_LIMIT_DEG) deg =  HT_LIMIT_DEG;
    float n = (deg + HT_LIMIT_DEG) / (2.0f * HT_LIMIT_DEG);
    return (uint16_t)(CHANNEL_MIN + n * (CHANNEL_MAX - CHANNEL_MIN));
}

// ======================================================
// 4. Global Objects
// ======================================================

unsigned long configWindowStart = 0;
uint16_t rampChannels[NUM_CHANNELS];
uint32_t lastStepTime = 0;
uint8_t activeChannel = 0;
bool rampUp = true;

volatile bool espnow_received = false;
volatile uint16_t espnow_len = 0;
uint8_t espnow_buffer[250];
volatile uint16_t crsf_len = 0;

#if defined(ESP32)
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#endif

#if defined(ESP32)
QueueHandle_t rxqueue;
#endif

FakeVRXFakeTrainer vrxModule;
MSP recv_msp;
CRSF crsf;

uint16_t rc_channels[NUM_CHANNELS] = {0};

int16_t hud_bat1_volts = 0;
int16_t hud_bat1_amps  = 0;
uint16_t hud_bat1_mAh  = 0;

bool motArmed = false;

Location hom = {0,0,0,0,0};
Location cur = {0,0,0,0,0};

bool finalHomeStored = false;

// ======================================================
// ESP-NOW Receive Callback
// ======================================================
#if defined(ESP32)
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
#endif

    if (len <= 8 || incomingData == NULL) return;

    if (incomingData[8] == 0 && incomingData[9] == 0 && len > 10) {
        return;
    }

    crsf_len = len - 8;

    if (crsf_len > sizeof(espnow_buffer)) {
        crsf_len = sizeof(espnow_buffer);
    }

    memcpy(espnow_buffer, incomingData + 8, crsf_len);

    espnow_received = true;
}

// ======================================================
// ESP-NOW Send Callback
// ======================================================
#if defined(ESP32)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    bool success = (status == ESP_NOW_SEND_SUCCESS);
#else
void OnDataSent(uint8_t *mac_addr, uint8_t status) {
    bool success = (status == 0);
#endif

    if (success) {
        LOG_DEBUG_INLINE("ESP-NOW Send OK          ");
    } else {
        LOG_ERROR_INLINE("ESP-NOW Send FAIL (Status: %d) t=%lu ms      ", status, millis());
    }
}

// ======================================================
// 7. Setup()
// ======================================================

void initSerial()
{
    Serial.begin(115200);
    LOG_INFO("Start");
}

void initWiFi()
{
    UID[0] &= ~0x01;   // unicast fix
    WiFi.disconnect();

    #if defined(ESP32)
        WiFi.mode(WIFI_STA);
        // OBLIGATOIRE pour le raw frame (fix du 257) :
        esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
    #else
        WiFi.mode(WIFI_STA);
    #endif
}

void initMac()
{
    #if defined(ESP32)
        esp_wifi_set_mac(WIFI_IF_STA, UID);
        esp_wifi_start();
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    #else
        wifi_set_macaddr(STATION_IF, UID);
        wifi_set_channel(1);
    #endif
}

void initESPNow()
{
    #if defined(ESP32)
        if (esp_now_init() != ESP_OK)
    #else
        if (esp_now_init() != 0)
    #endif
    {
        LOG_ERROR("ESP-NOW init failed");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    #if defined(ESP32)
        // Pas de peer : le custom_esp_now_send bypass passe a cote de l'API
    #else
        esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
        esp_now_add_peer(UID, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
    #endif
}

void initRamp()
{
    for(int i = 0; i < NUM_CHANNELS; i++)
        rampChannels[i] = CHANNEL_MIN;
}

void initESP32Queue()
{
#if defined(ESP32)
    rxqueue = xQueueCreate(20, sizeof(mspPacket_t));
    if (rxqueue == NULL)
        LOG_ERROR("Queue creation failed");
#endif
}

void initInfo()
{
    LOG_INFO("==================================================");
    LOG_INFO("Radio Mode Configuration");
    LOG_INFO("Config window active for %lu ms", CONFIG_WINDOW_MS);
    LOG_INFO("  1 = RX_ONLY   2 = TX_ONLY   3 = BOTH");
    LOG_INFO("==================================================");
    configWindowStart = millis();
}

void setup() {
    initSerial();
    pinMode(RECENTER_PIN, INPUT_PULLUP);
    initBNO085();
    delay(3000);
    initWiFi();
    initMac();
    initESP32Queue();
    initESPNow();
    vrxModule.init(UID);
    initRamp();
    initInfo();
}

// ======================================================
// 8. Loop()
// ======================================================

// Multiplie deux quaternions : out = a * b
void quatMul(float aw,float ax,float ay,float az,
             float bw,float bx,float by,float bz,
             float &ow,float &ox,float &oy,float &oz) {
    ow = aw*bw - ax*bx - ay*by - az*bz;
    ox = aw*bx + ax*bw + ay*bz - az*by;
    oy = aw*by - ax*bz + ay*bw + az*bx;
    oz = aw*bz + ax*by - ay*bx + az*bw;
}

// ======================================================
// Extraction Pan / Tilt depuis un quaternion relatif
// Choisir la methode ici :
//   0 = LOOK_VECTOR  (simple, coupling aux grands angles)
//   1 = EULER_ZYX    (decouple, recommande)
// ======================================================
#define PAN_TILT_METHOD 1

// Methode 0 : vecteur regard (Z forward) -> spherique monde
// Simple mais pan/tilt se contaminent mutuellement aux grands angles.
void extractPanTilt_LookVector(float rw, float rx, float ry, float rz,
                               float &pan, float &tilt) {
    const float R2D = 57.29578f;
    float vx = 2.0f*(rx*rz + rw*ry);
    float vy = 2.0f*(ry*rz - rw*rx);
    float vz = 1.0f - 2.0f*(rx*rx + ry*ry);
    pan  = asinf(constrain(vx, -1.0f, 1.0f)) * R2D;
    tilt = atan2f(vy, sqrtf(vx*vx + vz*vz)) * R2D;
}

// Methode 1 : Euler ZYX intrinseque (yaw=pan, pitch=tilt)
// Pan et tilt sont extraits dans le repere propre de la tete
// -> pas de contamination croisee aux grands angles.
void extractPanTilt_EulerZYX(float rw, float rx, float ry, float rz,
                              float &pan, float &tilt) {
    const float R2D = 57.29578f;
    float sinPan = 2.0f*(rw*rz + rx*ry);
    float cosPan = 1.0f - 2.0f*(ry*ry + rz*rz);
    pan  = atan2f(sinPan, cosPan) * R2D;
    float sinTilt = 2.0f*(rw*ry - rz*rx);
    tilt = asinf(constrain(sinTilt, -1.0f, 1.0f)) * R2D;
}

// Dispatcher — appelle la methode choisie par PAN_TILT_METHOD
void extractPanTilt(float rw, float rx, float ry, float rz,
                    float &pan, float &tilt) {
#if PAN_TILT_METHOD == 0
    extractPanTilt_LookVector(rw, rx, ry, rz, pan, tilt);
#else
    extractPanTilt_EulerZYX(rw, rx, ry, rz, pan, tilt);
#endif
}

void loop()
{
    // ---- Serial Mode Switch (config window) ----
    if (millis() - configWindowStart < CONFIG_WINDOW_MS)
    {
        if (Serial.available())
        {
            char c = Serial.read();
            if (c >= '1' && c <= '3')
            {
                radioMode = (RadioMode)(c - '1');
                LOG_INFO("RadioMode changed to %d", radioMode + 1);
            }
        }
    }

    // ---- ESP-NOW Receive Handling (CRSF) ----
    if (radioMode != MODE_TX_ONLY)
    {
        if (espnow_received)
        {
            uint16_t len_to_process = 0;

            #if defined(ESP32)
            portENTER_CRITICAL(&mux);
            #else
            noInterrupts();
            #endif

            len_to_process = crsf_len;
            memcpy(crsf.crsf_buf, espnow_buffer, len_to_process);
            espnow_received = false;

            #if defined(ESP32)
            portEXIT_CRITICAL(&mux);
            #else
            interrupts();
            #endif

            processCRSFFrame(crsf.crsf_buf, len_to_process);
        }
    }

    // ---- Head Tracking Send ----
    if (radioMode != MODE_RX_ONLY)
    {
        if (bno.wasReset()) bno.enableReport(SH2_ARVR_STABILIZED_RV, 5000);

        static uint16_t panCh  = (CHANNEL_MIN + CHANNEL_MAX) / 2;
        static uint16_t tiltCh = (CHANNEL_MIN + CHANNEL_MAX) / 2;

        static float lastQr = 1, lastQi = 0, lastQj = 0, lastQk = 0;
        static float curPan = 0, curTilt = 0;   // angles courants (degres)

        if (bno.getSensorEvent(&htEvent) &&
            htEvent.sensorId == SH2_ARVR_STABILIZED_RV)
        {
            float qr = htEvent.un.arvrStabilizedRV.real;
            float qi = htEvent.un.arvrStabilizedRV.i;
            float qj = htEvent.un.arvrStabilizedRV.j;
            float qk = htEvent.un.arvrStabilizedRV.k;

            lastQr = qr; lastQi = qi; lastQj = qj; lastQk = qk;

            if (!haveRef) {
                refQr = qr; refQi = qi; refQj = qj; refQk = qk;
                haveRef = true;
            }

            // quaternion relatif = conjugue(ref) * actuel
            float rw, rx, ry, rz;
            quatMul(refQr, -refQi, -refQj, -refQk,
                    qr, qi, qj, qk,
                    rw, rx, ry, rz);

            extractPanTilt(rw, rx, ry, rz, curPan, curTilt);
        }

        // ---- Gestion bouton : court = recentrage, long = hold ----
        static bool lastBtn = HIGH;
        static uint32_t pressStart = 0;
        static bool longFired = false;
        bool btn = digitalRead(RECENTER_PIN);

        if (lastBtn == HIGH && btn == LOW) {
            // debut d'appui
            pressStart = millis();
            longFired = false;
        }
        if (btn == LOW && !longFired && (millis() - pressStart >= LONGPRESS_MS)) {
            // appui long atteint -> bascule HOLD
            longFired = true;
            holdActive = !holdActive;
            if (holdActive) {
                holdPan = curPan; holdTilt = curTilt;   // fige la position actuelle
                LOG_INFO("HOLD ON");
            } else {
                LOG_INFO("HOLD OFF");
            }
        }
        if (lastBtn == LOW && btn == HIGH) {
            // relachement : si c'etait un appui court -> recentrage
            if (!longFired) {
                refQr = lastQr; refQi = lastQi; refQj = lastQj; refQk = lastQk;
                haveRef = true;
                holdActive = false;   // un recentrage annule le hold
                LOG_INFO("Recentre !");
            }
        }
        lastBtn = btn;

        // ---- Choix des angles a envoyer (hold ou live) ----
        float outPan  = holdActive ? holdPan  : curPan;
        float outTilt = holdActive ? holdTilt : curTilt;

        // gain + reverse + normalisation
        outPan  = normalizeAngle(outPan)  * PAN_GAIN  * (PAN_REVERSE  ? -1.0f : 1.0f);
        outTilt = normalizeAngle(outTilt) * TILT_GAIN * (TILT_REVERSE ? -1.0f : 1.0f);

        panCh  = angleToChannel(outPan);
        tiltCh = angleToChannel(outTilt);

        static uint32_t lastSend = 0;
        if (millis() - lastSend >= 15)
        {
            lastSend = millis();
            uint16_t mid = (CHANNEL_MIN + CHANNEL_MAX) / 2;
            vrxModule.sendFakeHeadtracking(panCh, mid, tiltCh);
        }
    }
}