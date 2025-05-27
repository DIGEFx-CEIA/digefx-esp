#include <Arduino.h>
#include <Preferences.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>

#define PIN_IGNITION 35 // GPIO for ignition signal
#define PIN_RELAY1 22   // GPIO for relay 1
#define PIN_RELAY2 23   // GPIO for relay 2
#define PIN_BATTERY 34  // GPIO ADC for battery reading

// Configuração do GPS
#define GPS_RX 16
#define GPS_TX 17
HardwareSerial GPSSerial(2); // Usando Serial2 para o GPS
TinyGPSPlus gps;

// Variáveis para debug do GPS
unsigned long lastGPSTest = 0;
const unsigned long GPS_TEST_INTERVAL = 1000; // Teste a cada 1 segundo
int gpsCharsProcessed = 0;
bool gpsDataReceived = false;
unsigned long startTime = 0; // Tempo de início da busca por fixação
bool isFirstFix = true; // Flag para primeira fixação
bool systemInitialized = false; // Flag para controle de inicialização

const int NUM_SAMPLES = 10;       // Number of readings for averaging
const float VOLTAGE_CALIBRATION = 0.00447222; // Calibration factor for voltage readings (((R1+R2)/R2)/1000)
const float DIODE_VOLTAGE = 0.8; // Diode voltage drop (0.8V for 1N5408)
float voltageBuffer[NUM_SAMPLES]; // Buffer for voltage readings
int sampleIndex = 0;              // Index for storing readings
bool bufferFilled = false;

bool ignitionOn = false;
float batteryVoltage = 0;
unsigned long relay1OffTime = 0;
unsigned long relay2OffTime = 0;
unsigned long relay1Time = 1 * 60000; // Configurable
unsigned long relay2Time = 1 * 60000; // Configurable
float minVoltage = 10.0;                // Configurable
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10000; // Send every 10 seconds

String deviceId = "DEVICE_1234"; // Device Identifier
bool finalStateSent = false;

Preferences preferences;

unsigned long lastMainProcessTime = 0; // Timer for main process to ensure 1 second delay
unsigned long lastSerialCheckTime = 0; // Timer for serial response check

// Comandos PMTK para otimização
const char PMTK_SET_NMEA_OUTPUT_RMCGGA[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"; // Apenas RMC e GGA
const char PMTK_SET_NMEA_UPDATE_1HZ[] = "$PMTK220,1000*1F"; // Atualização a cada 1 segundo
const char PMTK_API_SET_FIX_CTL_1HZ[] = "$PMTK300,1000,0,0,0,0*1C"; // Fix rate 1Hz
const char PMTK_SET_BAUD_9600[] = "$PMTK251,9600*17"; // Configura baud rate para 9600
const char PMTK_ENABLE_SBAS[] = "$PMTK313,1*2E"; // Habilita SBAS (WAAS/EGNOS)
const char PMTK_ENABLE_WAAS[] = "$PMTK301,2*2E"; // Habilita WAAS
const char PMTK_SET_DGPS_MODE[] = "$PMTK301,2*2E"; // Modo DGPS

void loadSettings() {
  preferences.begin("settings", true);
  minVoltage = preferences.getFloat("min_voltage", 10.0);
  relay1Time = preferences.getULong("relay1_time", 1 * 60000);
  relay2Time = preferences.getULong("relay2_time", 1 * 60000);
  deviceId = preferences.getString("device_id", "DEVICE_1234");
  preferences.end();
}

void saveSettings() {
  preferences.begin("settings", false);
  preferences.putFloat("min_voltage", minVoltage);
  preferences.putULong("relay1_time", relay1Time);
  preferences.putULong("relay2_time", relay2Time);
  preferences.putString("device_id", deviceId);
  preferences.end();
}
void sendStatus();
void handleSerialResponse();
void processCommand(String command);

void configureGPS() {
    Serial.println("Configurando GPS...");
    
    // Configurações básicas
    GPSSerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    delay(100);
    GPSSerial.println(PMTK_SET_NMEA_UPDATE_1HZ);
    delay(100);
    GPSSerial.println(PMTK_API_SET_FIX_CTL_1HZ);
    delay(100);
    GPSSerial.println(PMTK_SET_BAUD_9600);
    delay(100);
    
    // Configurações avançadas
    GPSSerial.println(PMTK_ENABLE_SBAS);
    delay(100);
    GPSSerial.println(PMTK_ENABLE_WAAS);
    delay(100);
    GPSSerial.println(PMTK_SET_DGPS_MODE);
    delay(100);
    
    Serial.println("Configuração do GPS concluída");
}

void setup()
{
    // Inicialização da Serial principal
    Serial.begin(115200);
    delay(100);
    
    // Limpa o buffer serial
    while(Serial.available()) {
        Serial.read();
    }
    
    // Inicialização do GPS
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(100);
    
    // Limpa o buffer do GPS
    while(GPSSerial.available()) {
        GPSSerial.read();
    }
    
    // Configuração inicial do sistema
    pinMode(PIN_IGNITION, INPUT);
    pinMode(PIN_RELAY1, OUTPUT);
    pinMode(PIN_RELAY2, OUTPUT);

    digitalWrite(PIN_RELAY1, LOW);
    digitalWrite(PIN_RELAY2, LOW);
    
    // Configuração do ADC
    analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
    
    // Inicialização do buffer de tensão
    uint16_t mV = analogReadMilliVolts(PIN_BATTERY);
    float initialReading = mV * VOLTAGE_CALIBRATION + DIODE_VOLTAGE;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        voltageBuffer[i] = initialReading;
    }
    
    // Carrega configurações
    loadSettings();
    
    // Configura o GPS
    configureGPS();
    
    startTime = millis();
    systemInitialized = true;
}

void sendStatus()
{
    String ignitionStatus = ignitionOn ? "On" : "Off";
    String relay1Status = (digitalRead(PIN_RELAY1) == HIGH) ? "On" : "Off";
    String relay2Status = (digitalRead(PIN_RELAY2) == HIGH) ? "On" : "Off";
    String gpsStatus = gps.location.isValid() ? "On" : "Off";

    String data = "DEVICE_ID:" + deviceId +
                  ";IGNITION:" + ignitionStatus +
                  ";BATTERY:" + batteryVoltage +
                  ";MIN_VOLTAGE:" + String(minVoltage) +
                  ";RELAY1:" + relay1Status +
                  ";RELAY1_TIME:" + String(relay1Time/60000) +
                  ";RELAY2:" + relay2Status +
                  ";RELAY2_TIME:" + String(relay2Time/60000) +
                  ";GPS_STATUS:" + gpsStatus;

    // Adiciona dados do GPS apenas se houver fixação válida
    if (gps.location.isValid()) {
        data += ";LAT:" + String(gps.location.lat(), 6) +
                ";LNG:" + String(gps.location.lng(), 6) +
                ";SPEED:" + String(gps.speed.kmph(), 1) +
                ";HDOP:" + String(gps.hdop.hdop(), 1) +
                ";SATS:" + String(gps.satellites.value());
    }

    data += "\n";
    Serial.print(data);
}

void handleSerialResponse()
{
    if (Serial.available())
    {
        String response = Serial.readStringUntil('\n');
        if (response == "ACK")
        {
            // Ready for the next transmission
        }
        else
        {
            processCommand(response);
        }
    }
}

void processCommand(String command)
{
    int separatorIndex = command.indexOf(':');
    if (separatorIndex != -1)
    {
        String key = command.substring(0, separatorIndex);
        String value = command.substring(separatorIndex + 1);

        key.trim();
        value.trim();

        if (key == "MIN_VOLTAGE")
        {
            minVoltage = value.toDouble();
            saveSettings();
            Serial.println("ACK");
        }
        else if (key == "RELAY1_TIME")
        {
            relay1Time = value.toInt() * 60000;
            saveSettings();
            Serial.println("ACK");
        }
        else if (key == "RELAY2_TIME")
        {
            relay2Time = value.toInt() * 60000;
            saveSettings();
            Serial.println("ACK");
        }
        else if (key == "DEVICE_ID")
        {
            deviceId = value;
            saveSettings();
            Serial.println("ACK");
        }
        else
        {
            Serial.println("Unknown key: " + key);
        }
    }
}

void loop()
{
    if (!systemInitialized) {
        return;
    }

    unsigned long currentMillis = millis();

    // Processa dados do GPS
    while (GPSSerial.available() > 0) {
        char c = GPSSerial.read();
        gps.encode(c);
    }

    // Verifica status do GPS a cada 5 segundos
    static unsigned long lastDebugTime = 0;
    if (currentMillis - lastDebugTime >= 5000) {
        Serial.println("\n=== Status do GPS ===");
        Serial.print("Satélites visíveis: ");
        Serial.println(gps.satellites.value());
        Serial.print("HDOP atual: ");
        Serial.println(gps.hdop.hdop(), 1);
        
        if (gps.location.isValid()) {
            Serial.println("Fixação válida!");
            Serial.print("Latitude: ");
            Serial.println(gps.location.lat(), 6);
            Serial.print("Longitude: ");
            Serial.println(gps.location.lng(), 6);
            Serial.print("Velocidade: ");
            Serial.println((int)floor(gps.speed.kmph()));
        } else {
            Serial.println("Aguardando fixação...");
        }
        Serial.println("==================\n");
        lastDebugTime = currentMillis;
    }

    // Main process runs every 1 second
    if (currentMillis - lastMainProcessTime >= 1000)
    {
        ignitionOn = digitalRead(PIN_IGNITION);

        uint16_t mV = analogReadMilliVolts(PIN_BATTERY);
        float currentVoltage = mV * VOLTAGE_CALIBRATION + DIODE_VOLTAGE;

        voltageBuffer[sampleIndex] = currentVoltage;
        sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

        if (sampleIndex == 0)
        {
            bufferFilled = true;
        }

        float sum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++)
        {
            sum += voltageBuffer[i];
        }
        batteryVoltage = sum / NUM_SAMPLES;

        if (ignitionOn)
        {
            digitalWrite(PIN_RELAY1, HIGH);
            digitalWrite(PIN_RELAY2, HIGH);
            relay1OffTime = millis() + relay1Time;
            relay2OffTime = millis() + relay2Time;
            finalStateSent = false;
        }
        else
        {
            if (millis() >= relay1OffTime)
            {
                digitalWrite(PIN_RELAY1, LOW);
            }
            if (millis() >= relay2OffTime)
            {
                digitalWrite(PIN_RELAY2, LOW);
            }
        }

        if (bufferFilled && batteryVoltage < minVoltage)
        {
            digitalWrite(PIN_RELAY1, LOW);
            digitalWrite(PIN_RELAY2, LOW);
        }
        
        // Send final state only once when all relays and ignition are off
        if (!ignitionOn && digitalRead(PIN_RELAY1) == LOW && digitalRead(PIN_RELAY2) == LOW)
        {
            if (!finalStateSent)
            {
                sendStatus();
                finalStateSent = true;
            }
        }
        else
        {
            finalStateSent = false; // Reset when system changes state
        }

        if (millis() - lastSendTime >= sendInterval)
        {
            sendStatus();
            lastSendTime = millis();
        }
        lastMainProcessTime = currentMillis;
    }

    // Handle serial response events at any time
    if (currentMillis - lastSerialCheckTime >= 1) 
    {
        handleSerialResponse();
        lastSerialCheckTime = currentMillis;
    }
}
