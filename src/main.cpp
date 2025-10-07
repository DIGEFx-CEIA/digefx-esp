#include <Arduino.h>
#include <Preferences.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>

#define PIN_IGNITION 34 // GPIO for ignition signal
#define PIN_RELAY1 22   // GPIO for relay 1
#define PIN_RELAY2 23   // GPIO for relay 2
#define PIN_BATTERY 35  // GPIO ADC for battery reading

// Pinos para LEDs de status
#define PIN_LED_CAMERA_1 25
#define PIN_LED_CAMERA_2 26
#define PIN_LED_CAMERA_3 27
#define PIN_LED_CAMERA_4 14
#define PIN_LED_PC         12
#define PIN_LED_INTERNET   13
#define PIN_BUZZER         21

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

// Variáveis para sistema de status e LEDs
bool pcOnline = false;
bool internetOnline = false;
bool applicationRunning = false;
bool camera1Online = false;
bool camera2Online = false;
bool camera3Online = false;
bool camera4Online = false;

// Sistema de heartbeat corrigido
unsigned long heartbeatTimer = 0;           // Timer para timeout do heartbeat
const unsigned long HEARTBEAT_TIMEOUT = 10000; // 10 segundos sem heartbeat = falha
unsigned long lastStatusUpdate = 0;
bool heartbeatSystemActive = false;         // Se o sistema de heartbeat está ativo
bool buzzerAlertTriggered = false;          // Evita múltiplos alertas de buzzer

// Comandos PMTK para otimização
const char PMTK_SET_NMEA_OUTPUT_RMCGGA[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"; // Apenas RMC e GGA
const char PMTK_SET_NMEA_UPDATE_1HZ[] = "$PMTK220,1000*1F"; // Atualização a cada 1 segundo
const char PMTK_API_SET_FIX_CTL_1HZ[] = "$PMTK300,1000,0,0,0,0*1C"; // Fix rate 1Hz
const char PMTK_SET_BAUD_9600[] = "$PMTK251,9600*17"; // Configura baud rate para 9600
const char PMTK_ENABLE_SBAS[] = "$PMTK313,1*2E"; // Habilita SBAS (WAAS/EGNOS)
const char PMTK_ENABLE_WAAS[] = "$PMTK301,2*2E"; // Habilita WAAS
const char PMTK_SET_DGPS_MODE[] = "$PMTK301,2*2E"; // Modo DGPS

const int ledPins[] = {PIN_LED_CAMERA_1, PIN_LED_CAMERA_2, PIN_LED_CAMERA_3, PIN_LED_CAMERA_4, PIN_LED_PC, PIN_LED_INTERNET};
const int numLeds = sizeof(ledPins) / sizeof(ledPins[0]);

void playTone(int half_period_us, int duration_ms) {
  long duration_us = duration_ms * 1000L;
  for (long i = 0; i < duration_us; i += half_period_us * 2) {
    digitalWrite(PIN_BUZZER, HIGH);
    delayMicroseconds(half_period_us);
    digitalWrite(PIN_BUZZER, LOW);
    delayMicroseconds(half_period_us);
  }
}

void updateLEDStatus() {
  /**
   * Atualiza status dos LEDs baseado no estado do sistema
   */
  digitalWrite(PIN_LED_PC, pcOnline && applicationRunning ? HIGH : LOW);
  digitalWrite(PIN_LED_INTERNET, internetOnline ? HIGH : LOW);
  digitalWrite(PIN_LED_CAMERA_1, camera1Online ? HIGH : LOW);
  digitalWrite(PIN_LED_CAMERA_2, camera2Online ? HIGH : LOW);
  digitalWrite(PIN_LED_CAMERA_3, camera3Online ? HIGH : LOW);
  digitalWrite(PIN_LED_CAMERA_4, camera4Online ? HIGH : LOW);
}

void processStatusCommand(String command) {
  /**
   * Processa comandos de status recebidos do Python
   * Formato: STATUS:PC:1,INTERNET:1,APP:1,CAM1:1,CAM2:0,CAM3:1,CAM4:0
   */
  if (command.startsWith("STATUS:")) {
    String statusData = command.substring(7); // Remove "STATUS:"
    
    // Parse dos parâmetros
    int index = 0;
    while (statusData.length() > 0) {
      int commaIndex = statusData.indexOf(',');
      String param;
      
      if (commaIndex == -1) {
        param = statusData;
        statusData = "";
      } else {
        param = statusData.substring(0, commaIndex);
        statusData = statusData.substring(commaIndex + 1);
      }
      
      // Processar parâmetro (formato: NOME:VALOR)
      int colonIndex = param.indexOf(':');
      if (colonIndex != -1) {
        String name = param.substring(0, colonIndex);
        int value = param.substring(colonIndex + 1).toInt();
        bool status = (value == 1);
        
        // Atualizar variáveis correspondentes
        if (name == "PC") {
          pcOnline = status;
        } else if (name == "INTERNET") {
          internetOnline = status;
        } else if (name == "APP") {
          applicationRunning = status;
        } else if (name == "CAM1") {
          camera1Online = status;
        } else if (name == "CAM2") {
          camera2Online = status;
        } else if (name == "CAM3") {
          camera3Online = status;
        } else if (name == "CAM4") {
          camera4Online = status;
        }
      }
    }
    
    // Atualizar LEDs
    updateLEDStatus();
    lastStatusUpdate = millis();
    
    Serial.println("ACK");
    
  } else if (command.startsWith("HEARTBEAT:")) {
    // Processar heartbeat - RESETAR o timer
    heartbeatTimer = millis();              // Reset do timer
    heartbeatSystemActive = true;           // Ativar sistema de heartbeat
    applicationRunning = true;              // Aplicação está viva
    pcOnline = true;                        // PC está vivo
    buzzerAlertTriggered = false;           // Reset do alerta de buzzer
    
    updateLEDStatus();                      // Atualizar LEDs imediatamente
    
    Serial.println("ACK");
    
  } else if (command.startsWith("INIT:")) {
    // Comando de inicialização - INICIAR sistema de heartbeat
    heartbeatTimer = millis();               // Inicializar timer
    heartbeatSystemActive = true;            // Ativar sistema
    buzzerAlertTriggered = false;            // Reset alerta
    pcOnline = true;                         // PC está conectado
    applicationRunning = true;               // Aplicação iniciando
    
    updateLEDStatus();                       // Atualizar LEDs
    
    Serial.println("ESP32_READY");
    
  } else if (command.startsWith("SHUTDOWN:")) {
    // Comando de desligamento - DESATIVAR sistema de heartbeat
    heartbeatSystemActive = false;           // Desativar sistema
    buzzerAlertTriggered = false;            // Reset alerta
    pcOnline = false;                        // PC desligando
    applicationRunning = false;              // Aplicação parando
    
    // Desligar todas as câmeras também
    camera1Online = false;
    camera2Online = false;
    camera3Online = false;
    camera4Online = false;
    internetOnline = false;
    
    updateLEDStatus();                       // Apagar todos os LEDs
    Serial.println("ACK");
  }
}

void checkHeartbeatTimeout() {
  /**
   * Verifica timeout do heartbeat com lógica corrigida
   * Timer é resetado a cada heartbeat recebido
   */
  
  // Só verificar timeout se o sistema de heartbeat estiver ativo
  if (heartbeatSystemActive) {
    unsigned long timeSinceLastHeartbeat = millis() - heartbeatTimer;
    
    // Se passou do timeout E ainda não acionou o buzzer
    if (timeSinceLastHeartbeat > HEARTBEAT_TIMEOUT && !buzzerAlertTriggered) {
      
      // Aplicação parou de responder - FALHA CRÍTICA
      applicationRunning = false;
      pcOnline = false;
      updateLEDStatus();
      
      // Acionar buzzer de falha (3 beeps longos)
      for (int i = 0; i < 3; i++) {
        playTone(800, 300);   // Beep grave de 300ms
        delay(300);           // Pausa de 300ms
      }
      
      buzzerAlertTriggered = true;  // Marcar que buzzer já foi acionado
      
      Serial.println("HEARTBEAT_TIMEOUT"); // Enviar para debug
    }
  }
}

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
    
    // Aguardar estabilização da serial (importante!)
    delay(500);
    
    // Limpar buffer serial completamente (incluindo caracteres de boot)
    Serial.flush();  // Aguarda transmissão pendente
    while(Serial.available()) {
        Serial.read();
    }
    
    // Aguardar mais um pouco para garantir
    delay(100);
    
    // Limpar novamente (caracteres do bootloader podem ainda estar chegando)
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

    // Configuração dos pinos dos LEDs
    for (int i = 0; i < numLeds; i++) {
        pinMode(ledPins[i], OUTPUT);
    }
    pinMode(PIN_BUZZER, OUTPUT);

    digitalWrite(PIN_RELAY1, LOW);
    digitalWrite(PIN_RELAY2, LOW);
    
    // Som de boot estilo game retrô (Power-up)
    playTone(1515, 120); // Nota E4
    delay(30);
    playTone(1205, 120); // Nota G#4
    delay(30);
    playTone(758, 120);  // Nota E5

    // Pisca os LEDs 5 vezes para indicar inicialização
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < numLeds; j++) {
            digitalWrite(ledPins[j], HIGH);
        }
        delay(500);
        for (int j = 0; j < numLeds; j++) {
            digitalWrite(ledPins[j], LOW);
        }
        delay(500);
    }
    

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
    
    // Enviar mensagem de boot completo (após limpar garbage)
    Serial.println("\n\n================================");
    Serial.println("ESP32 DIGEFX - Sistema Iniciado");
    Serial.println("================================");
    Serial.print("Device ID: ");
    Serial.println(deviceId);
    Serial.print("Firmware Version: 2.0");
    Serial.println("\n================================\n");
    
    systemInitialized = true;
}

void sendStatus()
{
    String ignitionStatus = ignitionOn ? "On" : "Off";
    String relay1Status = (digitalRead(PIN_RELAY1) == HIGH) ? "On" : "Off";
    String relay2Status = (digitalRead(PIN_RELAY2) == HIGH) ? "On" : "Off";
    String gpsStatus = gps.location.isValid() ? "Valid" : "Invalid";

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
        // Ler linha com tratamento robusto de caracteres inválidos
        String response = "";
        unsigned long startRead = millis();
        const unsigned long READ_TIMEOUT = 100; // 100ms timeout para leitura
        
        while (Serial.available() && (millis() - startRead) < READ_TIMEOUT)
        {
            int c = Serial.read();
            
            // Verificar se é caractere válido
            if (c == -1) {
                // Fim do buffer
                break;
            }
            else if (c == '\n') {
                // Fim da linha
                break;
            }
            else if (c == '\r') {
                // Ignorar CR (comum em Windows)
                continue;
            }
            else if (c >= 32 && c <= 126) {
                // Caracteres ASCII imprimíveis válidos
                response += (char)c;
            }
            else {
                // Caractere inválido - enviar debug
                Serial.print("DEBUG:Invalid_char_0x");
                Serial.println(c, HEX);
            }
            
            // Pequena pausa para permitir chegada de mais dados
            if (Serial.available() == 0) {
                delay(5);
            }
        }
        
        // Processar apenas se houver dados válidos
        if (response.length() > 0)
        {
            response.trim(); // Remover espaços em branco extras
            
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
}

void processCommand(String command)
{
    // Primeiro verificar se é comando de status (não tem separador ':' simples)
    if (command.startsWith("STATUS:") || command.startsWith("HEARTBEAT:") || 
        command.startsWith("INIT:") || command.startsWith("SHUTDOWN:")) {
        processStatusCommand(command);
        return;
    }
    
    // Processar comandos de configuração tradicionais
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
    
    // Verificar timeout do heartbeat a cada 2 segundos
    static unsigned long lastHeartbeatCheck = 0;
    if (currentMillis - lastHeartbeatCheck >= 2000) {
        checkHeartbeatTimeout();
        lastHeartbeatCheck = currentMillis;
    }
}
