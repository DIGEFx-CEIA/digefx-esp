#include <Arduino.h>
#include <Preferences.h>

#define PIN_IGNITION 35 // GPIO for ignition signal
#define PIN_RELAY1 22   // GPIO for relay 1
#define PIN_RELAY2 23   // GPIO for relay 2
#define PIN_BATTERY 34  // GPIO ADC for battery reading

const int NUM_SAMPLES = 10;       // Number of readings for averaging
const float VOLTAGE_CALIBRATION = 0.003663004; // Calibration factor for voltage readings (15/4095)
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

void setup()
{
    Serial.begin(115200);
    pinMode(PIN_IGNITION, INPUT);
    pinMode(PIN_RELAY1, OUTPUT);
    pinMode(PIN_RELAY2, OUTPUT);

    digitalWrite(PIN_RELAY1, LOW);
    digitalWrite(PIN_RELAY2, LOW);
    loadSettings();

    float initialReading = analogRead(PIN_BATTERY) * VOLTAGE_CALIBRATION;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        voltageBuffer[i] = initialReading;
    }
}

void sendStatus()
{
    String ignitionStatus = ignitionOn ? "On" : "Off";
    String relay1Status = (digitalRead(PIN_RELAY1) == HIGH) ? "On" : "Off";
    String relay2Status = (digitalRead(PIN_RELAY2) == HIGH) ? "On" : "Off";

    String data = "DEVICE_ID:" + deviceId +
                  ";IGNITION:" + ignitionStatus +
                  ";BATTERY:" + batteryVoltage +
                  ";MIN_VOLTAGE:" + String(minVoltage) +
                  ";RELAY1:" + relay1Status +
                  ";RELAY1_TIME:" + String(relay1Time/60000) +
                  ";RELAY2:" + relay2Status + 
                  ";RELAY2_TIME:" + String(relay2Time/60000) + "\n";
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
    ignitionOn = digitalRead(PIN_IGNITION);
    float currentVoltage = analogRead(PIN_BATTERY) * VOLTAGE_CALIBRATION;

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

    handleSerialResponse();
}
