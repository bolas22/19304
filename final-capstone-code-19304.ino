#include <DHT.h>
// Pin definitions
#define MQ4_PIN A0
#define MQ9_PIN A1
#define FLAME_SENSOR_PIN 2
#define BUZZER_PIN 3
#define DHTPIN 4
#define DHTTYPE DHT11

// Sensor thresholds (adjust these based on your specific requirements)
#define MQ4_THRESHOLD 400  // Methane threshold
#define MQ9_THRESHOLD 300  // CO threshold
#define HUMIDITY_THRESHOLD 70  // High humidity threshold (%)
#define TEMPERATURE_THRESHOLD 50  // High temperature threshold (°C)

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  pinMode(FLAME_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  dht.begin();
}

void loop() {
  // Read sensor values
  int mq4_value = analogRead(MQ4_PIN);
  int mq9_value = analogRead(MQ9_PIN);
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int flame_detected = digitalRead(FLAME_SENSOR_PIN);

  // Print sensor readings
  Serial.print("MQ-4 (Methane): ");
  Serial.print(mq4_value);
  Serial.print(" | MQ-9 (CO): ");
  Serial.print(mq9_value);
  Serial.print(" | Humidity: ");
  Serial.print(humidity);
  Serial.print("% | Temperature: ");
  Serial.print(temperature);
  Serial.print("°C | Flame: ");
  Serial.println(flame_detected ? "Detected" : "Not detected");

  // Check for potential fire conditions
  bool high_gas_level = (mq4_value > MQ4_THRESHOLD) || (mq9_value > MQ9_THRESHOLD);
  bool high_humidity = humidity > HUMIDITY_THRESHOLD;
  bool high_temperature = temperature > TEMPERATURE_THRESHOLD;

  // Relation between gas sensors and temperature/humidity
  if (high_gas_level && high_temperature) {
    Serial.println("Warning: High gas levels and high temperature detected!");
  }
  if (high_gas_level && high_humidity) {
    Serial.println("Warning: High gas levels and high humidity detected!");
  }

  // Activate buzzer if conditions are met
  if ((high_gas_level || high_humidity || high_temperature) && flame_detected) {
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("ALERT: Fire hazard detected! Activating buzzer.");
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  delay(2000);  // Wait for 2 seconds before next reading
}