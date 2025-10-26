#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11);  // RX=10, TX=11 for Arduino Pro Mini

int DE = 6;
int RE = 7;

// ---------- CRC Calculation ----------
uint16_t calculateCRC(byte *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// ---------- Helper Function to Print Table Rows ----------
void printRow(const char *label, String value) {
  Serial.print("| ");
  Serial.print(label);
  for (int i = strlen(label); i < 18; i++) Serial.print(" ");
  Serial.print(" | ");
  Serial.print(value);
  for (int i = value.length(); i < 24; i++) Serial.print(" ");
  Serial.println("|");
}

// ---------- Function to Read from NPK Sensor ----------
bool readNPKSensor(byte functionCode) {
  const uint8_t deviceAddress = 0x01;
  const uint16_t startRegister = 0x0000;
  const uint16_t numRegisters = 7;

  byte query[6];
  query[0] = deviceAddress;
  query[1] = functionCode;
  query[2] = highByte(startRegister);
  query[3] = lowByte(startRegister);
  query[4] = highByte(numRegisters);
  query[5] = lowByte(numRegisters);

  uint16_t crc = calculateCRC(query, 6);
  byte fullQuery[8];
  for (int i = 0; i < 6; i++) fullQuery[i] = query[i];
  fullQuery[6] = crc & 0xFF;      // CRC low
  fullQuery[7] = crc >> 8;        // CRC high

  // Send query via RS485
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(2);
  mySerial.write(fullQuery, 8);
  mySerial.flush();
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  delay(300); // Wait for sensor to respond

  byte received[32];
  int len = mySerial.readBytes(received, sizeof(received));

  // Remove stray 0x00 if present
  if (len > 0 && received[0] == 0x00) {
    for (int i = 0; i < len - 1; i++) received[i] = received[i + 1];
    len--;
  }

  // Validate response
  if (len > 5 && received[0] == deviceAddress && received[1] == functionCode) {
    int regCount = received[2] / 2;
    int regs[10] = {0};

    for (int i = 0; i < regCount; i++) {
      regs[i] = (received[3 + i * 2] << 8) | received[4 + i * 2];
    }

    // Optional correction: if Potassium == 0, clear others (some sensors behave like this)
    if (regs[6] == 0) {
      regs[0] = 0;
      regs[4] = 0;
    }

    // Print results table
    Serial.println("--------------------------------------------------");
    Serial.println("| Parameter          | Value                     |");
    Serial.println("--------------------------------------------------");
    printRow("Soil Humidity", String(regs[0] / 10.0, 1) + " %");
    printRow("Soil Temp", String(regs[1] / 15.0, 1) + " °C");
    printRow("Soil Conductivity", String(regs[2]) + " uS/cm");
    printRow("Soil pH", String(regs[3] / 39.5, 2));
    printRow("Nitrogen", String(regs[4] / 15.0, 1) + " mg/kg");
    printRow("Phosphorus", String(regs[5]) + " mg/kg");
    printRow("Potassium", String(regs[6]) + " mg/kg");
    Serial.println("--------------------------------------------------");

    return true;
  }

  return false;
}

void setup() {
  Serial.begin(38400);
  mySerial.begin(9600); // Sensor baud rate (try 4800 if no response)

  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  Serial.println("🌱 NPK Sensor Reader (Arduino Pro Mini + RS485)");
  Serial.println("--------------------------------------------------");
}

void loop() {
  if (!readNPKSensor(0x03)) {
    if (!readNPKSensor(0x04)) {
      Serial.println("⚠ No response received or wrong function code");
      Serial.println("--------------------------------------------------");
    }
  }

  delay(2000); // Wait 2 seconds before next reading
}
