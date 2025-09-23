/*
 * QMC5883P Test Sketch
 *
 * Basic test for the QMC5883P 3-axis magnetometer
 */

#include <Adafruit_QMC5883P.h>

Adafruit_QMC5883P qmc;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("QMC5883P Test");

  if (!qmc.begin()) {
    Serial.println("Failed to find QMC5883P chip");
    while (1)
      delay(10);
  }

  Serial.println("QMC5883P Found!");

  // Set to normal mode
  qmc.setMode(QMC5883P_MODE_NORMAL);

  qmc5883p_mode_t currentMode = qmc.getMode();
  Serial.print("Mode: ");
  switch (currentMode) {
    case QMC5883P_MODE_SUSPEND:
      Serial.println("Suspend");
      break;
    case QMC5883P_MODE_NORMAL:
      Serial.println("Normal");
      break;
    case QMC5883P_MODE_SINGLE:
      Serial.println("Single");
      break;
    case QMC5883P_MODE_CONTINUOUS:
      Serial.println("Continuous");
      break;
    default:
      Serial.println("Unknown");
      break;
  }

  // Set ODR (Output Data Rate) to 50Hz
  qmc.setODR(QMC5883P_ODR_50HZ);
  qmc5883p_odr_t currentODR = qmc.getODR();
  Serial.print("ODR (Output Data Rate): ");
  switch (currentODR) {
    case QMC5883P_ODR_10HZ:
      Serial.println("10Hz");
      break;
    case QMC5883P_ODR_50HZ:
      Serial.println("50Hz");
      break;
    case QMC5883P_ODR_100HZ:
      Serial.println("100Hz");
      break;
    case QMC5883P_ODR_200HZ:
      Serial.println("200Hz");
      break;
    default:
      Serial.println("Unknown");
      break;
  }

  // Set OSR (Over Sample Ratio) to 4
  qmc.setOSR(QMC5883P_OSR_4);
  qmc5883p_osr_t currentOSR = qmc.getOSR();
  Serial.print("OSR (Over Sample Ratio): ");
  switch (currentOSR) {
    case QMC5883P_OSR_8:
      Serial.println("8");
      break;
    case QMC5883P_OSR_4:
      Serial.println("4");
      break;
    case QMC5883P_OSR_2:
      Serial.println("2");
      break;
    case QMC5883P_OSR_1:
      Serial.println("1");
      break;
    default:
      Serial.println("Unknown");
      break;
  }

  // Set DSR (Downsample Ratio) to 2
  qmc.setDSR(QMC5883P_DSR_2);
  qmc5883p_dsr_t currentDSR = qmc.getDSR();
  Serial.print("DSR (Downsample Ratio): ");
  switch (currentDSR) {
    case QMC5883P_DSR_1:
      Serial.println("1");
      break;
    case QMC5883P_DSR_2:
      Serial.println("2");
      break;
    case QMC5883P_DSR_4:
      Serial.println("4");
      break;
    case QMC5883P_DSR_8:
      Serial.println("8");
      break;
    default:
      Serial.println("Unknown");
      break;
  }

  // Set Range to 8G
  qmc.setRange(QMC5883P_RANGE_8G);
  qmc5883p_range_t currentRange = qmc.getRange();
  Serial.print("Range: ");
  switch (currentRange) {
    case QMC5883P_RANGE_30G:
      Serial.println("±30G");
      break;
    case QMC5883P_RANGE_12G:
      Serial.println("±12G");
      break;
    case QMC5883P_RANGE_8G:
      Serial.println("±8G");
      break;
    case QMC5883P_RANGE_2G:
      Serial.println("±2G");
      break;
    default:
      Serial.println("Unknown");
      break;
  }

  // Set SetReset mode to On
  qmc.setSetResetMode(QMC5883P_SETRESET_ON);
  qmc5883p_setreset_t currentSetReset = qmc.getSetResetMode();
  Serial.print("Set/Reset Mode: ");
  switch (currentSetReset) {
    case QMC5883P_SETRESET_ON:
      Serial.println("Set and Reset On");
      break;
    case QMC5883P_SETRESET_SETONLY:
      Serial.println("Set Only On");
      break;
    case QMC5883P_SETRESET_OFF:
      Serial.println("Set and Reset Off");
      break;
    default:
      Serial.println("Unknown");
      break;
  }
}

void loop() {
  static int range_counter = 0;
  static unsigned long last_range_change = 0;

  // Change range every 3 seconds
  if (millis() - last_range_change > 3000) {
    qmc5883p_range_t ranges[] = {QMC5883P_RANGE_2G, QMC5883P_RANGE_8G,
                                 QMC5883P_RANGE_12G, QMC5883P_RANGE_30G};
    const char* range_names[] = {"±2G", "±8G", "±12G", "±30G"};

    qmc.setRange(ranges[range_counter]);
    Serial.print("*** Changed to range: ");
    Serial.println(range_names[range_counter]);

    range_counter = (range_counter + 1) % 4;
    last_range_change = millis();
    delay(100); // Let range change settle
  }

  if (qmc.isDataReady()) {
    int16_t x, y, z;
    float gx, gy, gz;

    if (qmc.getRawMagnetic(&x, &y, &z)) {
      Serial.print("Raw - X: ");
      Serial.print(x);
      Serial.print(" Y: ");
      Serial.print(y);
      Serial.print(" Z: ");
      Serial.print(z);

      if (qmc.getGaussField(&gx, &gy, &gz)) {
        Serial.print(" | Gauss - X: ");
        Serial.print(gx, 3);
        Serial.print(" Y: ");
        Serial.print(gy, 3);
        Serial.print(" Z: ");
        Serial.println(gz, 3);
      } else {
        Serial.println(" | Failed to convert to Gauss");
      }

      if (qmc.isOverflow()) {
        Serial.println("WARNING: Data overflow detected!");
      }
    } else {
      Serial.println("Failed to read magnetic data");
    }
  }

  delay(200);
}