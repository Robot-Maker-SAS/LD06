#include "ld06.h"


#define LIDAR_SERIAL Serial1

void LD06::init(const int pin) {
  LIDAR_SERIAL.begin(230400);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void LD06::calc_lidar_data(uint8_t* values) {
  Speed = float(values[3] << 8 | values[2]) / 100;
  FSA = float(values[5] << 8 | values[4]) / 100;
  LSA = float(values[TOTAL_DATA_BYTE - 4] << 8 | values[TOTAL_DATA_BYTE - 5]) / 100;
  time_stamp = int(values[TOTAL_DATA_BYTE - 2] << 8 | values[TOTAL_DATA_BYTE - 3]);

  angle_step = ((LSA - FSA > 0) ? (LSA - FSA) / (NBMEASURES - 1) : (LSA + (360 - FSA)) / (NBMEASURES - 1));

  if (angle_step > 20)
    return;

  for (int i = 0; i < NBMEASURES; i++) {
    float raw_deg = FSA + i * angle_step;
    angles[i] = (raw_deg <= 360 ? raw_deg : raw_deg - 360);
    confidences[i] = values[8 + i * 3];
    distances[i] = int(values[8 + i * 3 - 1] << 8 | values[8 + i * 3 - 2]);
  }
}

// Read lidar packet data
int LD06::read_lidar_data() {
  return read_lidar_data_with_crc();
}

// Read lidar packet data and check CRC,
// return -1 if new packet received but crc error,
// return 1 if new packet received and crc ok
// else return 0
int LD06::read_lidar_data_with_crc() {
  int result = 0;
  static uint8_t crc = 0;
  static uint8_t n = 0;
  static uint8_t lidarData[TOTAL_DATA_BYTE];
  while (LIDAR_SERIAL.available()) {
    uint8_t current = LIDAR_SERIAL.read();
    if (n > 1 || (n == 0 && current == HEADER) || (n == 1 && current == VER_SIZE)) {
      lidarData[n] = current;
      if (n < TOTAL_DATA_BYTE - 1) {
        crc = CrcTable[crc ^ current];
        n++;
      } else {
        if (crc == current) {
          calc_lidar_data(lidarData);
          result = 1;
        } else {
          result = -1;
        }
        n = 0;
        crc = 0;
      }
    } else {
      n = 0;
      crc = 0;
    }
  }
  return result;
}

// Read lidar packet data without checking CRC,
// return 1 if new packet received
// else return 0
bool LD06::read_lidar_data_without_crc() {
  static uint8_t n = 0;
  static uint8_t lidarData[TOTAL_DATA_BYTE];
  while (LIDAR_SERIAL.available()) {
    uint8_t current = LIDAR_SERIAL.read();
    if (n > 1 || (n == 0 && current == HEADER) || (n == 1 && current == VER_SIZE)) {
      lidarData[n] = current;
      if (n == TOTAL_DATA_BYTE - 1) {
        calc_lidar_data(lidarData);
        n = 0;
        return true;
      }
    } else
      n = 0;
  }
  return false;
}

// Read lidar packets and return true when a new full 360° scan is available
bool LD06::readFullScan() {
  static DataPoint currentScan[MAX_PTS_SCAN];
	static uint16_t currentScanIndex = 0;
  static bool isInit = false;
  static float lastAngle = 0;
  bool newScan = false;
  DataPoint data;

  if (read_lidar_data()) {
    for (int i = 0; i < NBMEASURES; i++) {
      if (angles[i] < lastAngle) {
        if (!isInit) {
          isInit = true;
        } else {
          newScan = true;
          scanIndex = 0;
          for (uint16_t j = 0; j < currentScanIndex; j++) {
            scan[scanIndex] = currentScan[j];
						scanIndex++;
          }
          currentScanIndex = 0;
        }
      }
      lastAngle = angles[i];
      if( currentScanIndex < MAX_PTS_SCAN) {
        data.angle = angles[i];
        data.distance = distances[i];
        data.x = data.distance * cos(data.angle * PI / 180);
        data.y = -data.distance * sin(data.angle * PI / 180);
        data.intensity = confidences[i];
        currentScan[currentScanIndex] = data;
			  currentScanIndex++;
      }
    }
  }
  return newScan;
}

// Print full scan using csv format
void LD06::csvPrintScan() {
  Serial.println("Angle(°),Distance(mm),x(mm),y(mm)");
  for (uint16_t i = 0; i < scanIndex; i++) {
    Serial.println(String() + scan[i].angle + "," + scan[i].distance + "," + scan[i].x + "," + scan[i].y);
  }
  Serial.println("");
}

// Print full scan using teleplot format (check :https://teleplot.fr/)
void LD06::teleplotPrintScan() {
  Serial.print(">lidar:");
  for (uint16_t i = 0; i < scanIndex; i++) {
    Serial.print(String() + scan[i].x + ":" + scan[i].y + ";");
  }
  Serial.println("|xy");
}
