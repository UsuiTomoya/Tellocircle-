#include <SoftwareSerial.h>
#include "TFMini.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

TFMini tfmini;
Adafruit_MPU6050 mpu;

SoftwareSerial SerialTFMini(10, 11);          //The only value that matters here is the first one, 2, Rx

const int NUM_READINGS = 10;
const int NUM_READINGS_2 = 10;
float distances[NUM_READINGS];
float pitches[NUM_READINGS];
int readings_count = 0;

// 10回分のdistanceの平均値を計算する
int calculateAverageDistance() {
  float total = 0.0;
  for (int i = 0; i < NUM_READINGS; i++) {
    total += static_cast<float>(distances[i]);
  }
  return total / NUM_READINGS;
}

// 10回分のpitchの平均値を計算する
float calculateAveragePitch() {
  float total = 0.0;
  for (int i = 0; i < NUM_READINGS; i++) {
    total += static_cast<float>(pitches[i]);
  }
  return total / NUM_READINGS;
}
 
void getTFminiData(int* distance, int* strength)
{
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (SerialTFMini.available())
  {
    rx[i] = SerialTFMini.read();
    if (rx[0] != 0x59)
    {
      i = 0;
    }
    else if (i == 1 && rx[1] != 0x59)
    {
      i = 0;
    }
    else if (i == 8)
    {
      for (j = 0; j < 8; j++)
      {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256))
      {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    }
    else
    {
      i++;
    }
  }
}
 
 
void setup()
{
  Serial.begin(115200);       //Initialize hardware serial port (serial debug port)
  while (!Serial);            // wait for serial port to connect. Needed for native USB port only
  Serial.println ("Initializing...");
  SerialTFMini.begin(TFMINI_BAUDRATE);    //Initialize the data rate for the SoftwareSerial port
  tfmini.begin(&SerialTFMini);      //Initialize the TF Mini sensor

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println("");

  delay(1000);
}
 
void loop(){
  int distance = 0;
  int strength = 0;
  
  getTFminiData(&distance, &strength);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //まずはdistancesの平均から
  while (!distance) {
    getTFminiData(&distance, &strength);
  }
  
  distances[readings_count] = distance; // distanceを配列に格納する
  
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 57.3;
  pitches[readings_count] = pitch;
  readings_count++; // 読み取った数をカウントする
  
  if (readings_count == NUM_READINGS) { // 10回分のデータを読み取った場合
    int average_distance = calculateAverageDistance(); // 平均値を計算する
    float average_pitch = calculateAveragePitch();
    //Serial.print("Average distance: ");
    Serial.print(average_distance);
    //Serial.print(", Pitches_average: ");
    Serial.print(",");
    Serial.print(average_pitch);
    Serial.println();
    readings_count = 0; // カウントをリセットする
  }

  delay(100);

}
