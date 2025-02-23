#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  Serial.println("MPU6050 connection successful");

  for (int i = 0; i < 10; i++) {
    Serial.println("0,2");  // Ensures the y-axis range remains 0 to 2 G
    delay(50);
  }
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Calculate acceleration amplitude
  float accel_x = ax / 16384.0;  // Conversion to g's (9.8 m/s^2)
  float accel_y = ay / 16384.0;
  float accel_z = az / 16384.0;
  float amplitude = sqrt(sq(accel_x) + sq(accel_y) + sq(accel_z));


  // Clip amplitude to fixed range (e.g., 0 to 2 G)
  if (amplitude > 2.0) {
    amplitude = 2.0;
  } else if (amplitude < 0.0) {
    amplitude = 0.0;
  }

  // Print time and amplitude
  unsigned long time = millis();
  Serial.print(time);
  Serial.print(",");
  Serial.println(amplitude);

  //   // Send constant dummy values to stabilize y-axis ticks
  // Serial.print(time);
  // Serial.print(",");
  // Serial.println(0.0);  // Low value
  // Serial.print(time);
  // Serial.print(",");
  // Serial.println(2.0);  // High value

  delay(150);  // Sampling rate adjustment (100 Hz)
}