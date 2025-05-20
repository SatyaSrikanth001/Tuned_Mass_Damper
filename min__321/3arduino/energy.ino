#include <Wire.h>
#include <MPU6050.h>

MPU6050 accelSensor;

unsigned long prevTime = 0;
float dt; // Time difference
float velocity = 0;
float displacement = 0;

// Physical parameters
const float m = 0.225; // Mass of the damper (kg)
const float k = 9.588; // Spring constant (N/m)
const float c = 2 * m * 5.84336233568; // Damping coefficient

void setup() {
  Serial.begin(115200);
  Wire.begin();
  accelSensor.initialize();

  if (!accelSensor.testConnection()) {
    Serial.println("Sensor connection failed!");
    while (1);
  }

  Serial.println("Time (s), Acceleration (m/s^2), Velocity (m/s), Displacement (m), Kinetic Energy (J), Potential Energy (J), Mechanical Energy (J), Dissipated Energy (J)");

  prevTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0; // Convert ms to seconds
  prevTime = currentTime;

  // Read acceleration data along the X-axis
  int16_t rawAccel = accelSensor.getAccelerationX();
  float acceleration = rawAccel / 16384.0 * 9.81; // Convert raw data to m/s^2

  // Numerical integration for velocity and displacement
  velocity += acceleration * dt;
  displacement += velocity * dt;

  // Energy calculations
  float kineticEnergy = 0.5 * m * velocity * velocity;
  float potentialEnergy = 0.5 * k * displacement * displacement;
  float mechanicalEnergy = kineticEnergy + potentialEnergy;
  float dissipatedEnergy = 0.5 * c * velocity * velocity;

  // Send data to Serial Monitor for real-time plotting
  Serial.print(currentTime / 1000.0, 3); Serial.print(",");
  Serial.print(acceleration, 3); Serial.print(",");
  Serial.print(velocity, 3); Serial.print(",");
  Serial.print(displacement, 3); Serial.print(",");
  Serial.print(kineticEnergy, 5); Serial.print(",");
  Serial.print(potentialEnergy, 5); Serial.print(",");
  Serial.print(mechanicalEnergy, 5); Serial.print(",");
  Serial.println(dissipatedEnergy, 5);

  delay(10); // Small delay for better Serial communication
}
