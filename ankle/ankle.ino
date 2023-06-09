#include <LSM6DS3.h>
#include <Servo.h>

Servo FRONT_SERVO;
Servo REAR_SERVO;
LSM6DS3 IMU(I2C_MODE, 0x6B);

#define PIN_FRONT_SERVO 9
#define PIN_REAR_SERVO 8

void InitServos() {
    FRONT_SERVO.attach(PIN_FRONT_SERVO);
    REAR_SERVO.attach(PIN_REAR_SERVO);
}

void InitIMU() {
    // put your setup code here, to run once:
    Serial.begin(2000000);
    while (!Serial);
    //Call .begin() to configure the IMUs
    if (IMU.begin() != 0) {
      Serial.println("Device error");
    } else {
      Serial.println("aX,aY,aZ,gX,gY,gZ");
    }
}
void setup() {
    InitServos();
    InitIMU();
}

float GetFrontAccel() {
    float total = 0;
    size_t n_dots = 16;

    for (size_t i = 0; i < n_dots; i++) {
      // print the data in CSV format
      float val = 0;
      val = IMU.readFloatGyroY();
      total += val;
    }
    Serial.println(total / n_dots);
    return total / n_dots;
}


// should return a value from (-100:100)
float NormalizeAccel(float accel) {
  auto tmp = map(accel, -70, 70, -100, 100);
  return constrain(tmp, -100, 100);
}

constexpr auto rear_start_deg = 110;
constexpr auto rear_end_deg = 50;
constexpr auto front_start_deg = 90;
constexpr auto front_end_deg = 160;

float LoadToDegrees(float load) {
    if (load > 0) {
        return map(load, 0, 100, front_start_deg, front_end_deg);
    }
    return map(load, -100,  0, rear_end_deg, rear_start_deg);
}

int STEP = 1;

void DecayRear() {
    auto cur = REAR_SERVO.read();
    if (cur + STEP > rear_start_deg) {
      REAR_SERVO.write(rear_start_deg);
    } else {
      REAR_SERVO.write(cur + STEP);
    }
}

void DecayFront() {
    auto cur = FRONT_SERVO.read();
    if (cur - STEP < front_start_deg) {
      FRONT_SERVO.write(front_start_deg);
    } else {
      FRONT_SERVO.write(cur - STEP);
    }
}
void AdjustServos(float load) {
    auto degrees = LoadToDegrees(load);
    if (load > 0) {
        DecayRear();
        FRONT_SERVO.write(degrees);
    } else {
        REAR_SERVO.write(degrees);
        DecayFront();
    }
}


void loop() {
  auto accel = GetFrontAccel();
  auto load = NormalizeAccel(accel);
  AdjustServos(load);
}
