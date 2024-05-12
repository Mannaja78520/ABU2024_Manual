#ifndef RobotAnalysis_h
#define RobotAnalysis_h

#include <DefinePin.h>

#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68

MPU9250_WE IMU = MPU9250_WE(MPU9250_ADDR);
float IMUyaw = 0, IMUpitch = 0, IMUroll = 0;

class RobotAnalysis{
    unsigned long LastTime = 0;
    public:
        void init(){
            Wire.begin();

            if(!IMU.init()){
                Serial.println("MPU9250 does not respond");
            }
            else{
                Serial.println("MPU9250 is connected");
            }
            // xyzFloat gOffs = {947.7, 1148.0, -155.0};
            // IMU.setGyrOffsets(gOffs);
            // IMU.enableGyrDLPF();
            // IMU.setGyrDLPF(MPU9250_DLPF_6);
            // IMU.setSampleRateDivider(5);
            // IMU.setGyrRange(MPU9250_GYRO_RANGE_250);

            Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
            delay(1000);
            IMU.autoOffsets();
            Serial.println("Done!");
            IMU.enableGyrDLPF();
            IMU.setAccDLPF(MPU9250_DLPF_6);
            IMU.setSampleRateDivider(99);
            IMU.setGyrRange(MPU9250_GYRO_RANGE_250);
            IMU.setAccRange(MPU9250_ACC_RANGE_2G);
            IMU.enableAccDLPF(true);

        }
        void loop(){
            unsigned long CurrentTime = millis();
            float Dt = CurrentTime / 1000.0 - LastTime / 1000.0;
            LastTime = CurrentTime;
            // xyzFloat corrGyrRaw = IMU.getCorrectedGyrRawValues();
            xyzFloat Angle = IMU.getGyrValues();
            float AngleZ = abs(Angle.z) < 0.035 ? 0 : Angle.z;
            // Serial.println(Dt, 6);
            // Serial.println(corrGyrRaw.z);
            // Serial.println(corrGyrRaw.z * Dt);
            // Serial.println();
            // // Serial.println("Gyroscope data in degrees/s: ");
            // Serial.println(gyr.x);
            // // Serial.print("   ");
            // // Serial.print(gyr.y);
            // // Serial.print("   ");
            // // Serial.println(gyr.z);
            // IMUroll += gyr.x * Dt;
            // IMUpitch += gyr.y * Dt;
            IMUyaw += AngleZ * Dt;
            // // Serial.println("Gyroscope data in degree: ");
            // // Serial.print(yaw);
            // // Serial.print("   ");
            // // Serial.print(pitch);
            // // Serial.print("   ");
            // // Serial.println(roll);

            // Serial.println(IMUyaw, 6);


            // // Serial.println("********************************************");
        }
};

#endif