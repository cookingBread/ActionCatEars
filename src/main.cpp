#include <Arduino.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include <MPU6050.h>

#define PIN_LED  2
#define I2C_SDA 17
#define I2C_SCL 16
#define I2C_ADDR_GYR 0x68  // gyroscope陀螺仪

BluetoothSerial SerialBT;         //定义一个蓝牙串口对象名称
MPU6050 gyroscope(I2C_ADDR_GYR);
struct gyr
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    String toString(){
        return "ax:"+String(ax)+" ay"+String(ay)+" az"+String(az)+" gx"+String(gx)+" gy"+String(gy)+" gz"+String(gz);
    }
} gyr;



extern void log(String msg);
extern void init_i2c_gyroscope();
extern void get_gyroscope();

void setup() {
    // 配置串口
    Serial.begin(115200);
    log("Serial speed 115200");

    // 配置蓝牙
    // SerialBT.begin("ESP32_Blue");

    delay(3000); // todo:
    // 配置 I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    init_i2c_gyroscope();
}

void loop() {
    // SerialBT.println("Hello from ESP32");
    Serial.println(gyr.toString());
}

void init_i2c_gyroscope(){
    gyroscope.initialize();
    log(gyroscope.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void recive_gyroscope(){
    gyroscope.getMotion6(&gyr.ax, &gyr.ay, &gyr.az, &gyr.gx, &gyr.gy, &gyr.gz);
}

void log(String msg){
    Serial.println(msg);
}


