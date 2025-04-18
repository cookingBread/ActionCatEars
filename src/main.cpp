#include <Arduino.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>


// 引脚定义
#define I2C_SDA 17
#define I2C_SCL 16
#define I2C_ADDR_GYR 0x68  // gyroscope陀螺仪
#define SERVO_LEFT_DOWN 0  //todo:
#define SERVO_LEFT_UP 0
#define SERVO_RIGHT_DOWN 0
#define SERVO_RIGHT_UP 0

BluetoothSerial SerialBT;         //定义一个蓝牙串口对象名称

// ******声明******
// 日志函数
extern void log(String msg);
// 陀螺仪
extern void init_i2c_gyroscope();
extern void recive_gyroscope();
MPU6050 gyroscope(I2C_ADDR_GYR);
struct gyr
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    String toString(){
        return "ax:"+String(ax)+" ay:"+String(ay)+" az:"+String(az)+" gx:"+String(gx)+" gy:"+String(gy)+" gz:"+String(gz);
    }
} gyr;

// 舵机控制
extern Servo servo_ld,servo_lu,servo_rd,servo_ru;
extern void init_servos();
extern void servos_write();

// 耳朵控制
extern void ears_down(){}
extern void ears_up(){}
extern void ears_to_forward(){}
extern void ears_to_left(){}
extern void ears_to_right(){}


void setup() {
    // 配置串口
    Serial.begin(115200);
    delay(3000); // todo:给时间打开串口
    log("Serial speed 115200");

    // 配置蓝牙
    // SerialBT.begin("ESP32_Blue");


    // 配置 I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    init_i2c_gyroscope();
}

void loop() {
    // SerialBT.println("Hello from ESP32");
    log(gyr.toString());
    recive_gyroscope();
    delay(200);
}

// ******陀螺仪******
// 初始化陀螺仪i2c连接
void init_i2c_gyroscope(){
    gyroscope.initialize();
    log(gyroscope.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void recive_gyroscope(){
    gyroscope.getMotion6(&gyr.ax, &gyr.ay, &gyr.az, &gyr.gx, &gyr.gy, &gyr.gz);
}

// ***通用日志口***
void log(String msg){
    Serial.println(msg);
}

// ***舵机控制***
void init_servos(){
    servo_ld.attach(SERVO_LEFT_DOWN);
    servo_lu.attach(SERVO_LEFT_UP);
    servo_rd.attach(SERVO_RIGHT_DOWN);
    servo_ru.attach(SERVO_RIGHT_UP);
    // 设定初始角度，目前没考虑好是90还是极值
    servo_ld.write(90);
    servo_lu.write(90);
    servo_rd.write(90);
    servo_ru.write(90);
}

/**
 * @description: 设置舵机角度
 * 需要根据实际安装情况调整值映射（x或180-x）
 * 默认将所有角度设置为，下0、上180、左0、右180
 * 若填入-1则保持不变
 * @param {int} lu  左上舵机
 * @param {int} ld  左下舵机
 * @param {int} ru  右上舵机
 * @param {int} rd  右下舵机
 * @return {*}
 */
void servos_write(int lu, int ld, int ru, int rd){
    servo_lu.write(0);
    servo_ld.write(0);
    servo_ru.write(0);
    servo_rd.write(0);
}

// ***耳朵控制***
void ears_down(){

}
void ears_up(){}
void ears_to_forward(){}
void ears_to_left(){}
void ears_to_right(){}