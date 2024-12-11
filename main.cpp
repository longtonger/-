#include <Arduino.h>

// 定义传感器数量
#define SENSOR_COUNT 16  // 16个红外传感器
const int controlPins[] = {26, 27, 14, 12}; // 控制传感器地址的4根线


#define SENSOR_READ_PIN 25  // 模拟读取引脚

// 定义电机控制引脚
#define ENA   33 // 左电机 PWM 控制
#define ENB   32 // 右电机 PWM 控制
#define IN1   5  // 左电机方向控制
#define IN2   18  // 左电机方向控制
#define IN3   16  // 右电机方向控制
#define IN4   17  // 右电机方向控制
#define Bin   21

// PID 参数
float kp = 60;    // 比例系数
float ki = 0.2;     // 积分系数
float kd = 500;   // 微分系数
float error = 0, lastError = 0, integral = 0, derivative = 0;
float controlOutput = 0;

int flag = 0;
int normalSpeed = 190;

// 初始化设置
void setup()
{
    // 初始化传感器控制线
    for (int i = 0; i < 4; i++) 
    {
        pinMode(controlPins[i], OUTPUT);
        digitalWrite(controlPins[i], LOW); // 初始为低电平
    }
    pinMode(SENSOR_READ_PIN, INPUT); // 模拟读取引脚

    // 初始化电机控制引脚
    pinMode(ENA, OUTPUT);  
    pinMode(ENB, OUTPUT);  
    pinMode(IN1, OUTPUT);  
    pinMode(IN2, OUTPUT);  
    pinMode(IN3, OUTPUT);  
    pinMode(IN4, OUTPUT); 

    pinMode(Bin, OUTPUT);

     // 设置 PWM 频率和分辨率
    ledcSetup(0, 1000, 8); // PWM 通道 0, 频率 1kHz，分辨率 8 位
    ledcSetup(1, 1000, 8); // PWM 通道 1, 频率 1kHz，分辨率 8 位

    // 将 PWM 引脚与通道连接
    ledcAttachPin(ENA, 0); // 连接 ENA 引脚到 PWM 通道 0
    ledcAttachPin(ENB, 1); // 连接 ENB 引脚到 PWM 通道 1
}


// 停止电机
void stopMotors() 
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(1, 150);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(0, 150);
    delay(100);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(0, 0);  // 停止左电机 PWM
    ledcWrite(1, 0);  // 停止右电机 PWM
}

// 移动函数
void move(float leftSpeed, float rightSpeed)
{
    if(leftSpeed <= 0)
    {
        leftSpeed = -leftSpeed;
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        ledcWrite(1, leftSpeed);
    } 
    else 
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(1, leftSpeed);  // 设置左电机速度
    }
    if(rightSpeed <= 0) 
    {
        rightSpeed = -rightSpeed;
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(0, rightSpeed);
    } 
    else 
    {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        ledcWrite(0, rightSpeed);  // 设置左电机速度
    }
}

// 设置传感器地址
void setSensorAddress(int address) 
{
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(controlPins[i], (address >> i) & 0x01); // 设置每一位
    }
}

// 读取传感器值
int readSensor(int address)
{
    setSensorAddress(address); // 设置传感器地址
    int sensorValue = analogRead(SENSOR_READ_PIN); //读取模拟值

    return sensorValue;
}

int minflag=0;
int flag1=-1000;

// 计算赛道中心位置
float calculateCenterPosition()
{
    float weightedSum = 0.0;
    float weightSum = 0.0;
    float Sumforstop=0;

    for (int i = SENSOR_COUNT-1; i >= 0; i--) 
    {
        int sensorValue = readSensor(i); // 读取传感器值
        Sumforstop+=sensorValue;
        sensorValue=(sensorValue < 3200) ? 1 : 0;
        delay(1);
        float position = (i - (SENSOR_COUNT - 1) / 2.0); // 计算传感器相对位置
        weightedSum += sensorValue * position; // 权重累加
        weightSum += sensorValue; // 总值累加
    }

    Sumforstop/=16;
    
    if (weightSum == 0 && weightedSum == 0)
    {
        return -20000; // 没有检测到黑线
    }

    if (Sumforstop <= 3200 && millis()-flag1 >= 600)
    {
        if(minflag == 0)
        {
            flag++;
            if(flag == 3)
            {
                normalSpeed = 115;
                minflag = millis();
            }
            else if(flag == 6)
            {
                normalSpeed = 115;
                minflag = millis();
            }
        }
        else if(millis()-minflag >= 5000 && minflag!=0 )
        {
            flag++;
            if(flag == 4)
            {
                normalSpeed = 190;
                minflag = 0;
            }
            else if(flag == 7)
            {
                stopMotors(); 
                while(1);
            }
        }
        flag1 = millis();
        return -10000; // 返回一个特殊值表示检测到黑线
    }
    


    return weightedSum / weightSum; // 返回加权位置
}

// PID 计算函数
float pidControl(float setpoint, float actualPosition)
{
    error = setpoint - actualPosition;
    integral += error;
    derivative = error - lastError;
    controlOutput = kp * error + ki * integral + kd * derivative;
    lastError = error;
    return controlOutput;
}

int lastLeftSpeed=0,lastRightSpeed=0;
float centerPosition = 0;

void loop()
{
    centerPosition = calculateCenterPosition(); // 获取拟合中心位置

    if (centerPosition >= -20001 && centerPosition <= -19999) // 未检测到黑线
    {
        // 没有检测到黑线时，继续使用上一次的电机速度
        move(lastLeftSpeed, lastRightSpeed);
        return;
    }
    else if (centerPosition >= -10001 && centerPosition <= -9999) // 检测到黑线
    {
        return;
    }
    else // 检测到黑线
    {
        // 使用 PID 控制调整电机速度
        controlOutput = pidControl(0, centerPosition);

        float leftSpeed = normalSpeed + controlOutput;
        float rightSpeed = normalSpeed - controlOutput;

        // 约束速度范围
        leftSpeed = constrain(leftSpeed, -195, 200);
        rightSpeed = constrain(rightSpeed, -195, 200);

        // 驱动电机并更新状态变量
        move(leftSpeed, rightSpeed);

        lastLeftSpeed = leftSpeed;
        lastRightSpeed = rightSpeed;
    }
}



