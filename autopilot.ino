
//используем библиотеку для работы с сервоприводом
#include <Servo.h>
// библиотека для работы I²C
#include <Wire.h>
// библиотека для работы с модулями IMU
#include <TroykaIMU.h>

#define INPUT_PIN 5
#define SERVO_PIN_1 2
#define SERVO_PIN_2 3
// множитель фильтра
#define BETA 0.22

Servo servo1;
Servo servo2;

// создаём объект для фильтра Madgwick
Madgwick filter;
// создаём объект для работы с акселерометром
Accelerometer accel;
// создаём объект для работы с гироскопом
Gyroscope gyro;
// создаём объект для работы с компасом
Compass compass;

int stick;
int correction;

// переменные для данных с гироскопа, акселерометра и компаса
float gx, gy, gz, ax, ay, az, mx, my, mz;
 
// получаемые углы ориентации (Эйлера)
float yaw, pitch, roll;
 
// переменная для хранения частоты выборок фильтра
float fps = 100;
 
// калибровочные значения компаса
// полученные в калибровочной матрице из примера «compassCalibrateMatrixx»
const double compassCalibrationBias[3] = {
  524.21,
  3352.214,
  -1402.236
};
 
const double compassCalibrationMatrix[3][3] = {
  {1.757, 0.04, -0.028},
  {0.008, 1.767, -0.016},
  {-0.018, 0.077, 1.782}
};

void setup() {

  pinMode(INPUT_PIN, INPUT); // Set our input pins as such

  servo1.attach(SERVO_PIN_1);
  servo2.attach(SERVO_PIN_2);

  Serial.begin(9600); // Pour a bowl of Serial

  // инициализация акселерометра
  accel.begin();
  // инициализация гироскопа
  gyro.begin();
  // инициализация компаса
  compass.begin();
 
  // калибровка компаса
  compass.calibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);
}

void loop() {

// запоминаем текущее время
  unsigned long startMillis = millis();
 
  // считываем данные с акселерометра в единицах G
  accel.readGXYZ(&ax, &ay, &az);
  // считываем данные с гироскопа в радианах в секунду
  gyro.readRadPerSecXYZ(&gx, &gy, &gz);
  // считываем данные с компаса в Гауссах
  compass.readCalibrateGaussXYZ(&mx, &my, &mz);
 
  // устанавливаем коэффициенты фильтра
  filter.setKoeff(fps, BETA);
  // обновляем входные данные в фильтр
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
 
  // получение углов yaw, pitch и roll из фильтра
  yaw =  filter.getYawDeg();
  pitch = filter.getPitchDeg();
  roll = filter.getRollDeg();
 
  // выводим полученные углы в serial-порт
//  Serial.print("yaw: ");
//  Serial.print(yaw);
//  Serial.print("\t\t");
//  Serial.print("pitch: ");
//  Serial.print(pitch);
//  Serial.print("\t\t");
//  Serial.print("roll: ");
//  Serial.print(roll);


  stick = map(pulseIn(INPUT_PIN, HIGH, 25000), 1070, 1880, 0, 180);

  correction = stick - (roll * 2);

//  Serial.print("\t\t");
//  Serial.print("stick: ");
//  Serial.print(stick);
//
//  Serial.print("\t\t");
//  Serial.print("corr: ");
//  Serial.println(correction);

  servo1.write(correction);
  servo2.write(correction);
  //delay(300); // I put this here just to make the terminal 
              // window happier

              // вычисляем затраченное время на обработку данных
  unsigned long deltaMillis = millis() - startMillis;
  // вычисляем частоту обработки фильтра
  fps = 1000 / deltaMillis;
}
