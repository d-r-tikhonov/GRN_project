#include <Servo.h>              // подключаем библиотеку для работы с сервоприводом
#include "max6675.h"            // подключаем библиотеку для работы с термопарой
#include <PinChangeInterrupt.h> // подключаем библиотеку для прерываний

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// #define NDEBUG

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

volatile int current_count = 0;
volatile int counter       = 0;  // переменная-счётчик
int buttonPin = 2;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int thermoDO  = 3;  //он же SO
int thermoCS  = 4;
int thermoCLK = 5;  //он же SCK
int vccPin    = 6;  //пин для питания
int gndPin    = 7;  //пин для земли


MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

const float MAX_DIF_TEMPERATURE = 5.0;
float setup_temperature = 0.0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

const int pin_in_trans = A0;

const int SIZE_ARRAY  = 200;
const int MAX_DIF     = 2;
const int COMP        = 100;
const int NOISE_LEVEL = 0.8 * SIZE_ARRAY;

int jumped = 1;

const int NUM1_IN_NOISE = 0.2 * SIZE_ARRAY;

int cur_angle = 90;
int jump = 45;

int mode = 0;

const int NUM_CALIBR_ITERS = 5;
int ideal_angles[NUM_CALIBR_ITERS] = {};
int cal_iter_counter = 0;

const int FREQ_READING = 1;

int volt_data[SIZE_ARRAY] = {0};

int SetServo();
int PreCalibrate();
int GetNum1();

Servo myservo;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

const int MAX_BIN_SIZE  = 10;
const int FREQ_GENERATE = 10;

int max_input_voltage = 0;
int binary_num[MAX_BIN_SIZE] = {-1};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup() 
{
  Serial.begin(9600);
  Serial.println("Start setup");

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // подключили кнопку на D2 и GND
  pinMode(buttonPin, INPUT);
  // FALLING - при нажатии на кнопку будет сигнал 0, его и ловим
  attachInterrupt(digitalPinToInterrupt(buttonPin), btnIsr, FALLING);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  myservo.attach(10);  // привязываем сервопривод к аналоговому выходу 10

  pinMode(pin_in_trans, INPUT);

  mode = PreCalibrate(); // rising or falling, or can't calibrate

  if (!mode)
  {
    Serial.println("Can't calibrate: out of calibration range");
  }
  else if (mode == 1)
  {
    Serial.println("Precalibration success: Rising mode");
  }
  else if (mode == -1)
  {
    Serial.println("Precalibration success: Falling mode");
  }

  jump *= (-1) * mode;

  SetServo(cur_angle);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  #ifndef NDEBUG
  Serial.println("\nMAX6675 SETUP");
  #endif

  //активируем питание
  pinMode(vccPin, OUTPUT); 
  digitalWrite(vccPin, HIGH);

  //активируем землю
  pinMode(gndPin, OUTPUT); 
  digitalWrite(gndPin, LOW);

  //ждем стабилизации чипа MAX
  delay(500);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  Calibrate();
  setup_temperature = thermocouple.readCelsius(); 

  #ifndef NDEBUG
  Serial.print("\nCalibrate() temperature C = ");
  Serial.print(setup_temperature);
  #endif
}

void loop() 
{
  if (counter != current_count)
  {
    int random_number = generate_random_number();

    Serial.print("\nРандомно сгенерированное число от 0 до 1023: ");
    Serial.print(random_number);

    current_count = counter;

    delay(500);
  }

  //Сохраняем показания температуры
  float current_temperature = thermocouple.readCelsius(); 

  if (abs(current_temperature - setup_temperature) > MAX_DIF_TEMPERATURE)
  {
    Serial.print("\nCritical Calibrate() temperature C = ");
    Serial.print(current_temperature);

    mode = PreCalibrate();
    jumped = 1;
    Calibrate();

    setup_temperature = current_temperature;
  }

  delay(500);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int generate_random_number()
{
  int cmp_voltage = max_input_voltage/2;

  #ifndef NDEBUG
  Serial.print("\nПолученный массив volt_item: ");
  #endif

  for (int i = 0; i < MAX_BIN_SIZE; i++)
  {
    int volt_item = analogRead(pin_in_trans);
    
    #ifndef NDEBUG
    Serial.print(volt_item);
    Serial.print(" ");
    #endif

    if (volt_item > cmp_voltage)
    {
      binary_num[i] = 1;
    }
    else
    {
      binary_num[i] = 0;
    }

    delay(FREQ_GENERATE);
  }

  int random_number = 0;

  #ifndef NDEBUG
  Serial.print("\nПолученный массив 1 и 0: ");
  #endif

  for (int i = 0; i < MAX_BIN_SIZE; i++)
  {
    #ifndef NDEBUG
    Serial.print(binary_num[i]);
    #endif

    random_number += (round(pow(2,i)) * binary_num[i]);
  }
 
  return random_number;
}

void btnIsr() 
{
  counter++;  // + нажатие
}

void Calibrate()
{
  #ifndef NDEBUG
  Serial.println("\nStart Calibrate()!");
  #endif

  while (jumped != 0 && jump != 0 && mode)
  {
    jumped = 0;

    int num1 = GetNum1();

    if ((SIZE_ARRAY / 2 - num1) > MAX_DIF) 
    {
      SetServo(cur_angle + jump);
      jumped = 1;
    }

    if ((num1 - SIZE_ARRAY / 2) > MAX_DIF)
    {
      SetServo(cur_angle - jump);
      jumped = 1;
    }

    if (jumped == 0)
    {
      Serial.println("Iteration successful");
      Serial.print("Found number of 1s = ");
      Serial.println(num1, DEC);
      Serial.print("Ideal angle is ");
      Serial.println(cur_angle, DEC);

      if (cal_iter_counter < NUM_CALIBR_ITERS)
      {
        ideal_angles[cal_iter_counter] = cur_angle;
        cal_iter_counter++;

        jump = 10;
        jumped = 1;

        if (cal_iter_counter < NUM_CALIBR_ITERS)
        {
          SetServo(cur_angle + jump);
        }
        else
        {
          Serial.println("Calibration finished successfully");
          Serial.print("Ideal angles are: ");
          for (int i = 0; i < NUM_CALIBR_ITERS; i++)
          {
            Serial.print(ideal_angles[i], DEC);
            Serial.print(" ");
          }
          Serial.print("\nFinal ideal angle is ");
          Serial.println(IdealAngle(), DEC);

          SetServo(IdealAngle());
          jumped = 0;
          int num1 = GetNum1();

          Serial.print("Found number of 1s = ");
          Serial.println(num1, DEC);
          Serial.print("for final ideal angle ");
          Serial.println(cur_angle, DEC);
        }
      }
    }

    jump /= 2;

    if (jump == 0)
    {
      Serial.print("Iteration failed: can't calibrate with such accuracy: ");
      Serial.println(MAX_DIF, DEC);
      Serial.println("Starting new binsearch for 10");

      jump = 10;
    }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int PreCalibrate()
{
  int cur_num1 = 0;

  for (int i = 0; i < 181; i += 45)
  {
    SetServo(i);  
    cur_num1 = GetNum1();

    if (cur_num1 > NUM1_IN_NOISE)
    {
      cur_angle = i;
      return -1;
    }
  }


  return 0;
}

int GetNum1()
{
  double mid  = 0;
  int max_val = COMP;

  for(int i = 0; i < SIZE_ARRAY; i++)
  {
    int volt_item = analogRead(pin_in_trans);

    volt_data[i] = volt_item;
    mid += volt_item;

    if (volt_item > max_val)
    {
      max_val = volt_item;
    }

    Serial.print(volt_item, DEC);
    Serial.print(" ");
    delay(FREQ_READING);
  }

  mid /= SIZE_ARRAY;

  int num1 = 0;
  int cur_comp = max_val / 2;

  for(int i = 0; i < SIZE_ARRAY; i++)
  {
    if (volt_data[i] > cur_comp)
    {
      num1++;
    }
  }

  #ifndef NDEBUG
  Serial.print("\nCurrent comparator: ");
  Serial.println(cur_comp, DEC);

  Serial.print("Number of 1s: ");
  Serial.println(num1, DEC);
  #endif

  max_input_voltage = max_val; // сохраняем значение для последующей генерации случайных чисел

  return num1;
}

int SetServo(int angle)

{
  if (angle <= 10)
  {
    //Serial.print("Invalid angle! < 10");
    angle = 10;
  }

  if (angle > 180)
  {
    // Serial.print("Invalid angle! > 180");
    angle = 180;
  }

  #ifndef NDEBUG
  Serial.print("\nGoing to ");
  Serial.println(angle, DEC);
  #endif

  myservo.write(angle);
  delay(500);

  cur_angle = angle;

  return angle;
}

int IdealAngle()
{
  float sum = 0;
  for (int i = 0; i < NUM_CALIBR_ITERS; i++)
  {
    sum += (float)ideal_angles[i];
  }

  return (int)(sum / ((float)NUM_CALIBR_ITERS));
}
