#include <Arduino.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <RtcDs3231.h>
//#include "GravityTDS.h"

#include <EEPROM.h>

RtcDS3231<TwoWire> Rtc(Wire); //Часы реального времени
// RS, E, DB4, DB5, DB6, DB7
LiquidCrystal lcd(14, 15, 16, 17, 18, 19); //дисплей 16*2 PINS

//////////////////---------------KEYPAD 4X4---------------////////////////////////////
const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = 
{
{'1','2','3','A'},
{'4','5','6','B'},
{'7','8','9','C'},
{'*','0','#','.'}
};

byte rowPins[ROWS] = {20,21,22,23}; //4 столбца PINS
byte colPins[COLS] = {24,25,26,27}; //4 строки PINS
Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

float getIntNumber(); //Ввод значения int с клавиатуры
float getFloatNumber();//Ввод значения float с клавиатуры
//////////////////////////////////////////////////////////////////////////////////////

//PH control////////////////////////////////////////////////////////////
#define PHSensorPin A6          //pH meter Analog output to Arduino Analog Input PIN
#define Offset 0.00            //deviation compensate
#define samplingInterval 20
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;    
/////////////////////////////////////////////////////////////////////////
#define phMinusPin 31 //pH DOWN Relay PIN
#define phPlusPin 32 //pH UP Relay PIN
///////////////////////////////////////////////////////////////////////////
////--------DHT--------------------/////////////////////////////////////////////////

#define DHTPIN 28 //Датчик температуры и влажности
#define DHTTYPE DHT11
DHT dht (DHTPIN,DHTTYPE);

///////////----------Реле для управления микроклиматом-------------//////////////

#define HumidityUPPin 37 // Пин реле с увлажнителем воздуха 1
#define TempDOWNPin 38 // Пин реле с обогревателем 2
#define TempUPPin 39 // Пин реле с вентилятором 3
////////////////////////////////////////////////////////////////////////////////////
#define LightRelay 40 // Реле лампы

/////////////////////////////////////////////

struct phSettings //структура для настроек pH
{
    float tankVolume;
    float requiredPh;
    float minPh;
    float maxPh;
};
phSettings phSetValue{20,6,5.5,6}; //Переменная для Хранения настроек контроля pH
phSettings phSetProf1{20,6,5.5,6.5}; //Параметры для выращивания помидор

float phVal; //Переменная для getPhValue

unsigned long printTime;
unsigned long last_time; //Переменная для счетчика цикла controlPh
///////////////////////////////////////////////////////////////////////////////
struct temperatureSettings // Границы температуры воздуха
{
    float TemperatureMin;
    float TemperatureMax;
};
struct humiditySettings // Границы влажности
{
    float humidityMin;
    float humidityMax;  
};
struct timerSettings // Время
{
    uint8_t dayHourSet;
    uint8_t dayMinuteSet;
    uint8_t nightHourSet;
    uint8_t nightMinuteSet;
};

timerSettings temperatureTime {8,30,21,30}; //Время дневной и ночной температуры

float temperatureVal, humidityVal; //текущая температура и влажность
humiditySettings humiditySetValue {50,70};//влажность
temperatureSettings dayTemperature {23,25}; //дневная температура
temperatureSettings nightTemperature {16,18};//ночная температура
temperatureSettings temperature; //Переключение дневной и ночной температуры

timerSettings lightTime {8,30,21,30}; //Время включения и выключения лампы

//////////////////----------ФУНКЦИИ---------////////////////////////////////////////////
float getPhValue(); //Получить значение pH
double avergeArrayPh(int*,int);//Среднее значение pH 
//void printPhValLCD(float);//Вывести значение pH на дисплей 16*2

phSettings setPh(); //Задать настройки pH вручную
void controlPh(phSettings,float); //Контроллирование заданного уровня pH с помощью насосов


temperatureSettings SetDayT(); //Задать дневную температуру
temperatureSettings SetNightT(); //Задать ночную температуру
humiditySettings SetHumidity(); //Задать нижнюю и верхнюю границы влажности
timerSettings SetStartFinishTime();//Задать время дневной и ночной температуры

void ControlHumidity(humiditySettings,float); // Контроль уровня влажности (настройки влажности, значение влажности)
void ControlTemperature(temperatureSettings,float);// Контроль дневной и ночной температуры воздуха (настройки температуры, значение температуры)
temperatureSettings DayOrNightTemperature(timerSettings,const RtcDateTime&); //Переключение между дневной и ночной температурой (настройки времени температуры, время)

void ControlLight(timerSettings,const RtcDateTime&); //Контроль освещения

void PrintAlltoLCD(float,float,float); //ph, температура, влажность

void setup() 
{
    lcd.begin(16,2);

//phSetValue = setPh();
//dayTemperature = SetDayT();
//nightTemperature = SetNightT();
//humiditySetValue = SetHumidity();
//temperatureTime = SetStartFinishTime();
    pinMode(phMinusPin,OUTPUT);        //ph
    digitalWrite(phMinusPin,HIGH);
    pinMode(phPlusPin,OUTPUT);
    digitalWrite(phPlusPin,HIGH);

    dht.begin();                       //dht
    pinMode(HumidityUPPin,OUTPUT);
    digitalWrite(HumidityUPPin,HIGH);
    pinMode(TempUPPin,OUTPUT);
    digitalWrite(TempUPPin,HIGH);
    pinMode(TempDOWNPin,OUTPUT);
    digitalWrite(TempDOWNPin,HIGH);

    pinMode(LightRelay,OUTPUT);       //light
    digitalWrite(LightRelay,HIGH);

    Rtc.Begin(); // запуск часов реального времени(Вставлять после настроек)

}

void loop() 
{
    RtcDateTime now = Rtc.GetDateTime();
    phVal = getPhValue();//Переменная с постоянно обновляемым показателем pH
    if (millis()-last_time >= (unsigned long)1000*30)
    {
    controlPh(phSetValue,phVal); //контроль с ручной настройкой параметров
    //controlPh(phSetProf1,phVal); //с предустановленной настройкой параметров
    last_time = millis();
    }

    temperatureVal = dht.readTemperature();
    humidityVal = dht.readHumidity();
    ControlHumidity(humiditySetValue,humidityVal);
    temperature = DayOrNightTemperature(temperatureTime,now);
    ControlTemperature(temperature,temperatureVal);

    ControlLight(lightTime,now);

    PrintAlltoLCD(phVal,temperatureVal,humidityVal);
}

void PrintAlltoLCD(float phValuePrint,float tval,float hval)
{
    lcd.setCursor(0,0);
    lcd.print("PH=");
    lcd.setCursor(3,0);
	lcd.print(phValuePrint,2);

    lcd.setCursor(0,1);
    lcd.print("t=");
    lcd.print(tval,0);

    lcd.print(" H=");
    lcd.print(hval,0);

}

///////------------FOR KEYPAD-----------/////////////////////////////

float getIntNumber()
{
    float num = 0;
    char key = kpd.getKey();
    while(key != '#')
   {
      switch (key)
      {
         case NO_KEY: case 'A': case 'B': case 'C': case '.':
            break;

         case '0': case '1': case '2': case '3': case '4':
         case '5': case '6': case '7': case '8': case '9':
         lcd.print(key);
            num = num * 10 + (key - '0');
            break;
         case '*':
            num = 0;
            lcd.clear();
            break;
         case '#':
            return num;
            break;
      }
      key = kpd.getKey();
   }
  return num;
}

float getFloatNumber()//Ввод значения float с клавиатуры
{
    float num = 0.0;
    float coef = 1.0;
    float fnum = 0.0;
  char key = kpd.getKey();
  while(key != '#' || key != '.')
   {
      switch (key)
      {
         case NO_KEY: case 'A': case 'B': case 'C':
            break;

         case '0': case '1': case '2': case '3': case '4':
         case '5': case '6': case '7': case '8': case '9':
         lcd.print(key);
            num = num * 10 + (key - '0');
            break;
        case '.':
            lcd.print(key);
            goto abc;
            break;
         case '*':
            num = 0;
            lcd.clear();
            break;
         case '#':
            return num;
            break;
      }
      key = kpd.getKey();
   }

   abc:
       while(key != '#')
       {
           switch (key)
            {
                case NO_KEY: case 'A': case 'B': case 'C': case '.':
                break;

                case '0': case '1': case '2': case '3': case '4':
                case '5': case '6': case '7': case '8': case '9':
                lcd.print(key);
                coef = coef * 0.1;
                fnum = ((key - '0')*coef);
                num = num + fnum;
                break;
                case '*':
                num = 0;
                lcd.clear();
                break;
                case '#':
                return num;
                break;
            }
            key = kpd.getKey();
       }
   return num;

}

///////---------PH------------////////////////////////////////////////////
float getPhValue() //Получить значение pH
{
  static unsigned long samplingTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(PHSensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergeArrayPh(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  return pHValue;
}

double avergeArrayPh(int* arr, int number)//Среднее значение pH
{
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5)
  {   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }
  else
  {
    if(arr[0]<arr[1])
    {
      min = arr[0];max=arr[1];
    }
    else
    {
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++)
    {
      if(arr[i]<min)
      {
        amount+=min;        //arr<min
        min=arr[i];
      }else 
      {
        if(arr[i]>max)
        {
          amount+=max;    //arr>max
          max=arr[i];
        }else
        {
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

void printPhValLCD(float phValuePrint)//Вывести значение pH на дисплей 16*2
{
    unsigned long printInterval = 1000;
    if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    lcd.setCursor(0,0);
    lcd.print("PH=");
    lcd.setCursor(3,0);
	lcd.print(phValuePrint,2);
    printTime=millis();
  }
}

phSettings setPh()//Настроить min/max/required ph, tank litrage
{
    phSettings val;
    lcd.print("Tank V:");
    lcd.setCursor(0,1);
    val.tankVolume = getFloatNumber();
    lcd.clear();
    lcd.print("req. pH:");
    lcd.setCursor(0,1);
    val.requiredPh = getFloatNumber();
    lcd.clear();
    lcd.print("min. pH:");
    lcd.setCursor(0,1);
    val.minPh = getFloatNumber();
    lcd.clear();
    lcd.print("max. pH:");
    lcd.setCursor(0,1);
    val.maxPh = getFloatNumber();
    lcd.clear();
    return val;
}
void controlPh(phSettings set,float ph)
{
    float difPh=0; // Хранит количество мл ph up/down для добавления в р-р
    if (ph >= set.maxPh) 
      difPh = (ph - set.requiredPh) * 3 * (set.tankVolume/10); // Вычисление к-ва pH DOWN для добавления в р-р
    else if (ph <= set.minPh)
      difPh = (set.requiredPh - ph) * 3 * (set.tankVolume/10); // Вычисление к-ва pH UP, для добавления в р-р
    
    if (ph >= set.maxPh) //Если pH р-ра больше максимального
        {
            if (difPh >= 1) // Если нужное к-во ph DOWN больше 1го мл то добавляем в р-р 1 мл
            {
                while(difPh >= 1)
                {
                difPh--;
                digitalWrite(phMinusPin, LOW);
                lcd.setCursor(0,1);
                lcd.print(difPh);
                delay(900);
                digitalWrite(phMinusPin,HIGH);
                }
            }
            if (difPh >= 0.75 && difPh < 1)
            {
                
                difPh-=0.75;
                digitalWrite(phMinusPin, LOW);
                lcd.setCursor(0,1);
                lcd.print(difPh);
                delay(700);
                digitalWrite(phMinusPin,HIGH);
            }
            if (difPh >=0.5 && difPh <0.75)
            {
                difPh-=0.5;
                digitalWrite(phMinusPin, LOW);
                lcd.setCursor(0,1);
                lcd.print(difPh);
                delay(500);
                digitalWrite(phMinusPin,HIGH);
            }
            if (difPh >0 && difPh <0.5)
            {
                difPh-=0.5;
                digitalWrite(phMinusPin, LOW);
                lcd.setCursor(0,1);
                lcd.print(difPh);
                delay(300);
                digitalWrite(phMinusPin,HIGH);
            }
            lcd.clear();
            difPh = 0;
        }
        else if (ph <= set.minPh) //Если ph р-ра меншье минимального
        {
            if (difPh >= 1)
            {
                while (difPh >= 1) // Если нужное к-во ph DOWN больше 1го мл то добавляем в р-р 1 мл
                {
                    difPh--;
                    digitalWrite(phPlusPin, LOW);
                    lcd.setCursor(0,1);
                lcd.print(difPh);
                    delay(900);
                    digitalWrite(phPlusPin,HIGH);
                }
            }
            if (difPh >= 0.75 && difPh < 1)
            {
                difPh-=0.75;
                digitalWrite(phPlusPin, LOW);
                lcd.setCursor(0,1);
                lcd.print(difPh);
                delay(700);
                digitalWrite(phPlusPin,HIGH);
            }
            else if (difPh >=0.5 && difPh <0.75)
            {
                difPh-=0.5;
                digitalWrite(phPlusPin, LOW);
                lcd.setCursor(0,1);
                lcd.print(difPh);
                delay(500);
                digitalWrite(phPlusPin,HIGH);
            }
            else if (difPh >0 && difPh <0.5)
            {
                difPh-=0.5;
                digitalWrite(phPlusPin, LOW);
                lcd.setCursor(0,1);
                lcd.print(difPh);
                delay(300);
                digitalWrite(phPlusPin,HIGH);
            }
            difPh = 0;
            lcd.clear();
        }
}

////////--------DHT------------////////////////////////////////////////////
temperatureSettings SetDayT()  //Задать нижнюю и верхнюю границы дневной температуры
{
    temperatureSettings val;
    lcd.clear();
    lcd.print("Day MIN t");
    lcd.print((char)223);
    lcd.print("C:");
    lcd.setCursor(0,1);
    val.TemperatureMin = getIntNumber();
    lcd.print(val.TemperatureMin);
    delay(2000);
    lcd.clear();
    lcd.print("Day MAX t");
    lcd.print((char)223);
    lcd.print("C:");
    lcd.setCursor(0,1);
    val.TemperatureMax = getIntNumber();
    lcd.print(val.TemperatureMax);
    delay(2000);
    lcd.clear();
    return val;
}

temperatureSettings SetNightT() //Задать нижнюю и верхнюю границы ночной температуры
{
    temperatureSettings val;
    lcd.clear();
    lcd.print("Night MIN t");
    lcd.print((char)223);
    lcd.print("C:");
    lcd.setCursor(0,1);
    val.TemperatureMin = getIntNumber();
    lcd.clear();
    lcd.print("Night MAX t");
    lcd.print((char)223);
    lcd.print("C:");
    lcd.setCursor(0,1);
    val.TemperatureMax = getIntNumber();
    lcd.clear();
    return val;
}

humiditySettings SetHumidity() //Задать нижнюю и верхнюю границы влажности
{
    humiditySettings val;
    lcd.clear();
    lcd.print("Min humidity %:");
    lcd.setCursor(0,1);
    val.humidityMin = getIntNumber();
    lcd.print(val.humidityMin);
    delay(2000);
    lcd.clear();
    lcd.print("Max humidity %:");
    lcd.setCursor(0,1);
    val.humidityMax = getIntNumber();
    lcd.print(val.humidityMax);
    delay(2000);
    lcd.clear();
    return val;
}
timerSettings SetStartFinishTime() //Задать начало дня и ночи
{
    timerSettings val;
    lcd.clear();
    lcd.print("Day start Hour:");
    lcd.setCursor(0,1);
    val.dayHourSet = getIntNumber();
    lcd.clear();
    lcd.print("Minute");
    lcd.setCursor(0,1);
    val.dayMinuteSet = getIntNumber();
    lcd.clear();
    lcd.print("Night start hour");
    lcd.setCursor(0,1);
    val.nightHourSet = getIntNumber();
    lcd.clear();
    lcd.print("Minute");
    lcd.setCursor(0,1);
    val.nightMinuteSet= getIntNumber();
    lcd.clear();
    return val;
}
///////------------FOR CONTROL-----------------/////////////////////////////////////////////////////////////////////////
void ControlHumidity(humiditySettings hset, float hval) //Контроль влажности воздуха
{
    if ((hval >= hset.humidityMin && hval <= hset.humidityMax) || hval > hset.humidityMax)
        digitalWrite(HumidityUPPin,HIGH);
    if (hval < hset.humidityMin)
        digitalWrite(HumidityUPPin, LOW);
}

void ControlTemperature(temperatureSettings tset, float tval) //Контроль температуры воздуха
{
    if (tval >= tset.TemperatureMin && tval <= tset.TemperatureMax)
    {
        digitalWrite(TempUPPin,HIGH);
        digitalWrite(TempDOWNPin,HIGH);
    }
    if (tval > tset.TemperatureMax)
    {
        digitalWrite(TempUPPin,HIGH);
        digitalWrite(TempDOWNPin,LOW);
        lcd.print("down");
    }
    if (tval < tset.TemperatureMin)
    {
        digitalWrite(TempUPPin,LOW);
        digitalWrite(TempDOWNPin,HIGH);
        lcd.print("up");
    }
}

temperatureSettings DayOrNightTemperature(timerSettings timeSet,const RtcDateTime& nowTime)//Переключение между дневной и ночной температурой
{
    temperatureSettings val;
    if (
        (nowTime.Hour() == timeSet.dayHourSet && nowTime.Minute() >= timeSet.dayMinuteSet)||
        (nowTime.Hour() > timeSet.dayHourSet && nowTime.Hour() < timeSet.nightHourSet)||
        (nowTime.Hour() == timeSet.nightHourSet && nowTime.Minute() < timeSet.nightMinuteSet)
        )
    {
        val.TemperatureMin = dayTemperature.TemperatureMin;
        val.TemperatureMax = dayTemperature.TemperatureMax;
        lcd.print("d");
    }
    else
    {
        lcd.print("n");
        val.TemperatureMin = nightTemperature.TemperatureMin;
        val.TemperatureMax = nightTemperature.TemperatureMax;
    }
    return val;
}

void PrintDHTtoLCD(float tval,float hval) //Вывести температуру и влажность
{
    lcd.setCursor(0,1);
    lcd.print("t=");
    lcd.print(tval,0);
    lcd.print(" H=");
    lcd.print(hval,0);
    lcd.print("%");
}

//////-----LIGHT CONTROL------///////////////

void ControlLight(timerSettings lightT, const RtcDateTime& nowTime) //Контроль светового дня
{
    if  (
        (nowTime.Hour() == lightT.dayHourSet && nowTime.Minute() >= lightT.dayMinuteSet) ||
        (nowTime.Hour() > lightT.dayHourSet && nowTime.Hour() < lightT.nightHourSet) ||
        (nowTime.Hour() == lightT.nightHourSet && nowTime.Minute() < lightT.nightMinuteSet)
        )
    {
        digitalWrite(LightRelay,LOW);
    }
    else
    {
        digitalWrite(LightRelay,HIGH);
    }
}

////////------Nutrients control------------//////////////////