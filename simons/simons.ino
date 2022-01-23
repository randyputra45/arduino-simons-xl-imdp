#include <WiFi.h>
#include <IOXhop_FirebaseESP32.h>
#include <NTPClient.h>
#include <time.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <BH1750.h>

//_____DEFINE TIME________
const long utcOffsetInSeconds = 25200;
int currentDay = 0;
WiFiClient client;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

//_________DEFINE LED_RGB__________
#define LEDR 23 // RED pin of rgb led is connected to 25 gpio pin  
#define LEDG 15 // green pin is connected to 26 gpio  
#define LEDB 19 //   
int m = 0;
#define R_channel 0
#define G_channel 1
#define B_channel 2
#define pwm_Frequency 5000 // pwm frequency  
#define pwm_resolution 8 // 8 bit resolution

//_________DEFINE RELAY__________
//int pin_Relay1 = 33;
int pin_Relay2 = 32;
String led_state = "";
//_________GY-30_____________
BH1750 lightMeter;
//#define pin_SCL 23
//#define pin_SDA 21

//__________DEFINE_DO_____________
#define DO_PIN 36

#define VREF_DO 5200    //VREF (mv)
#define ADC_RES 4096.0 //ADC Resolution

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

#define READ_TEMP 25 //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V 2498 //mv
#define CAL1_T 25   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V 1300 //mv
#define CAL2_T 15   //℃


float DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

float Temperaturet;
float ADC_Raw;
float ADC_Voltage;
float DO;
float DOValue;

float readDO(int voltage_mv, int temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  float V_saturation = (float)CAL1_V + (float)35 * temperature_c - (float)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  float V_saturation = (float)((float)temperature_c - CAL2_T) * ((float)CAL1_V - CAL2_V) / ((float)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

//__________DEFINE_HC_SR04_____________
const int trigPin = 5;
const int echoPin = 18;
//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701
long duration;
float jarakValue;
float hasil_Jarak;
//float jarakInch;

//_________DEFINE_PH____________
#define SensorPin 35          //pH meter Analog output to Arduino Analog Input 0
#define Offset -2.13           //deviation compensate
//#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;

//__________DEFINE_____________
#define TdsSensorPin 34 //TDS SENSOR
#define oneWireBus 14   //DS18B20
#define VREF 3.3        // volt TDS
#define SCOUNT  30      //SUM SAMPLING
//#define supply_tds 13   //supply untuk tds board

//_________DEFINE_TDS__________
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;
double k_value = 0;

//_______DEFINE_DS18B20_______
//const int oneWireBus = 14;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

void setup() {
  // pinMode(LED,OUTPUT);
  Serial.begin(115200);
  ledcAttachPin(LEDR, R_channel);
  ledcAttachPin(LEDG, G_channel);
  ledcAttachPin(LEDB, B_channel);
  ledcSetup(R_channel, pwm_Frequency, pwm_resolution);
  ledcSetup(G_channel, pwm_Frequency, pwm_resolution);
  ledcSetup(B_channel, pwm_Frequency, pwm_resolution);
  RGB_Color(255, 0, 0); // RED ccolor
  initWiFi();
  //________
  Wire.begin();
  //______PIN_RGB_SEBAGAI_OUTPUT______
  //pinMode(LEDR,OUTPUT);
  //pinMode(LEDG,OUTPUT);
  //pinMode(LEDB,OUTPUT);


  lightMeter.begin();
  Serial.println(F("BH1750 Test"));

  //pinMode(pin_Relay1, OUTPUT);
  pinMode(pin_Relay2, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(DO_PIN, INPUT);

  pinMode(oneWireBus, INPUT);
  pinMode(TdsSensorPin, INPUT);

  pinMode(SensorPin, INPUT);

  //pinMode(supply_tds, OUTPUT);
  sensors.begin();
  //digitalWrite(supply_tds, HIGH);
  timeClient.begin();
  timeClient.setTimeOffset(utcOffsetInSeconds);

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.setString("Control2/switch", "ON");

}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    RGB_Color(0, 255, 0); // GRE ccolor
    //digitalWrite(LEDG, HIGH);
    //digitalWrite(LEDR, LOW);
    //digitalWrite(LEDB, LOW);
    m = 1;
    //____BAGIAN WAKTU____
    while (!timeClient.update()) {
      timeClient.forceUpdate();
    }
    unsigned long epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime ((time_t *)&epochTime);
    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon + 1;
    int currentYear = ptm->tm_year + 1900;
    String strmonthDay;
    String strcurrentMonth;

    if (monthDay < 10) {
      strmonthDay = "0" + String(monthDay);
    }
    else {
      strmonthDay = String(monthDay);
    }
    if (currentMonth < 10) {
      strcurrentMonth = "0" + String(currentMonth);
    }
    else {
      strcurrentMonth = String(currentMonth);
    }
    String currentDate = strmonthDay + "-" + strcurrentMonth + "-" + String(currentYear);

    String currentTime = timeClient.getFormattedTime();

    String DateAndTime = currentDate + "_" + currentTime;
    //Serial.println(DateAndTime);

    //____KODING CONTROL_____
    led_state = Firebase.getString("Control2/switch");
    Serial.println(led_state);
    if (led_state == "ON") {
      Serial.println("relay 1 hidup");
      digitalWrite(pin_Relay2, LOW);
    }
    else if (led_state == "OFF") {
      Serial.println("relay 1 mati");
      digitalWrite(pin_Relay2, HIGH);
    }
    else {
      Serial.println("error");
    }

    //___KODING_SEN_CAHAYA_______
    float lux = lightMeter.readLightLevel();

    //____SEN_DO________________
    Temperaturet = (float)READ_TEMP;
    ADC_Raw = analogRead(DO_PIN);
    ADC_Voltage = float(VREF_DO) * ADC_Raw / ADC_RES;
    DOValue = readDO(ADC_Voltage, Temperaturet);

    //_____SEN_JARAK_____
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculate the distance
    jarakValue = duration * SOUND_SPEED / 2;
    hasil_Jarak = 25 - jarakValue;

    //____SEN_PH______________
    static unsigned long samplingTime = millis();
    static unsigned long printTime = millis();
    static float pHValue, voltage;
    if (millis() - samplingTime > samplingInterval)
    {
      pHArray[pHArrayIndex++] = analogRead(SensorPin);
      if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
      voltage = avergearray(pHArray, ArrayLenth) * 5.2 / 4096;
      pHValue = 3.5 * voltage + Offset;
      samplingTime = millis();
    }

    //______SEN_TDS___________
    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
    {
      analogSampleTimepoint = millis();
      analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
      analogBufferIndex++;
      if (analogBufferIndex == SCOUNT)
        analogBufferIndex = 0;
    }
    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U)
    {
      printTimepoint = millis();
      for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
        analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
      //double k_value = 1180.0000 / (133.4200 * compensationVolatge * compensationVolatge * compensationVolatge - 255.8600 * compensationVolatge * compensationVolatge + 857.3900 * compensationVolatge);
      tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.53; //convert voltage value to tds value
      /*Serial.print(", TDS: ");
        Serial.print(tdsValue);
        Firebase.setFloat("DataSensor/TDS", tdsValue);
        if (Firebase.failed()) {
        Serial.print("setting /message failed:");
        RGB_Color(0, 0, 255); // BLU ccolor
        delayMicroseconds(10000);
        Serial.println(Firebase.error());
        return;
        }*/
    }

    /*if (WiFi.status() == WL_CONNECTED) {
      RGB_Color(0, 255, 0); // GRE ccolor
      //digitalWrite(LEDG, HIGH);
      //digitalWrite(LEDR, LOW);
      //digitalWrite(LEDB, LOW);
      m = 1;*/
    //_____SEND_DATA_ALL____
    if (millis() - printTime > printInterval)  //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
    {
      sensors.requestTemperatures();
      float temperatureC = sensors.getTempCByIndex(0);
      float temperatureF = sensors.getTempFByIndex(0);
      //k_value = float(k_value);
      //Serial.print(k_value);
      Serial.print("  C: ");
      Serial.print(temperatureC);
      Firebase.setFloat("DataSensor/suhuC", sensors.getTempCByIndex(0));
      if (Firebase.failed()) {
        Serial.print("setting /message failed:");
        RGB_Color(0, 0, 255); // BLU ccolor
        delayMicroseconds(10000);
        Serial.println(Firebase.error());
        return;
      }
      Serial.print(", F: ");
      Serial.print(temperatureF);
      Firebase.setFloat("DataSensor/suhuF", sensors.getTempFByIndex(0));

      Serial.print(", TDS: ");
      Serial.print(tdsValue);
      Firebase.setFloat("DataSensor/TDS", tdsValue);
      if (Firebase.failed()) {
        Serial.print("setting /message failed:");
        RGB_Color(0, 0, 255); // BLU ccolor
        delayMicroseconds(10000);
        Serial.println(Firebase.error());
        return;
      }
      Serial.print(", Distance (cm): ");
      Serial.print(hasil_Jarak);
      Firebase.setFloat("DataSensor/Jarak", hasil_Jarak);
      if (Firebase.failed()) {
        Serial.print("setting /message failed:");
        RGB_Color(0, 0, 255); // BLU ccolor
        delayMicroseconds(10000);
        Serial.println(Firebase.error());
        return;
      }
      Serial.print(", Light: ");
      Serial.print(lux);
      Firebase.setFloat("DataSensor/Cahaya", lux);
      if (Firebase.failed()) {
        Serial.print("setting /message failed:");
        RGB_Color(0, 0, 255); // BLU ccolor
        delayMicroseconds(10000);
        //Serial.println(Firebase.error());
        return;
      }
      Serial.print(", pH value: ");
      Serial.print(pHValue, 2);
      Firebase.setFloat("DataSensor/pH", pHValue);
      if (Firebase.failed()) {
        Serial.print("setting /message failed:");
        RGB_Color(0, 0, 255); // BLU ccolor
        delayMicroseconds(10000);
        //Serial.println(Firebase.error());
        return;
      }
      printTime = millis();
      Serial.print(" ,DO: ");
      Serial.println(DOValue);
      Firebase.setFloat("DataSensor/DO", DOValue);
      if (Firebase.failed()) {
        Serial.print("setting /message failed:");
        RGB_Color(0, 0, 255); // BLU ccolor
        //delayMicroseconds(10000);
        Serial.println(Firebase.error());
        return;
      }
      if (m == 1) {
        Firebase.setString("DataSensor/DateAndTime", DateAndTime);
        //RGB_Color(255, 20, 147); // yellow ccolor
        //delayMicroseconds(10000);
      }
      //Firebase.setString("DataSensor/DateAndTime", DateAndTime);
      Serial.println("Sending Sensor Data Successfully");
      Serial.println();
    }
  }
  else {
    RGB_Color(255, 0, 0); // RED ccolor
    //digitalWrite(LEDG, LOW);
    //digitalWrite(LEDR, HIGH);
    //digitalWrite(LEDB, LOW);
    m = 0;
    //ESP.restart();
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
  }
}

void RGB_Color(int p, int q, int r)
{
  ledcWrite(R_channel, p);
  ledcWrite(G_channel, q);
  ledcWrite(B_channel, r);
}
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

double avergearray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) { //less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0]; max = arr[1];
    }
    else {
      min = arr[1]; max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;      //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  //arr>max
          max = arr[i];
        } else {
          amount += arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount / (number - 2);
  }//if
  return avg;
}
