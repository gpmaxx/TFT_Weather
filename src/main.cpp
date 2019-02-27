/*
*  Version:    2019-02-22 gpmaxx  initial release
*
*  Desc:       Weather Helper for Wemos D1 Mini and 128 & 160 TFT_eSPI
*
*  Libraries:  ArduinoJson:  https://github.com/bblanchon/ArduinoJson
*              TFT_eSPI:     https://github.com/Bodmer/TFT_eSPI
*              Bounce2:      https://github.com/thomasfredericks/Bounce2
*
*  Notes:      Pin assignments below are very particular. The D1 is finicky.
*              Changing pin assignments or adding additional connections is likely
*              to cause headaches.  The TFT_eSPI defaults are thus overridden
*              with compile options (see platformio.ini) to use the pins that are
*              most convienient. Using any other board beside there Wemos D1 Mini has never been tested.
*
*              Wemos D1 pin       connect to
*              ------------       ----------
*              RST                optionaly to GND via a button (external reset)
*              A0,D8,TX,RX,5V     nothing
*              D0                 TFT AO
*              D5                 TFT SCK
*              D6                 GND via Switch 1 (used for display mode)
*              D7                 TFT SDA
*              3V                 TFT VCC
*              D1                 GND via button (select button)
*              D2                 TFT CS
*              D3                 TFT RST
*              D4                 TFT LED
*              G                  Ground
*
*              Code assume 128 x 160 TFT display. The graphics functions assume
*              this size and use some hardcoded and magic values to get things
*              looking right. A different size screen the values will have to
*              be changed carefully.
*
*  ToDo:      Add humidex calculation
              switch to control units

*
*
*   Maybe:    Switch API to one that includes POP value
*             move humidex to spiffs or progmmem
*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Bounce2.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Time.h>
#include <TimeLib.h>
#include <Timezone.h>

#define FS_NO_GLOBALS
#include "FS.h"
#include "BMP_functions.h"


////////////////// Global Constants //////////////////
const char* WIFI_CONFIG_AP = "ESP_WIFI_CONFIG";
const uint8_t SWITCH_PIN_1 = D6;
const uint8_t LED_BACKLIGHT_PIN = D4;
const uint8_t SELECT_BUTTON_PIN = D1;
const uint8_t TFT_ROTATION = 3;
const uint8_t DEBOUNCE_INTERVAL = 25;

const String CITY_ID = "6094578"; // Oshawa
const String API_KEY = "6cbddad616a3b1956cbdf325c9da3058";
const char* SERVER_NAME = "api.openweathermap.org";

const char* TIME_HOST = "worldtimeapi.org";
const uint8_t TIME_PORT = 80;
const uint32_t TIME_UPDATE_INTERVAL_MS = 1000 * 60 * 60 * 24; // 24 hours
const uint16_t LONG_PRESS_THRESHOLD = 1000;

const uint32_t DISPLAY_UPDATE_INTERVAL = 1000 * 60 * 15; // 15 mins
const char* MONTHS_OF_YEAR[] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
const char* DAYS_OF_WEEK[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
const uint8_t TFT_WIDTH_FULL = 160;
const uint8_t TFT_WIDTH_QUATER = 40;
const uint8_t TFT_WIDTH_HALF = 80;
const uint8_t TFT_WIDTH_THREEQUATERS = 120;
const uint16_t TFT_HALF_HEIGHT = 64;
///////////// Global Variables ////////////
String queryString;
char friendlyDate[12];             // date buffer format: Mon, Jan 23
char friendlyTime[8];              // time buffer format: 12:34pm
bool detailedMode = false;

uint8_t humidexTable[19][29] =
{{16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44},
{17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45},
{17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45},
{18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46},
{18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46},
{19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47},
{0,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48},
{0,0,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48},
{0,0,0,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49},
{0,0,0,0,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50},
{0,0,0,0,0,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51},
{0,0,0,0,0,0,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51},
{0,0,0,0,0,0,0,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52},
{0,0,0,0,0,0,0,0,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53},
{0,0,0,0,0,0,0,0,0,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54},
{0,0,0,0,0,0,0,0,0,0,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55},
{0,0,0,0,0,0,0,0,0,0,0,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56},
{0,0,0,0,0,0,0,0,0,0,0,0,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58},
{0,0,0,0,0,0,0,0,0,0,0,0,0,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59} };

/////////////// Data Structures //////////

enum WeatherType {EMPTY,CURRENT,FORECAST_TODAY,FORECAST_TOMORROW};

struct WeatherData {
    WeatherType type;
    bool isMetric;
    uint8_t conditionID;
    char description[20];
    char icon[8];
    uint16_t pressure;
    uint8_t humidity;
    float temp;
    float tempMin;
    float tempMax;
    float windSpeed;
    time_t timestamp;
    time_t sunrise;
    time_t sunset;
    float windChill;
    float humidex;
};



/////////// Global Object Variables ///////////
WiFiManager wifiManager;
TFT_eSPI tft = TFT_eSPI();
Bounce debouncer = Bounce();
WiFiClient client;
WeatherData currentWeather;
TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};  //UTC - 4 hours
TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};   //UTC - 5 hours
Timezone tz(usEDT, usEST);


////////////////  Code //////////////////////

void infiniteLoop() {
  Serial.println(F("infinite delay"));
  while (true) {
    delay(0xFFFFFFFF);
  }
}

void tftMessage(char* theMessage) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0,0);
  tft.print(theMessage);
}

void tftMessage(String theMessage) {
  char buffer[theMessage.length()+1];
  theMessage.toCharArray(buffer,theMessage.length()+1);
  tftMessage(buffer);
}

void wifiConfigCallback(WiFiManager* myWiFiManager) {
  Serial.println(F("Entered WiFi Config Mode"));
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
  tftMessage(myWiFiManager->getConfigPortalSSID());
}

void wifiConnect() {

  Serial.println(F("Connecting to WiFi..."));
  if (!wifiManager.autoConnect(WIFI_CONFIG_AP)) {
    Serial.println(F("failed to connect timout"));
    tftMessage("WiFi connect timeout");
    delay(10000);
    ESP.reset();
    delay(10000);
  }

  Serial.println(F("WiFi Connected"));

}

void ledSwitchInterrupt() {
    Serial.println("switch interrupt");
}


//  Format:  Mon, Jan 23
void setFriendlyDate(char* formattedDate, const uint8_t buffSize, const time_t epoch) {
  String convertedDate = DAYS_OF_WEEK[weekday(epoch)-1];
  convertedDate += ", ";
  convertedDate += MONTHS_OF_YEAR[month(epoch)-1];
  convertedDate += " ";
  convertedDate += day(epoch);
  convertedDate.toCharArray(formattedDate,buffSize);
}

void setFriendlyTime(char* formattedTime, const uint8_t buffSize, const time_t epoch) {
  uint8_t h = hour(epoch);
  char* partOfDay = "am";
  if (h >= 0 &&  h <= 11) {
      if (h == 0) {
        h = 12;
      }
  }
  else {
    partOfDay = "pm";
    if (h != 12) {
      h -= 12;
    }
  }
  snprintf(formattedTime,buffSize,"%d:%02d%s",h,minute(epoch),partOfDay);
}

void updateTime() {
  static uint32_t lastTimeUpdate = 0;

  if ((lastTimeUpdate == 0) || ((millis() - lastTimeUpdate) > TIME_UPDATE_INTERVAL_MS)) {
    client.setTimeout(10000);
    if (client.connect(TIME_HOST,TIME_PORT)) {
      queryString = "GET /api/ip HTTP/1.0";
      client.println(queryString);
      queryString = "Host: ";
      queryString += TIME_HOST;
      client.println(queryString);
      client.println(F("Connection: close"));
      if (client.println() == 0) {
        Serial.println(F("Failed to send request"));
      }

    }
    else {
      Serial.println(F("connection failed")); //error message if no client connect
      Serial.println();
    }

    while(client.connected() && !client.available()) delay(1);

    char endOfHeaders[] = "\r\n\r\n";
    if (!client.find(endOfHeaders)) {
      Serial.println(F("Invalid response"));
      return;
    }

    client.stop();

    DynamicJsonBuffer jsonBuffer(500);

    JsonObject& root = jsonBuffer.parseObject(client);
    if (!root.success()) {
      Serial.println(F("Parsing failed!"));
      tftMessage("time query error");
      Serial.println("time query error");
      infiniteLoop();
    }

    time_t epochTime = root["unixtime"];
    Serial.print("UTC epoch: ");
    Serial.println(epochTime);

    setTime(tz.toLocal(epochTime));
    Serial.print("Local epoch: ");
    Serial.println(now());
    lastTimeUpdate = millis();

    setFriendlyDate(friendlyDate,sizeof(friendlyDate),now());
    setFriendlyTime(friendlyTime,sizeof(friendlyTime),now());

  }

}

void clearWeather(WeatherData* theWeather) {
  theWeather->type = EMPTY;
  theWeather->conditionID = 0;
  memset(theWeather->description,'\0',sizeof(theWeather->description));
  memset(theWeather->icon,'\0',sizeof(theWeather->icon));
  theWeather->pressure = 0;
  theWeather->humidity= 0;
  theWeather->temp = 0;
  theWeather->tempMin = 0;
  theWeather->tempMax = 0;
  theWeather->windSpeed = 0;
  theWeather->timestamp = 0;
  theWeather->sunrise = 0;
  theWeather->sunset = 0;
}

void printTempUnits(const bool isMetric) {
    if (isMetric) {
      Serial.println("°C");
    }
    else {
      Serial.println("°F");
    }
}

void printWind(const float windspeed, const bool isMetric) {
    Serial.print("Windspeed: ");
    if (isMetric) {
        Serial.print(windspeed * 3.6F); // m/s -> km/h conversion
        Serial.println("kph");
    }
    else {

      Serial.print(windspeed);
      Serial.println("mph");
    }
}



void printWeather(WeatherData* theWeather) {
  time_t t;
  Serial.printf("Type: %d\r\n",theWeather->type);
  Serial.printf("Units: %s\r\n",(theWeather->isMetric) ? "Metric" : "Imperial");\
  t = tz.toLocal(theWeather->timestamp);
  Serial.printf("Date: %d-%02d-%02d\r\n",year(t),month(t),day(t));
  setFriendlyTime(friendlyTime,sizeof(friendlyTime),t);
  Serial.printf("ConditionID: %d\r\n",theWeather->conditionID);
  Serial.printf("Description: %s\r\n",theWeather->description);
  Serial.printf("Icon: %s\r\n",theWeather->icon);
  Serial.printf("Pressure: %d\r\n",theWeather->pressure);
  Serial.printf("Humidity: %d%s\r\n",theWeather->humidity,"%");
  Serial.print("Temp: ");
  Serial.print(theWeather->temp);
  printTempUnits(theWeather->isMetric);
  if ((theWeather->type == FORECAST_TODAY) || (theWeather->type == FORECAST_TOMORROW)) {
    Serial.print("Day Min Temp: ");
    Serial.print(theWeather->tempMin);
    printTempUnits(theWeather->isMetric);
    Serial.print("Day Max Temp: ");
    Serial.print(theWeather->tempMax);
    printTempUnits(theWeather->isMetric);
  }
  printWind(theWeather->windSpeed,theWeather->isMetric);
  if (theWeather->type == CURRENT) {
    t = tz.toLocal(theWeather->sunrise);
    setFriendlyTime(friendlyTime,sizeof(friendlyTime),t);
    Serial.printf("Sunrise: %s\r\n",friendlyTime);
    t = tz.toLocal(theWeather->sunset);
    setFriendlyTime(friendlyTime,sizeof(friendlyTime),t);
    Serial.printf("Sunset: %s\r\n",friendlyTime);
  }
}

float FToC(float valueInF) {
  return (valueInF - 32) * .55555;
}

float CToF(float valueInC) {
  return (valueInC * 1.8) + 32;
}

// return the humidex or windchill for the provided weather conditions
// humidex is approximate based on approximate dewpoint
int16_t feelsLike(WeatherData* theWeather) {

  float v16;
  float result;
  uint8_t dewPoint;

  if (theWeather->isMetric) {

    if ((theWeather->temp <= 10) && (theWeather->windSpeed > 4.8)) {
        result = pow(theWeather->windSpeed,0.16) * ((0.3965 * theWeather->temp) - 11.37);
        result += 13.12;
        result += (0.6215 * theWeather->temp);
    }
    else {
        dewPoint = round(theWeather->temp - ((100 - theWeather->humidity)/5));
        if ((dewPoint >= 10) && (dewPoint <= 28) && (theWeather->temp >= 15) &&  (theWeather->temp <= 43)) {
            result =  humidexTable[dewPoint - 10][round(theWeather->temp - 15)];
        }
        else {
          result = theWeather->temp;
        }


    }
  }
  else {
    if ((theWeather->temp <= 50 )  && (theWeather->windSpeed > 3 )) {
        result = pow(theWeather->temp,0.16) * ((0.4275 * theWeather->temp) - 35.75);
        result += 35.74;
        result += (0.6215 * theWeather->temp);
    }
    else {
      uint8_t tempC = FToC(theWeather->temp);
      dewPoint = round(tempC - ((100 - theWeather->humidity)/5));
      if ((dewPoint >= 10) && (dewPoint <= 28) && (tempC >= 15) &&  (tempC <= 43)) {
          result =  CToF(humidexTable[dewPoint - 10][round(tempC - 15)]);
      }
      else {
        result = theWeather->temp;
      }

    }

  }

  return round(result);
}

void displayUpdate(WeatherData* theWeather) {
  char filepath[15];
  char tBuffer[20];

  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_BLACK,TFT_WHITE);

  if (theWeather->type == CURRENT) {
    tft.drawString(friendlyDate,TFT_WIDTH_HALF - (tft.textWidth(friendlyDate,4)/2),2,4);
    if (detailedMode) {
      tft.setTextSize(1);
      tft.fillScreen(TFT_WHITE);
      tft.setTextColor(TFT_BLACK);
      tft.setCursor(0,0);

      tft.println("Date: Most Recent");
      setFriendlyDate(friendlyDate,sizeof(friendlyDate),tz.toLocal(theWeather->timestamp));
      tft.printf("Date: %s\n",friendlyDate);
      setFriendlyTime(friendlyTime,sizeof(friendlyTime),tz.toLocal(theWeather->timestamp));
      tft.printf("Time: %s\n",friendlyTime);
      tft.printf("Temp: %d%s\n",round(theWeather->temp),(theWeather->isMetric) ? "C" : "F");
      tft.printf("Desc: %s\n",theWeather->description);
      tft.printf("Hum:  %d%%\n",theWeather->humidity);
      tft.printf("Wind: %d%s\n",round(theWeather->windSpeed),(theWeather->isMetric) ? "kph":"mph");
      tft.printf("Feel: %d%s\n",feelsLike(theWeather),(theWeather->isMetric) ? "C" : "F");
      setFriendlyTime(friendlyTime,sizeof(friendlyTime),theWeather->sunrise);
      tft.printf("Rise: %s\n",friendlyTime);
      setFriendlyTime(friendlyTime,sizeof(friendlyTime),theWeather->sunset);
      tft.printf("Set:  %s\n",friendlyTime);
    }
    else {
      snprintf(filepath,sizeof(filepath),"%s%s%s","/icons/",theWeather->icon,".bmp");
      drawBmp(&tft,filepath,10,30);
      snprintf(tBuffer,sizeof(tBuffer),"%d",round(theWeather->temp));
      tft.drawString(tBuffer,TFT_WIDTH_THREEQUATERS - (tft.textWidth(tBuffer,7)/2),30,7);
      setFriendlyTime(friendlyTime,sizeof(friendlyTime),now());
      tft.drawString(friendlyTime,TFT_WIDTH_HALF - (tft.textWidth(friendlyTime,4)/2),90,4);
    }

  }
  else {
    tft.setTextSize(1);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK);
    tft.setCursor(0,0);

    if (detailedMode) {
      if (theWeather->type == FORECAST_TODAY) {
        tft.println("Date: Today");
      }
      else {
        tft.println("Date: Tomorrow");
      }

      setFriendlyDate(friendlyDate,sizeof(friendlyDate),theWeather->timestamp);
      tft.printf("Date: %s\n",friendlyDate);
      tft.printf("Min:  %d%s\n",round(theWeather->tempMin),(theWeather->isMetric) ? "C" : "F");
      tft.printf("Max:  %d%s\n",round(theWeather->tempMax),(theWeather->isMetric) ? "C" : "F");
      tft.printf("Desc: %s\n",theWeather->description);
      tft.printf("Hum:  %d%%\n",theWeather->humidity);
      tft.printf("Wind: %d%s\n",round(theWeather->windSpeed),(theWeather->isMetric) ? "kph":"mph");
      tft.printf("Feel: %d%s",feelsLike(theWeather),(theWeather->isMetric) ? "C" : "F");
    }
    else {
      if (theWeather->type == FORECAST_TODAY) {
        tft.drawString("Today",40,0,4);
      }
      else {
        tft.drawString("Tomorrow",25,0,4);
      }
      snprintf(filepath,sizeof(filepath),"%s%s%s","/icons/",theWeather->icon,".bmp");
      drawBmp(&tft,filepath,10,30);
      snprintf(tBuffer,sizeof(tBuffer),"Min: %d",round(theWeather->tempMin));
      tft.drawString(tBuffer,100,35,2);
      snprintf(tBuffer,sizeof(tBuffer),"Max: %d",round(theWeather->tempMax));
      tft.drawString(tBuffer,100,55,2);
      snprintf(tBuffer,sizeof(tBuffer),"%s",theWeather->description);
      tft.drawString(tBuffer,TFT_WIDTH_HALF - (tft.textWidth(tBuffer,2)/2),90,2);
    }
  }

}

void getCurrentWeather(const String cityID, const bool isMetric, WeatherData* theWeather) {
  Serial.println(F("Fetching weather"));
  if (client.connect(SERVER_NAME, 80)) {  //starts client connection, checks for connection
    queryString = "GET /data/2.5/weather?id=" + cityID + "&units=";
    if (!isMetric) {
      queryString += "imperial";
    }
    else
    {
      queryString += "metric";
    }
    queryString += "&cnt=2&APPID=" + API_KEY + "\r\n";
    Serial.print(queryString);
    queryString += "Host: ";
    queryString += SERVER_NAME;
    queryString += "\r\n";
    queryString += "User-Agent: ArduinoWiFi/1.1";
    queryString += "\r\n";
    queryString += "Connection: close";
    queryString += "\r\n\r\n";

    client.print(queryString);
  }
  else {
    Serial.println("connection failed"); //error message if no client connect
    Serial.println();
    return;
  }

  Serial.print(F("Waiting"));
  while(client.connected() && !client.available()) {
    delay(5); //waits for data
    Serial.print(F("."));
  }
  Serial.println();

  /*
  char endOfHeaders[] = "\r\n\r\n";
  if (!client.find(endOfHeaders)) {
    Serial.println(F("Invalid response"));
    return;
  }
  */

  DynamicJsonBuffer jsonBuffer(2000);

  JsonObject& root = jsonBuffer.parseObject(client);
  if (!root.success()) {
    Serial.println(F("Parsing failed!"));
    return;
  }

  client.stop();

  JsonObject& weather_0 = root["weather"][0];
  clearWeather(theWeather);
  theWeather->type = CURRENT;
  theWeather->isMetric = isMetric;
  theWeather->conditionID = weather_0["id"];
  const char* cond = weather_0["description"];
  strncpy(theWeather->description,cond,sizeof(theWeather->description));
  const char* icon = weather_0["icon"];
  strncpy(theWeather->icon,icon,sizeof(icon));

  JsonObject& main = root["main"];
  theWeather->temp = main["temp"];
  theWeather->pressure = main["pressure"];
  theWeather->humidity = main["humidity"];
  theWeather->windSpeed = root["wind"]["speed"];
  theWeather->timestamp = root["dt"];
  theWeather->sunrise = root["sys"]["sunrise"];
  theWeather->sunrise = tz.toLocal(theWeather->sunrise);
  theWeather->sunset = root["sys"]["sunset"];
  theWeather->sunset = tz.toLocal(theWeather->sunset);

  printWeather(theWeather);


}

void getForecastWeather(const String cityID, const bool isMetric, const bool forToday, WeatherData* theWeather)
{

  Serial.println(F("Getting Weather Data"));
  if (client.connect(SERVER_NAME, 80)) {  //starts client connection, checks for connection
    queryString = "GET /data/2.5/forecast/daily?id=" + cityID + "&units=";
    if (isMetric) {
      queryString += "metric";
    }
    else
    {
      queryString += "imperial";
    }
    queryString += "&cnt=2&APPID=" + API_KEY + "\r\n";
    queryString += "Host: ";
    queryString += SERVER_NAME;
    queryString += "\r\n";
    queryString += "User-Agent: ArduinoWiFi/1.1";
    queryString += "\r\n";
    queryString += "Connection: close";
    queryString += "\r\n\r\n";

    Serial.println("Query String: ");
    Serial.println(queryString);
    client.print(queryString);
  }
  else {
    Serial.println("connection failed"); //error message if no client connect
    Serial.println();
    return;
  }

  Serial.println(F("Waiting for data"));
  while(client.connected() && !client.available()) {
    delay(5); //waits for data
    Serial.print(F("."));
  }
  Serial.println();

  /*
  char endOfHeaders[] = "\r\n\r\n";
  if (!client.find(endOfHeaders)) {
    Serial.println(F("Invalid response"));
    return;
  }
  */

  DynamicJsonBuffer jsonBuffer(2000);

  JsonObject& root = jsonBuffer.parseObject(client);
  if (!root.success()) {
    Serial.println(F("Parsing failed!"));
    return;
  }

  JsonArray &list = root["list"];
  uint8_t index = forToday ? 0 : 1;

  clearWeather(theWeather);
  if (forToday) {
    theWeather->type = FORECAST_TODAY;
  }
  else {
    theWeather->type = FORECAST_TOMORROW;
  }
  theWeather->isMetric = isMetric;
  JsonObject&  listEntry = list[index];
  JsonObject& weather = listEntry["weather"][0];
  const char* cond = weather["description"];
  strncpy(theWeather->description,cond,sizeof(theWeather->description));
  const char* icon = weather["icon"];
  strncpy(theWeather->icon,icon,sizeof(icon));
  theWeather->pressure = listEntry["pressure"];
  theWeather->humidity = listEntry["humidity"];
  theWeather->temp = listEntry["temp"]["day"];
  theWeather->tempMin = listEntry["temp"]["min"];
  theWeather->tempMax = listEntry["temp"]["max"];
  theWeather->windSpeed = listEntry["speed"];
  theWeather->timestamp = listEntry["dt"];

  printWeather(theWeather);

}

void setup() {
  Serial.begin(74880);
  Serial.println(F("\r\nTFT Weather"));
  tft.init(INITR_BLACKTAB);
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(TFT_BLACK);

  tftMessage("TFT Weather");

  pinMode(SWITCH_PIN_1,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN_1),ledSwitchInterrupt,CHANGE);

  pinMode(LED_BACKLIGHT_PIN,OUTPUT);
  digitalWrite(LED_BACKLIGHT_PIN,HIGH); // high to turn on

  debouncer.attach(SELECT_BUTTON_PIN,INPUT_PULLUP);
  debouncer.interval(DEBOUNCE_INTERVAL);

  if (!SPIFFS.begin()) {
    Serial.println(F("SPIFFS initialisation failed!"));
    tftMessage("SPIFFS Error");
    infiniteLoop();
  }
  Serial.println(F("\r\n\SPIFFS initialised."));

  //wifiManager.resetSettings();
  wifiManager.setAPCallback(wifiConfigCallback);
  wifiConnect();

  queryString.reserve(100);

  updateTime();


}

void loop() {
  static int8_t prevMin = minute(now());
  static bool firstrun = true;
  static uint32_t lastUpdateTime;
  static uint32_t buttonTimer;
  static WeatherType currentMode = CURRENT;

  bool shouldQuery = firstrun;
  bool shouldUpdate = false;

  debouncer.update();
  if (debouncer.fell()) {
    Serial.println("fell");
    buttonTimer = millis();
  }
  if (debouncer.rose()) {
    Serial.println("rose");
    if ((millis() - buttonTimer) > LONG_PRESS_THRESHOLD) {
      detailedMode = !detailedMode;
      shouldUpdate = true;
    }
    else {
      switch (currentMode) {
        shouldQuery = true;
        case CURRENT:
          currentMode = FORECAST_TODAY;
          break;
        case FORECAST_TODAY:
          currentMode = FORECAST_TOMORROW;
          break;
        case FORECAST_TOMORROW:
          currentMode = CURRENT;
          break;
       }
       shouldQuery = true;
    }
  }

  if ((millis() - lastUpdateTime) > DISPLAY_UPDATE_INTERVAL) {
    shouldQuery = true;
  }
  if (shouldQuery) {
    switch(currentMode) {
      tft.fillScreen(TFT_WHITE);
      tft.drawString("Querying...",0,0,2);
      case CURRENT:
        getCurrentWeather(CITY_ID,false,&currentWeather);
        break;
      case FORECAST_TODAY:
        getForecastWeather(CITY_ID,false,true,&currentWeather);
        break;
      case FORECAST_TOMORROW:
        getForecastWeather(CITY_ID,false,false,&currentWeather);
        break;
    }
    lastUpdateTime = millis();
    shouldUpdate = true;

  }
  if ((currentWeather.type == CURRENT) && (prevMin != minute(now()))) {
    prevMin = minute(now());
    shouldUpdate = true;
  }
  if (shouldUpdate) {
    displayUpdate(&currentWeather);
    firstrun = false;
  }

  delay(10);
  updateTime();

}
