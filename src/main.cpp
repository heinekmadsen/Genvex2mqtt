/**
  Genvex and Nila (Still to be added) Modbus firmware for ESP32 together with a TTL to RS485 Converter https://www.aliexpress.com/item/32836213346.html?spm=a2g0s.9042311.0.0.27424c4dqnr5i7
 
  Written by Dan Gunvald
    https://github.com/DanGunvald/NilanModbus
  Modified to use with Home Assistant by Anders Kvist, Kenn Dyrvig, Jacob Scherrebeck, Heine Madsen & other great people :)
    https://github.com/anderskvist/Nilan_Homeassistant
    https://github.com/jascdk/Nilan_Homeassistant
    https://github.com/heinekmadsen/Genvex2mqtt
   
  Read from a Nilan Air Vent System or Genvex Optima 250 (Danish Brand) using a ESP32
  and report the values to an MQTT broker. Then use it for your home-automation system like Home Assistant.
 
  External dependencies. Install using the Arduino library manager or PlatformIO if using VS Code:
 
     "Arduino JSON V6 by Benoît Blanchon https://github.com/bblanchon/ArduinoJson - IMPORTANT - Use latest V.6 !!! This code won´t compile with V.5
     "ModbusMaster by Doc Walker https://github.com/4-20ma/ModbusMaster
     "PubSubClient" by Nick O'Leary https://github.com/knolleary/pubsubclient
     "Double Reset Detector" by DataCute https://github.com/datacute/DoubleResetDetector
     "WifiManager" by Tzapu and Tablatronix https://github.com/tzapu/WiFiManager
     
  Project inspired by https://github.com/DanGunvald/NilanModbus
  Join this Danish Facebook Page for inspiration :) https://www.facebook.com/groups/667765647316443/
*/




#include <FS.h>          // this needs to be first, or it all crashes and burns...
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include "HardwareSerial.h"
#include <SPIFFS.h>
#include <WiFiClient.h>
#include <PubSubClientTools.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <Ticker.h>
#include <time.h>

/*------------------------------------------------------------------------------ 
VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------ 
VARIABLES - Shared
------------------------------------------------------------------------------*/

//enum reqtypes {reqtemp, reqruninfo, reqtemptarget, reqspeed, reqtime, reqhumidity, reqversion, reqmax};
String manufacturer;
// Time stuff
const int daylightOffset_sec = 3600;
const char* ntpServer = "pool.ntp.org";
String bootTime = "";

// Wifi
WiFiServer server(80);
WiFiClient client;
//default custom static IP
char static_ip[16] = "10.0.1.56";
char static_gw[16] = "10.0.1.1";
char static_sn[16] = "255.255.255.0";
char chipid[12];
//flag for saving data
bool shouldSaveConfig = false;

// MQTT
PubSubClient mqttclient(client);
bool configurationPublished[40];
bool firstLoop = true;
String lastReadValues[600];
// Esp8266 MAC will be added to the device name, to ensure unique topics
// Default is topics like 'heat/floorXXXXXXXXXXXX/3/target', where 3 is the output id and XXXXXXXXXXXX is the mac
const String   MQTT_PREFIX              = "ventilation/";       // include tailing '/' in prefix
const String   MQTT_ONLINE              = "/online";      

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6]  = "1883";
char mqtt_user[40];
char mqtt_pass[40];

char charGmtOffset_sec[6] = "0";
char charManufacturer[40];
String strManufacturer;
char charAutodiscover[40];
String strAutodiscover;
char charModbusSlaveID[6];


// Ticker
Ticker ticker;

// Modbus
ModbusMaster node;
bool lastModbusStatus;


// Json
String req[4]; //operation, group, address, value

// Other
static uint16_t SENDINTERVAL = 5000; // normally set to 180000 milliseconds = 3 minutes. Define as you like
static long lastMsg = -SENDINTERVAL;

String Groups[10];

/*------------------------------------------------------------------------------ 
VARIABLES - Genvex
------------------------------------------------------------------------------*/
#define GenvexHOST "GenvexGW-%s" // Change this to whatever you like.
static uint16_t Genvexrsbuffer[12];
static uint16_t GenvexVENTSET = 100;
static uint16_t GenvexTIMER = 106;
static uint16_t GenvexTEMPSET = 000;
String GenvexGroups[] = {"temp", "runinfo", "temptarget", "speed", "time", "humidity", "version"};
byte GenvexRegsizes[] = {10, 10, 1, 7, 6, 12, 6};
int GenvexRegaddresses[] = {000, 100, 000, 100, 200, 000, 200};
byte GenvexRegtypes[] = {8, 0, 1, 1, 1, 8, 0};
char *GenvexRegnames[][12] = {
    //temp
    {"T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8", "T9", "T2_Panel"},
    //alarm
    {NULL, "Alarm", "Inlet_Fan", "Extract_Fan", "Bypass", "Watervalve", "Humidity_Fan_Control", "Bypass_On_Off", "Inletfan_rpm", "Extractfan_rpm"},
    //temp
    {"TempTarget"},
    //speed
    {"SpeedMode", NULL, "Heat", NULL, NULL, NULL, "Timer"},
    //time
    {"Hour", "Minute", "Day", "Date", "Month", "Year"},
    //humidity
    {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, "Measured_Humidity", "Humidity_Calculated_Setpoint"},
    //version
    {"Programme_Version_Controller", "Program_Version_Display", "Sub_Version_Controller", "Sub_Version_Display", "ModbusVersion", "Sub_ModbusVersion"}
};

/*------------------------------------------------------------------------------ 
VARIABLES - NILAN
------------------------------------------------------------------------------*/
#define NilanHOST "NilanGW-%s"
static uint16_t Nilanrsbuffer[28];
static uint16_t NilanVENTSET = 1003;
static uint16_t NilanRUNSET = 1001;
static uint16_t NilanMODESET = 1002;
static uint16_t NilanTEMPSET = 1004;
//static uint16_t  NilanPROGRAMSET = 500;
static uint16_t  NilanSELECTSET = 500;
static uint16_t  NilanUSERFUNCSET = 601;
static uint16_t  NilanUSERVENTSET = 603;
static uint16_t  NilanUSERTIMESET = 602;
static uint16_t  NilanUSERTEMPSET = 604;
static uint16_t  NilanTEMPSET_T11 = 1700;
static uint16_t  NilanTEMPSET_T12 = 1701;
String NilanGroups[] = {"temp", "alarm", "time", "control", "speed", "airtemp", "airflow", "airheat", "program", "user", "user2", "actstate", "info", "inputairtemp", "hotwater", "app", "output", "display1", "display2", "display"};
byte NilanRegsizes[] = {23, 10, 6, 8, 2, 6, 2, 0, 1, 6, 6, 4, 14, 7, 2, 4, 28, 4, 4, 1};
int NilanRegaddresses[] = {200, 400, 300, 1000, 200, 1200, 1100, 0, 500, 600, 610, 1000, 100, 1200, 1700, 0, 100, 2002, 2007, 3000};
byte NilanRegtypes[] = {8, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 8, 1, 2, 1, 4, 4, 8};
char *NilanRegnames[][28] = {
    //temp
    {"T0_Controller", "T1_Intake", "T2_Inlet", "T3_Exhaust", "T4_Outlet", "T5_Cond", "T6_Evap", "T7_Inlet", "T8_Outdoor", "T9_Heater", "T10_Extern", "T11_Top", "T12_Bottom", "T13_Return", "T14_Supply", "T15_Room", "T16", "T17_PreHeat", "T18_PresPibe", "pSuc", "pDis", "RH", "CO2"},
    //alarm
    {"Status", "List_1_ID ", "List_1_Date", "List_1_Time", "List_2_ID ", "List_2_Date", "List_2_Time", "List_3_ID ", "List_3_Date", "List_3_Time"},
    //time
    {"Second", "Minute", "Hour", "Day", "Month", "Year"},
    //control
    {"Type", "RunSet", "ModeSet", "VentSet", "TempSet", "ServiceMode", "ServicePct", "Preset"},
    //speed
    {"ExhaustSpeed", "InletSpeed"},
    //airtemp
    {"CoolSet", "TempMinSum", "TempMinWin", "TempMaxSum", "TempMaxWin", "TempSummer"},
    //airflow
    {"AirExchMode", "CoolVent"},
    //airheat
    {},
     //program
    {"Selectset"},
    //program.user
    {"UserFuncAct", "UserFuncSet", "UserTimeSet", "UserVentSet", "UserTempSet", "UserOffsSet"},
    //program.user2
    {"User2FuncAct", "User2FuncSet", "User2TimeSet", "User2VentSet", "UserTempSet", "UserOffsSet"},
    //actstate
    {"RunAct", "ModeAct", "State", "SecInState"},
    //info
    {"UserFunc", "AirFilter", "DoorOpen", "Smoke", "MotorThermo", "Frost_overht", "AirFlow", "P_Hi", "P_Lo", "Boil", "3WayPos", "DefrostHG", "Defrost", "UserFunc_2"},
    //inputairtemp
    {"IsSummer", "TempInletSet", "TempControl", "TempRoom", "EffPct", "CapSet", "CapAct"},
    //hotwatertemp
    {"TempSet_T11", "TempSet_T12"},
    //app
    {"Bus.Version", "VersionMajor", "VersionMinor", "VersionRelease"},
    //output
    {"AirFlap", "SmokeFlap", "BypassOpen", "BypassClose", "AirCircPump", "AirHeatAllow", "AirHeat_1", "AirHeat_2", "AirHeat_3", "Compressor", "Compressor_2", "4WayCool", "HotGasHeat", "HotGasCool", "CondOpen", "CondClose", "WaterHeat", "3WayValve", "CenCircPump", "CenHeat_1", "CenHeat_2", "CenHeat_3", "CenHeatExt", "UserFunc", "UserFunc_2", "Defrosting", "AlarmRelay", "PreHeat"},
    //display1
    {"Text_1_2", "Text_3_4", "Text_5_6", "Text_7_8"},
    //display2
    {"Text_9_10", "Text_11_12", "Text_13_14", "Text_15_16"},
    //airbypass
    {"AirBypass/IsOpen"}
  };


 
/*------------------------------------------------------------------------------ 
FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------ 
FUNCTIONS - loop() helper functions
------------------------------------------------------------------------------*/

char *getName(int type, int address)
{
  if(strManufacturer == "genvex")
  {
    if (address >= 0 && address <= GenvexRegsizes[type])
    {
      return GenvexRegnames[type][address];
    }
    return NULL;
  }
  if(strManufacturer == "nilan")
  {
    if (address >= 0 && address <= NilanRegsizes[type])
    {
      return NilanRegnames[type][address];
    }
    return NULL;    
  }
}

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const String currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}


/*------------------------------------------------------------------------------ 
FUNCTIONS - setup() helper functions - SPIFFS, WifiManager PubSubClient, Ticker
------------------------------------------------------------------------------*/


// TICKER SETUP - USED FOR SHOWING IF ESP IS IN CONFIG MODE OR NOT
void tick()
{
  //toggle state
  digitalWrite(26, !digitalRead(26));     // set pin to the opposite state
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  //Serial.println("Entered config mode");
  //Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  //Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}


//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
  ticker.attach(0.6, tick);
}

void setupSpiffs()
{
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        //json.printTo(Serial);
        serializeJson(json, Serial);
        //if (json.success()) {
        if (!deserializeError)
        {
          Serial.println("\nparsed json");

          if(!json["mqtt_server"].isNull())
          {
            strcpy(mqtt_server, json["mqtt_server"]);
          }
          if(!json["mqtt_port"].isNull())
          {
            strcpy(mqtt_port, json["mqtt_port"]);
          }
          if(!json["mqtt_user"].isNull())
          {
            strcpy(mqtt_user, json["mqtt_user"]);
          }
          if(!json["mqtt_pass"].isNull())
          {
            strcpy(mqtt_pass, json["mqtt_pass"]);
          }
          if(!json["GmtOffset_sec"].isNull())
          {
            strcpy(charGmtOffset_sec, json["GmtOffset_sec"]);
          }
          if(!json["charManufacturer"].isNull())
          {
            strcpy(charManufacturer, json["charManufacturer"]);
          }
          if(!json["autodiscover"].isNull())
          {            
            strcpy(charAutodiscover, json["autodiscover"]);
          }
          if(!json["modbusSlaveID"].isNull())
          {
            strcpy(charModbusSlaveID, json["modbusSlaveID"]);
          }
          

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
    SPIFFS.format();
  }
  //end read
}

void prepareModbus()
{
  Serial.println("Preparing Modbus");
  Serial.println(atoi(charModbusSlaveID));
  Serial2.begin(19200, SERIAL_8E1);
  node.begin(atoi(charModbusSlaveID), Serial2);
  node.clearResponseBuffer();
  node.clearTransmitBuffer();
}

void setBootTime(){
  struct tm timeinfo;
  
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    //return;
  }
  bootTime = String(timeinfo.tm_mday) + "/" + (1 + timeinfo.tm_mon) + "/" + (1900 + timeinfo.tm_year) + " - " + timeinfo.tm_hour + ":" + timeinfo.tm_min + ":" + timeinfo.tm_sec;
}


/*---------------------------------------------------------------------------------------------------
FUNCTIONS - Functions related to MODBUS
---------------------------------------------------------------------------------------------------*/

// Modbus
char ReadModbus(uint16_t addr, uint8_t sizer, uint16_t *vals, int type)
{
  char result = 0;
  switch (type)
  {
  case 0:
    result = node.readInputRegisters(addr, sizer);
    break;
  case 1:
    result = node.readHoldingRegisters(addr, sizer);
    break;
  }
  if (result == node.ku8MBSuccess)
  {
    for (int j = 0; j < sizer; j++)
    {
      vals[j] = node.getResponseBuffer(j);
    }
    return result;
  }
  else
  {
  Serial.println("READ ku8MB bad");
  node.clearResponseBuffer();
  node.clearTransmitBuffer();
  }
  
  return result;
}
char WriteModbus(uint16_t addr, uint16_t val)
{
  char result = 0;
  if(strManufacturer == "genvex")
  {
    node.setTransmitBuffer(0, val);

    result = node.writeSingleRegister(addr, val);
    if (result == node.ku8MBSuccess)
    {
      Serial.println("Write OK");
    }
    else
    {
      Serial.println("Write NOT OK");
      Serial.println("Clear bufs");
      node.clearResponseBuffer();
      node.clearTransmitBuffer();
    }
  }
  if(strManufacturer == "nilan")
  {
    Serial.println("Nilan - WriteModbus");
    node.setTransmitBuffer(0, val);
    char result = 0;
    result = node.writeMultipleRegisters(addr, 1);
  }
  return result;
} 

/*------------------------------------------------------------------------------ 
FUNCTIONS - Json requests
------------------------------------------------------------------------------*/

JsonObject HandleRequest(JsonDocument& doc)
{
  JsonObject root = doc.to<JsonObject>();
  int r = (sizeof(GenvexGroups)/sizeof(GenvexGroups[0]));
  
  char type = 0;
  if (req[1] != "")
  {
    for (int i = 0; i < (sizeof(GenvexGroups)/sizeof(GenvexGroups[0])); i++)
    {
      if (GenvexGroups[i] == req[1])
      {
        r = i;
      }
    }
  }
  type = GenvexRegtypes[r];
  if (req[0] == "read")
  {
    int address = 0;
    int nums = 0;
    char result = -1;
    address = GenvexRegaddresses[r];
    nums = GenvexRegsizes[r];
 
    result = ReadModbus(address, nums, Genvexrsbuffer, type & 1);
    if (result == 0)
    {
      root["status"] = "Modbus connection OK";
      for (int i = 0; i < nums; i++)
      {
        char *name = getName(r, i);
        if (name != NULL && strlen(name) > 0)
        {
          if ((type & 2 && i > 0) || type & 4)
          {
            String str = "";
            str += (char)(Genvexrsbuffer[i] & 0x00ff);
            str += (char)(Genvexrsbuffer[i] >> 8);
            root[name] = str;
          }
          else if (type & 8)
          {
            root[name] = (Genvexrsbuffer[i] - 300.0) /10.0;
          }
          else
          {
            root[name] = Genvexrsbuffer[i];
          }
        }
      }
    }
    else {
      root["status"] = "Modbus connection failed";
    }
    root["requestaddress"] = address;
    root["requestnum"] = nums;
  }

  if (req[0] == "set" && req[1] == "temp" && req[2] != "" && req[3] != "")
  {
    int address = atoi(req[2].c_str());
    int value = (atoi(req[3].c_str())) - 100.0;
    char result = WriteModbus(address, value);
    root["result"] = result;
    root["address"] = address;
    root["value"] = value;
  }
  else
  {
  if (req[0] == "set" && req[2] != "" && req[3] != "")
  {
    int address = atoi(req[2].c_str());
    int value = atoi(req[3].c_str());
    char result = WriteModbus(address, value);
    root["result"] = result;
    root["address"] = address;
    root["value"] = value;
  }
  }
  if (req[0] == "help")
  {
    for (int i = 0; i < (sizeof(GenvexGroups)/sizeof(GenvexGroups[0])); i++)
    {
      root[GenvexGroups[i]] = 0;
    }
  }
  root["operation"] = req[0];
  root["group"] = req[1];
  return root;
}



/*---------------------------------------------------------------------------------------------------
FUNCTIONS - Functions related to MQTT
---------------------------------------------------------------------------------------------------*/

// MQTT callback function
void mqttcallback(char *topic, byte *payload, unsigned int length)
{
  if(strManufacturer = "genvex")
  {
    Serial.println("in Callback");

    // Convert payload to string for use with State topic in HA
    char terminatedPayload[length+1];
    for(unsigned int i=0; i<length; i++)
    {
      terminatedPayload[i] = payload[i];
    }
    terminatedPayload[length] = 0;
    String payloadString = String(terminatedPayload);

    if (strcmp(topic, "ventilation/control/setSpeedMode") == 0)
    {
      if ((length == 1 && payload[0] >= '0' && payload[0] <= '4'))
      {
        uint16_t speed = payload[0] - '0';
        WriteModbus(GenvexVENTSET, speed);
      }
      if(payloadString == "off") // If payload is off, set speed to 0
      {
        WriteModbus(GenvexVENTSET, 0);
      }
      if(payloadString == "auto") // If pauload is auto, set speed to 2
      {
        WriteModbus(GenvexVENTSET, 2);
      }      
    }
    if (strcmp(topic, "ventilation/control/setTimer") == 0)
    {
      if (length == 1 && payload[0] >= '0' && payload[0] <= '3')
      {
        uint16_t mode = payload[0] - '0';
        WriteModbus(GenvexTIMER, mode);
      }
    }
    if (strcmp(topic, "ventilation/control/setTempTarget") == 0)
    {
      if (length >= 3 && payload[0] >= '0' && payload[0] <= '200')
      {
        String str;
        for (int i = 0; i < length; i++)
        {
          if(isdigit((char)payload[i]))
          {
            str += (char)payload[i];
          }
        }      
        int value = ((str.toInt()) - 100.0);
        WriteModbus(GenvexTEMPSET, value);
      }
    }
  }
  if(strManufacturer == "nilan")
  {
    Serial.println("Nilan - mqttcallback");
    if (strcmp(topic, "ventilation/ventset") == 0)
    {
      if (length == 1 && payload[0] >= '0' && payload[0] <= '4')
      {
        int16_t speed = payload[0] - '0';
        WriteModbus(NilanVENTSET, speed);
      }
    }
    if (strcmp(topic, "ventilation/modeset") == 0)
    {
      if (length == 1 && payload[0] >= '0' && payload[0] <= '4')
      {
        int16_t mode = payload[0] - '0';
        WriteModbus(NilanMODESET, mode);
      }
    }
    if (strcmp(topic, "ventilation/runset") == 0)
    {
      if (length == 1 && payload[0] >= '0' && payload[0] <= '1')
      {
        int16_t run = payload[0] - '0';
        WriteModbus(NilanRUNSET, run);
      }
    }
     if (strcmp(topic, "ventilation/userset") == 0)
    {
      if (payload[0] == '1')
      {
        digitalWrite(4, HIGH);
        digitalWrite(25, HIGH);
        mqttclient.publish("ventilation/userset", "on");
       } 
        else if (payload[0] == '0')
       {  
        digitalWrite(4, LOW);
        digitalWrite(25, LOW);
        mqttclient.publish("ventilation/userset", "off");
      }
    }
    if (strcmp(topic, "ventilation/userfuncset") == 0)
    {
      if (length == 1 && payload[0] >= '0' && payload[0] <= '4')
      {
        int16_t select = payload[0] - '0';
        WriteModbus(NilanUSERFUNCSET, select);
      }
    } 
    if (strcmp(topic, "ventilation/userventset") == 0)
      {
      if (length == 1 && payload[0] >= '0' && payload[0] <= '4')
      {
        int16_t vent = payload[0] - '0';
        WriteModbus(NilanUSERVENTSET, vent);
      }
    }
    if (strcmp(topic, "ventilation/usertimeset") == 0)
    {
      if (length == 3 && payload[0] >= '0' && payload[0] <= '3')
      {
        int16_t period = payload[0] - '0';
        WriteModbus(NilanUSERTIMESET, period);
      }
    }
     if (strcmp(topic, "ventilation/usertempset") == 0)
    {
      if (length == 2 && payload[0] >= '0' && payload[0] <= '2')
      {
        String str;
        for (int i = 0; i < length; i++)
        {
          str += (char)payload[i];
        }
        WriteModbus(NilanUSERTEMPSET, str.toInt());
      }
    }
    if (strcmp(topic, "ventilation/tempset") == 0)
    {
      if (length == 4 && payload[0] >= '0' && payload[0] <= '2')
      {
        String str;
        for (int i = 0; i < length; i++)
        {
          str += (char)payload[i];
        }
        WriteModbus(NilanTEMPSET, str.toInt());
      }
    }
    if (strcmp(topic, "ventilation/tempset_T11") == 0)
    {
      if (length == 4 && payload[0] >= '0' && payload[0] <= '2')
      {
        String str;
        for (int i = 0; i < length; i++)
        {
          str += (char)payload[i];
        }
        WriteModbus(NilanTEMPSET_T11, str.toInt());
      }
    }
    if (strcmp(topic, "ventilation/tempset_T12") == 0)
    {
      if (length == 4 && payload[0] >= '0' && payload[0] <= '2')
      {
        String str;
        for (int i = 0; i < length; i++)
        {
          str += (char)payload[i];
        }
        WriteModbus(NilanTEMPSET_T12, str.toInt());
      }
    }
    if (strcmp(topic, "ventilation/selectset") == 0)
    {
      if (length == 1 && payload[0] >= '0' && payload[0] <= '4')
      {
        int16_t select = payload[0] - '0';
        WriteModbus(NilanSELECTSET, select);
      }
    }  
  }
  lastMsg = -SENDINTERVAL;  
}

// Publish discovery messages for HomeAssistant
// See https://www.home-assistant.io/docs/mqtt/discovery/
void publishConfiguration(String mqname, char *name)//, String unit)//, int i)
{
  if(strManufacturer == "genvex")
  {
    Serial.println("");
    Serial.println("-------------- Publish Config start --------------");
    String tempTopic = "";
    if(!strstr(name,"On_Off"))
    {
      tempTopic = String("homeassistant/sensor/" + strManufacturer + "/" + name + "/config");
    }
    else
    {
      if(strstr(name,"On_Off"))
      {
        tempTopic = String("homeassistant/binary_sensor/" + strManufacturer + "/" + name + "/config");
      }
    }
    
    

    String tempMessage = String(
        "{\"name\": \"" + strManufacturer + "_" + name + "\", "
        "\"state_topic\": \"" + mqname + "\", " 
        "\"availability_topic\": \"" + MQTT_PREFIX + strManufacturer + MQTT_ONLINE +"\", "
        "\"payload_available\": \"True\", "
        "\"payload_not_available\": \"False\", ");
  
    // Add unit if temp
    if(mqname.startsWith("ventilation/temp"))
    {
      tempMessage += ("\"unit_of_measurement\": \"°C\", ");
      tempMessage += ("\"device_class\": \"temperature\", ");
    }
    //String strName = String(name);
    if(((strstr(name,"Inlet_Fan")) || (strstr(name,"Extract_Fan")) || (strstr(name,"Bypass")) || (strstr(name,"Humidity")) || (strstr(name,"Watervalve"))) && (!strstr(name,"On_Off")))
    {
      Serial.print("in if");
      tempMessage += ("\"unit_of_measurement\": \"%\", ");
    }

    if(strstr(name,"Alarm"))
    {
      //tempMessage += ("\"unit_of_measurement\": \"°C\", ");
      //tempMessage += ("\"device_class\": \"temperature\", ");
    }

    if(strstr(name,"On_Off"))
    {
      tempMessage += ("\"payload_on\": \"1\", ");
      tempMessage += ("\"payload_off\": \"0\", ");
    }

    // Add final line of config
    tempMessage += ("\"qos\": \"0\"}");
    // Publish the config
    Serial.println("topic");
    Serial.println(tempTopic);
    Serial.println("message");
    Serial.println(tempMessage);
    Serial.println(mqttclient.publish(tempTopic.c_str(), tempMessage.c_str(), true));
    Serial.println("-------------- Publish Config End --------------");
  }
  if(strManufacturer = "nilan")
  {}
  sleep(1); // Need to sleep to give HA a chance to create entities before published values (To prevent unknown state)
}

void publishClimateConfig()
{  // Publish climate entity config
  if(strManufacturer == "genvex")
  {
    Serial.println("");
    Serial.println("-------------- Publish Climate --------------");

    String climateTopic = String("homeassistant/climate/ventilation/" + strManufacturer + "/" + "config");

    String climateMessage = String(
        "{\"name\": \"" + strManufacturer + "_" + "climate" + "\", "
        "\"current_temperature_topic\": \"" + MQTT_PREFIX + "temp/T1" + "\", "
        "\"min_temp\": \"16\"," 
        "\"max_temp\": \"28\","
        "\"mode_state_topic\": \"" + MQTT_PREFIX + "speed/SpeedMode" + "\", "
        "\"mode_state_template\": \"{% set modes = {'1':'auto', '2':'auto', '3':'auto', '4':'auto'} %} {{ modes[value] if value in modes.keys() else 'off' }}\","
        "\"mode_command_topic\": \"ventilation/control/setSpeedMode\","
        "\"modes\": [\"" + "off" + "\", \"" + "auto" + "\"],"
        "\"temperature_command_topic\": \"" + MQTT_PREFIX + "control/setTempTarget" + "\", "
        "\"temperature_state_topic\": \"" + MQTT_PREFIX + "temp/TempTarget" + "\", "
        "\"fan_mode_command_topic\": \"" + MQTT_PREFIX + "control/setSpeedMode" + "\", "
        "\"fan_mode_state_topic\": \"" + MQTT_PREFIX + "speed/SpeedMode" + "\", "
        "\"fan_modes\": [\"" + "1" + "\", \"" + "2" + "\", \"" + "3" + "\", \"" + "4" + "\"],"
        "\"availability_topic\": \"" + MQTT_PREFIX + strManufacturer + MQTT_ONLINE +"\", "
        "\"payload_available\": \"True\", "
        "\"payload_not_available\": \"False\", "
        "\"qos\": \"0\"}");

    Serial.println("topic");
    Serial.println(climateTopic);
    Serial.println("message");
    Serial.println(climateMessage);
    Serial.println(mqttclient.publish(climateTopic.c_str(), climateMessage.c_str(), true));
    Serial.println("-------------- End Climate --------------");
    sleep(1); // HA delay.........
  }
}

// Publish values to MQTT
void publisToMQTT(int rrint, int iRegSize, char numstr[8], String mqname)
{
  /*---------------------------------------------------------------------------------------------------
  Logic to determine if a publish i needed. Only publish if value has changed since last Modbus read.
  ---------------------------------------------------------------------------------------------------*/

  // Combining reqtype iterator and regsize iterator to generate unique index value for lastReadValue array.
  String lastReadValueIndex = (String)rrint;
  lastReadValueIndex += (String)iRegSize;

  if(!((lastReadValues[(lastReadValueIndex.toInt())]).equals((String)numstr)) || firstLoop)//((lastReadValues[(lastReadValueIndex.toInt())]).isEmpty())) //If numstr is NOT in lastReadValues
  {
    mqttclient.publish(mqname.c_str(), numstr); // Publish new value to mqtt
    lastReadValues[(lastReadValueIndex.toInt())] = (String)numstr; // Add/update the value in lastReadValues with the unique lastReadValueIndex
  }
}

// Text publish
void publisToMQTT(int rrint, int iRegSize, String text, String mqname)
{
  /*---------------------------------------------------------------------------------------------------
  Logic to determine if a publish i needed. Only publish if value has changed since last Modbus read.
  ---------------------------------------------------------------------------------------------------*/

  // Combining reqtype iterator and regsize iterator to generate unique index value for lastReadValue array.
  String lastReadValueIndex = (String)rrint;
  lastReadValueIndex += (String)iRegSize;

  if(!((lastReadValues[(lastReadValueIndex.toInt())]).equals(text)) || firstLoop)//((lastReadValues[(lastReadValueIndex.toInt())]).isEmpty())) //If numstr is NOT in lastReadValues
  {
    mqttclient.publish(mqname.c_str(), text.c_str()); // Publish new value to mqtt
    lastReadValues[(lastReadValueIndex.toInt())] = text; // Add/update the value in lastReadValues with the unique lastReadValueIndex
  }
}

// Publish config topic to MQTT
void publishConfigToMQTT(int rrint, String mqname, char *name, int iRegSize, int r)
{
  /*-------------------------------------------------------------------------------------------------
    Logic to determine if a config topic for current register has been published, if not, publish it.
  -------------------------------------------------------------------------------------------------*/

  // Check if reqtypeiterator is already in configurationPublished bool array, if not, publish config topic
  if(!configurationPublished[rrint])
  {
    publishConfiguration(mqname, name);

    // Check if it's the last register in regSize, if true, set configurationPublished to true for reqtype index
    if(strManufacturer = "genvex")
    {
      if(iRegSize == ((GenvexRegsizes[r])-1))
      {
        configurationPublished[rrint] = true;
      }                  
    }
    if(strManufacturer == "nilan")
    {
      Serial.println("Nilan - publishConfigToMQTT");
      if(iRegSize == ((NilanRegsizes[r])-1))
      {
        configurationPublished[rrint] = true;
      }  
    }
  }
}


/*---------------------------------------------------------------------------------------------------
FUNCTIONS - Standard functions like setup() and loop()
---------------------------------------------------------------------------------------------------*/

void setup() 
{
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  Serial.begin(115200);
  Serial.println();
  uint8_t chipid[6];
  esp_efuse_read_mac(chipid);
  Serial.printf("%X\n",chipid);
  ArduinoOTA.setHostname("Genvex");
  setupSpiffs();

  // WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wm;
  ticker.attach(0.2, tick);
  //set config save notify callback
  wm.setSaveConfigCallback(saveConfigCallback);

  // setup custom parameters
  // 
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "MQTT server IP", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT server port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "MQTT user", mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_pass("pass", "MQTT pass", mqtt_pass, 40);
  WiFiManagerParameter custom_GmtOffset_sec("gmtOffset_sec", "GMT Timezone offset (For Denmark(+1h) set to \"3600\")", charGmtOffset_sec, 6);
  WiFiManagerParameter custom_manufacturer("manufacturer", "Manufacturer (Valid manufactures: \"Genvex\" or \"Nilan\")", charManufacturer, 40);
  WiFiManagerParameter custom_autodiscover("autodiscover", "Publish autodiscover config (0 disabled - 1 enabled)", charAutodiscover, 40);
  WiFiManagerParameter custom_modbusSlaveID("modbusSlaveID", "Modbus Slave ID (Genvex default is 1, Nilan default is 30)", charModbusSlaveID, 6);

  //add all your parameters here
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_pass);
  wm.addParameter(&custom_GmtOffset_sec);
  wm.addParameter(&custom_manufacturer);  
  wm.addParameter(&custom_autodiscover);  
  wm.addParameter(&custom_modbusSlaveID);


  // set static ip
  // IPAddress _ip,_gw,_sn;
  // _ip.fromString(static_ip);
  // _gw.fromString(static_gw);
  // _sn.fromString(static_sn);
  // wm.setSTAStaticIPConfig(_ip, _gw, _sn);

  //reset settings - wipe credentials for testing
  //wm.resetSettings();

  //automatically connect using saved credentials if they exist
  //If connection fails it starts an access point with the specified name
  //here  "AutoConnectAP" if empty will auto generate basedcon chipid, if password is blank it will be anonymous
  //and goes into a blocking loop awaiting configuration
  if (!wm.autoConnect("VentAP")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    // if we still have not connected restart and try all over again
    ESP.restart();
    delay(5000);
 
  }
  // always start configportal for a little while
  wm.setConfigPortalTimeout(60);
  wm.startConfigPortal("VentAP");

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  ticker.detach(); 
  digitalWrite(26, 1);
  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  strcpy(charGmtOffset_sec, custom_GmtOffset_sec.getValue());
  strcpy(charManufacturer, custom_manufacturer.getValue());
  strcpy(charAutodiscover, custom_autodiscover.getValue());
  strcpy(charModbusSlaveID, custom_modbusSlaveID.getValue());
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonDocument json(1024);
    //JsonObject json = jsonDocument.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"]   = mqtt_port;
    json["mqtt_user"]   = mqtt_user;
    json["mqtt_pass"]   = mqtt_pass;
    json["GmtOffset_sec"] = charGmtOffset_sec;
    json["charManufacturer"]        = charManufacturer;
    json["autodiscover"] = charAutodiscover;
    json["modbusSlaveID"] = charModbusSlaveID;

     json["ip"]          = WiFi.localIP().toString();
     json["gateway"]     = WiFi.gatewayIP().toString();
     json["subnet"]      = WiFi.subnetMask().toString();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    //json.printTo(Serial);
    serializeJson(json, Serial);
    //json.printTo(configFile);
    serializeJson(json, configFile);
    configFile.close();
    //end save
    shouldSaveConfig = false;
  }
   ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  ArduinoOTA.setTimeout(20000);
  ArduinoOTA.begin();

  // Converting charManufacturer to String and save in strManufacturer (and convert to lower case)
  strManufacturer = (String)charManufacturer;
  strManufacturer.toLowerCase();

  // Prepare Modbus
  prepareModbus();

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.subnetMask());
  
  
  Serial.println("DBG - serverbegin");
  server.begin();
  Serial.println("DBG - mqtt set server is next");

  // Configuring mqttclient (PubSubClient)
  mqttclient.setServer(mqtt_server, (uint16_t)atoi(mqtt_port));
  // Making sure the buffer is large enough for the MQTT discovery configurations
  mqttclient.setBufferSize(1024);
  mqttclient.setCallback(mqttcallback);

  sleep(5); 
  
  configTime((atol(charGmtOffset_sec)), daylightOffset_sec, ntpServer);
  
}


bool readRequest(WiFiClient &client)
{
  req[0] = "";
  req[1] = "";
  req[2] = "";
  req[3] = "";
 
  int n = -1;
  bool readstring = false;
  while (client.connected())
  {
    if (client.available())
    {
      char c = client.read();
      if (c == '\n')
      {
        return false;
      }
      else if (c == '/')
      {
        n++;
      }
      else if (c != ' ' && n >= 0 && n < 4)
      {
        req[n] += c;
      }
      else if (c == ' ' && n >= 0 && n < 4)
      {
        return true;
      }
    }
  }
 
  return false;
}
 
void writeResponse(WiFiClient& client, const JsonDocument& doc)  
{
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();
  serializeJsonPretty(doc,client);
}
 
 
void mqttreconnect()
{
  int numretries = 0;
  while (!mqttclient.connected() && numretries < 3)
  {
    if (mqttclient.connect(chipid, mqtt_user, mqtt_pass))
    {
      digitalWrite(14, 1);

      // Genvex
      if(strManufacturer == "genvex")
      {
        mqttclient.subscribe("ventilation/control/setSpeedMode");
        mqttclient.subscribe("ventilation/control/setTimer");
        mqttclient.subscribe("ventilation/control/setTempTarget");
      }
      if(strManufacturer == "nilan")
      {
        Serial.println("Nilan - mqttreconnect");
        mqttclient.subscribe("ventilation/ventset");
        mqttclient.subscribe("ventilation/modeset");
        mqttclient.subscribe("ventilation/runset");
        mqttclient.subscribe("ventilation/tempset");
        mqttclient.subscribe("ventilation/selectset");
        mqttclient.subscribe("ventilation/tempset_T11");
        mqttclient.subscribe("ventilation/tempset_T12");
        mqttclient.subscribe("ventilation/userset");
        mqttclient.subscribe("ventilation/userfuncset");
        mqttclient.subscribe("ventilation/userventset");
        mqttclient.subscribe("ventilation/usertimeset");
        mqttclient.subscribe("ventilation/usertempset");
      }
      // Publish availability topic - True
      String will = String(MQTT_PREFIX + strManufacturer + MQTT_ONLINE);
      mqttclient.publish(will.c_str(), (const uint8_t *)"True", 4, true);
    }
    else
    {
      digitalWrite(14, 0);
      delay(1000);
    }
    numretries++;
  }
}

void loop()
{
  ArduinoOTA.handle();
  WiFiClient client = server.available();
  if (client)
  {
    bool success = readRequest(client);
    if (success)
    {
      StaticJsonDocument<2000> doc;
      HandleRequest(doc);
 
      writeResponse(client, doc);
    }
    client.stop();
  }
 
  if (!mqttclient.connected())
  {
    mqttreconnect();
  }
 
  if (mqttclient.connected())
  {
    
    mqttclient.loop();
    long now = millis();
    if (now - lastMsg > SENDINTERVAL)
    {
      if(firstLoop)
      {
        
        //timeClient.update();
        //Serial.println(timeClient.getFormattedDate());
        // Not sure why this is needed, but if it's not there it's not getting data to Home-Assistant when MQTT Discovery is enabled
        Serial.println("Sleeping.............");
        sleep(5);
        setBootTime();
      }
      
      // Iterating over the reqtypes
      if(strManufacturer == "genvex")
      {
        for (int rrint = 0; rrint < (sizeof(GenvexGroups)/sizeof(GenvexGroups[0])); rrint++)
        {
          String currentGroup = GenvexGroups[rrint];

          char result = ReadModbus(GenvexRegaddresses[rrint], GenvexRegsizes[rrint], Genvexrsbuffer, GenvexRegtypes[rrint] & 1);
          
          if (result == 0) // If ModBus read had no errors
          {

            /*---------------------------------------------------------------------------------------------------
              Logic to determine if modbus read was ok and publish error and online topic
            ---------------------------------------------------------------------------------------------------*/
          
            // Check the lastModBusReadSttus, if not true go ahead and publis error and online with "positive" values (0 and True)
            if(!lastModbusStatus)
            {
              // Publish error topic - 0
              mqttclient.publish("ventilation/error/modbus/", "0"); //no error when connecting through modbus
              lastModbusStatus = true;

              // Publish availability topic - True
              String will = String(MQTT_PREFIX + strManufacturer + MQTT_ONLINE);
              mqttclient.publish(will.c_str(), (const uint8_t *)"True", 4, true);
              // Publish availability time
              will = String(MQTT_PREFIX + strManufacturer + "/LastBootTime");
              mqttclient.publish(will.c_str(), bootTime.c_str(), true);
              
            } 



            // Iterating through the regsize for the reqtype
            for (int iRegSize = 0; iRegSize < GenvexRegsizes[rrint]; iRegSize++)
            {
              String unit = "";
              char *name = getName(rrint, iRegSize);
              char numstr[8];
              if   (name != NULL && strlen(name) > 0)
              { 
                String mqname = "temp/";
                if(GenvexGroups[rrint] == "temptarget")
                {
                  mqname = "ventilation/temp/"; // Subscribe to the "tempcontrol" register
                  dtostrf((Genvexrsbuffer[iRegSize] + 100.0) /10, 5, 2, numstr);
                }
                else
                {               
                  if(GenvexGroups[rrint] == "temp")
                  {
                    mqname = "ventilation/temp/"; // Subscribe to "temp" register
                    dtostrf((Genvexrsbuffer[iRegSize] - 300.0) /10, 5, 2, numstr);
                  }
                  else
                  {
                    mqname = "ventilation/" + (GenvexGroups[rrint]) + "/";
                    itoa((Genvexrsbuffer[iRegSize]), numstr, 10);
                  }
                }

                mqname += (char *)name;
                

                if((bool)atoi(charAutodiscover))
                {
                  publishConfigToMQTT(rrint, mqname, name, iRegSize, rrint);
                }

                publisToMQTT(rrint, iRegSize, numstr, mqname);
              }
            }
          }
          else // If Modbus Read was not good
          {
            // Check if lastModbusStatus is NOT false, if true, publish error and online topic to bad values (1 and false)
            if(!(lastModbusStatus == false))
            {
              // Publish error topic - 1
              mqttclient.publish("ventilation/error/modbus/", "1"); //error when connecting through modbus
              lastModbusStatus = false;
              // Publish availability topic - False
              String will = String(MQTT_PREFIX + strManufacturer + MQTT_ONLINE);
              mqttclient.publish(will.c_str(), (const uint8_t *)"False", 5, true);
            }
          

          }      
        }
      }

      if(strManufacturer == "nilan")
      {
        Serial.println("Nilan - loop");
        // Handle text fields
        //reqtypes rr2[] = {}; // put another register in this line to subscribe
        for (int rrint = 0; rrint < (sizeof(NilanGroups)/sizeof(NilanGroups[0])); rrint++)
        {
          String currentGroup = NilanGroups[rrint];
          char result = ReadModbus(NilanRegaddresses[rrint], NilanRegsizes[rrint], Nilanrsbuffer, NilanRegtypes[rrint] & 1);
        
 
          if (result == 0) // If ModBus read had no errors
          {

            /*---------------------------------------------------------------------------------------------------
              Logic to determine if modbus read was ok and publish error and online topic
            ---------------------------------------------------------------------------------------------------*/
          
            // Check the lastModBusReadSttus, if not true go ahead and publis error and online with "positive" values (0 and True)
            if(!lastModbusStatus)
            {
              // Publish error topic - 0
              mqttclient.publish("ventilation/error/modbus/", "0"); //no error when connecting through modbus
              lastModbusStatus = true;

              // Publish availability topic - True
              String will = String(MQTT_PREFIX + strManufacturer + MQTT_ONLINE);
              mqttclient.publish(will.c_str(), (const uint8_t *)"True", 4, true);
            }

            if(currentGroup == "display1" || currentGroup == "display2")
            {
              String text = "";
              String mqname = "ventilation/text/";
              for (int iRegSize = 0; iRegSize < NilanRegsizes[rrint]; iRegSize++)
              {
                char *name = getName(rrint, iRegSize);
 
                if ((Nilanrsbuffer[iRegSize] & 0x00ff) == 0xDF) {
                  text += (char)0x20; // replace degree sign with space
                } else {
                  text += (char)(Nilanrsbuffer[iRegSize] & 0x00ff);
                }
                if ((Nilanrsbuffer[iRegSize] >> 8) == 0xDF) {
                  text += (char)0x20; // replace degree sign with space
                } else {
                  text += (char)(Nilanrsbuffer[iRegSize] >> 8);
                }
              
                mqname += (char *)name;
                
                if((bool)atoi(charAutodiscover))
                {
                  publishConfigToMQTT(rrint, mqname, name, iRegSize, rrint);
                }
                publisToMQTT(rrint, iRegSize, text, mqname);
              }
            }
            else
            {               
              // Iterating through the regsize for the reqtype
              for (int iRegSize = 0; iRegSize < NilanRegsizes[rrint]; iRegSize++)
              {
                String unit = "";
                char *name = getName(rrint, iRegSize);
                char numstr[8];
                if   (name != NULL && strlen(name) > 0)
                { 
                  String mqname = "temp/";             
                  if(NilanGroups[rrint] == "temp")
                  {
                    if (strncmp("RH", name, 2) == 0) {
                      mqname = "ventilation/moist/"; // Subscribe to moisture-level
                    } else {
                      mqname = "ventilation/temp/"; // Subscribe to "temp" register
                    }
                    dtostrf((Nilanrsbuffer[iRegSize] / 100.0), 5, 2, numstr);

                  }
                  else
                  {
                    mqname = "ventilation/" + (NilanGroups[rrint]) + "/";
                    itoa((Nilanrsbuffer[iRegSize]), numstr, 10);
                  }

                  mqname += (char *)name;
                

                  if((bool)atoi(charAutodiscover))
                  {
                    publishConfigToMQTT(rrint, mqname, name, iRegSize, rrint);
                  }

                  publisToMQTT(rrint, iRegSize, numstr, mqname);
                }
              }
            }
          }
          else // If Modbus Read was not good
          {
            // Check if lastModbusStatus is NOT false, if true, publish error and online topic to bad values (1 and false)
            if(!(lastModbusStatus == false))
            {
              // Publish error topic - 1
              mqttclient.publish("ventilation/error/modbus/", "1"); //error when connecting through modbus
              lastModbusStatus = false;
              // Publish availability topic - False
              String will = String(MQTT_PREFIX + strManufacturer + MQTT_ONLINE);
              mqttclient.publish(will.c_str(), (const uint8_t *)"False", 5, true);
            }
          } 
        }
      }
      lastMsg = now;
      if(firstLoop)
      {
        publishClimateConfig();
        firstLoop = false;
      }
    }
  }

 }  