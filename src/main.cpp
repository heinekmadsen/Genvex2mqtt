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
#define HOST "GenvexGW-%s" // Change this to whatever you like.
#define MAXREGSIZE 12
#define SENDINTERVAL 5000 // normally set to 180000 milliseconds = 3 minutes. Define as you like
#define VENTSET 100
#define RUNSET 024
#define MODESET 015
#define TEMPSET 000
#define PROGRAMSET 500
char chipid[12];
Ticker ticker;
WiFiServer server(80);
WiFiClient client;
PubSubClient mqttclient(client);
static long lastMsg = -SENDINTERVAL;
static uint16_t rsbuffer[MAXREGSIZE];
ModbusMaster node;
char* usersetTopic1 = "ventilation/userset"; 

String req[4]; //operation, group, address, value
enum reqtypes
{
  reqtemp = 0,
  reqruninfo,
  reqtempcontrol,
  reqspeed,
  reqtime,
  reqhumidity,
  reqversion,
  reqmax
};
 
//String groups[] = {"temp", "runinfo", "tempcontrol", "control", "speed", "airtemp", "airflow", "humidity", "program", "user", "user2", "info", "inputairtemp", "app", "output", "display1", "display2", "display"};
String groups[] = {"temp", "runinfo", "temptarget", "speed", "time", "humidity", "version"};
//byte regsizes[] = {10, 9, 1, 7, 9, 6, 2, 12, 1, 6, 6, 14, 7, 4, 26, 4, 4, 1};
byte regsizes[] = {10, 10, 1, 7, 6, 12, 6};
//int regaddresses[] = {000, 100, 000, 100, 100, 1200, 1100, 000, 500, 600, 610, 100, 1200, 0, 100, 2002, 2007, 3000};
int regaddresses[] = {000, 100, 000, 100, 200, 000, 200};
//byte regtypes[] = {8, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 2, 1, 4, 4, 8};
byte regtypes[] = {8, 0, 1, 1, 1, 1, 0};
// char *regnames[][MAXREGSIZE] = {
    // //temp
    // {"T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8", "T9", "T2_Panel"},
    // //alarm
    // {"Alarm", "Inlet_Fan", "Extract_Fan", "Bypass", "Watervalve", "Humidity_Fan_Control", "BTypass_On_Off", "Inletfan_rpm", "Extractfan_rpm"},
    // //tempcontrol
    // {"TempSet"},
    // //control
    // {"VentSet", NULL, "HeatOn", NULL, NULL, NULL, "Timer"},
    // //speed
    // {"ExhaustSpeed", "InletSpeed"},
    // //airtemp
    // {"CoolSet", "TempMinSum", "TempMinWin", "TempMaxSum", "TempMaxWin", "TempSummer"},
    // //airflow
    // {"AirExchMode", "CoolVent"},
    // //humidity
    // {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, "Measured_Humidity", "Humidity_Calculated_Setpoint"},
    // //program
    // {"Program"},
    // //program.user
    // {"UserFuncAct", "UserFuncSet", "UserTimeSet", "UserVentSet", "UserTempSet", "UserOffsSet"},
    // //program.user2
    // {"User2FuncAct", "User2FuncSet", "User2TimeSet", "User2VentSet", "UserTempSet", "UserOffsSet"},
    // //info
    // {"UserFunc", "AirFilter", "DoorOpen", "Smoke", "MotorThermo", "Frost_overht", "AirFlow", "P_Hi", "P_Lo", "Boil", "3WayPos", "DefrostHG", "Defrost", "UserFunc_2"},
    // //inputairtemp
    // {"IsSummer", "TempInletSet", "TempControl", "TempRoom", "EffPct", "CapSet", "CapAct"},
    // //app
    // {"Bus.Version", "VersionMajor", "VersionMinor", "VersionRelease"},
    // //output
    // {"AirFlap", "SmokeFlap", "BypassOpen", "BypassClose", "AirCircPump", "AirHeatAllow", "AirHeat_1", "AirHeat_2", "AirHeat_3", "Compressor", "Compressor_2", "4WayCool", "HotGasHeat", "HotGasCool", "CondOpen", "CondClose", "WaterHeat", "3WayValve", "CenCircPump", "CenHeat_1", "CenHeat_2", "CenHeat_3", "CenHeatExt", "UserFunc", "UserFunc_2", "Defrosting"},
    // //display1
    // {"Text_1_2", "Text_3_4", "Text_5_6", "Text_7_8"},
    // //display2
    // {"Text_9_10", "Text_11_12", "Text_13_14", "Text_15_16"},
    // //airbypass
    // {"AirBypass/IsOpen"}};

char *regnames[][MAXREGSIZE] = {
    //temp
    {"T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8", "T9", "T2_Panel"},
    //alarm
    {NULL, "Alarm", "Inlet_Fan", "Extract_Fan", "Bypass", "Watervalve", "Humidity_Fan_Control", "BTypass_On_Off", "Inletfan_rpm", "Extractfan_rpm"},
    //temp
    {"TempTarget"},
    //speed
    {"SpeedMode", NULL, "Heat", NULL, NULL, NULL, "Timer"},
    //time
    {"Hour", "Minute", "Day", "Date", "Date", "Month", "Year"},
    //humidity
    {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, "Measured_Humidity", "Humidity_Calculated_Setpoint"},
    //version
    {"Programme_Version_Controller", "Program_Version_Display", "Sub_Version_Controller", "Sub_Version_Display", "ModbusVersion", "Sub_ModbusVersion"}
};

   
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
  Serial.println("READ ku8MB bad on reg:");
  Serial.println("Type:");
  Serial.println(type);
  Serial.println("Addr:");
  Serial.println(addr);
  Serial.println("KU8:");
  Serial.println(node.ku8MBIllegalDataAddress);
  Serial.println(node.ku8MBIllegalDataValue);
  Serial.println(node.ku8MBIllegalFunction);
  Serial.println(node.ku8MBInvalidCRC);
  Serial.println(node.ku8MBInvalidFunction);
  Serial.println(node.ku8MBInvalidSlaveID);
  Serial.println(node.ku8MBResponseTimedOut);
  Serial.println(node.ku8MBSlaveDeviceFailure);
  Serial.println(node.ku8MBSuccess);
  node.clearResponseBuffer();
  node.clearTransmitBuffer();
  }
  
  return result;
}
char WriteModbus(uint16_t addr, uint16_t val)
{
  Serial.println("DBG - Write addr");
  Serial.println(addr);
  Serial.println("DBG - Write value");
  Serial.println(val);
  node.setTransmitBuffer(0, val);
  char result = 0;
  //result = node.writeMultipleRegisters(addr, 1);
  result = node.writeSingleRegister(addr, val);
  if (result == node.ku8MBSuccess)
  {
    Serial.println("Write OK");
  }
  else
  {
    Serial.println("Write NOT OK");
    Serial.println("Write ku8MB");
    Serial.println("Addr:");
    Serial.println(addr);
    Serial.println("KU8:");
    Serial.println(node.ku8MBIllegalDataAddress);
    Serial.println(node.ku8MBIllegalDataValue);
    Serial.println(node.ku8MBIllegalFunction);
    Serial.println(node.ku8MBInvalidCRC);
    Serial.println(node.ku8MBInvalidFunction);
    Serial.println(node.ku8MBInvalidSlaveID);
    Serial.println(node.ku8MBResponseTimedOut);
    Serial.println(node.ku8MBSlaveDeviceFailure);
    Serial.println(node.ku8MBSuccess);
    delay(5000);
    Serial.println("Clear bufs");
    node.clearResponseBuffer();
    node.clearTransmitBuffer();
  }
  

  return result;
} 


//MQTT
void mqttcallback(char *topic, byte *payload, unsigned int length)
{
  Serial.println("in Callback");
  if (strcmp(topic, "ventilation/control/SpeedMode") == 0)
  {
    Serial.println("VentSet MQTT");
    if (length == 1 && payload[0] >= '0' && payload[0] <= '4')
    {
      uint16_t speed = payload[0] - '0';
      WriteModbus(VENTSET, speed);
    }
  }
  if (strcmp(topic, "ventilation/control/Timer") == 0)
  {
    if (length == 1 && payload[0] >= '0' && payload[0] <= '3')
    {
      uint16_t mode = payload[0] - '0';
      WriteModbus(MODESET, mode);
    }
  }
  if (strcmp(topic, "ventilation/control/TempTarget") == 0)
  {
    Serial.println("Setting temptarget");
    if (length == 3 && payload[0] >= '0' && payload[0] <= '200')
    {
      Serial.println("Payload");
      Serial.println(payload[0]);
      String str;
      for (int i = 0; i < length; i++)
      {
        str += (char)payload[i];
      }
      Serial.println("str");
      Serial.println(str);
      
      int value = ((str.toInt()) - 100.0);
      Serial.println("Value");
      Serial.println(value);
      WriteModbus(TEMPSET, value);
    }
  }
  if (strcmp(topic, "ventilation/tempset") == 0)
  {
    if (length == 3 && payload[0] >= '0' && payload[0] <= '3')
    {
      String str;
      for (int i = 0; i < length; i++)
      {
        str += (char)payload[i];
      }
      WriteModbus(TEMPSET, ((str.toInt() * 100)-100));
    }
  }
   if (strcmp(topic, "ventilation/programset") == 0)
  {
    if (length == 1 && payload[0] >= '0' && payload[0] <= '4')
    {
      uint16_t program = payload[0] - '0';
      WriteModbus(PROGRAMSET, program);
    }
   } 
  lastMsg = -SENDINTERVAL;
}
 

char *getName(reqtypes type, int address)
{
  if (address >= 0 && address <= regsizes[type])
  {
    return regnames[type][address];
  }
  return NULL;
}
 
JsonObject HandleRequest(JsonDocument& doc)
{
  JsonObject root = doc.to<JsonObject>();
  reqtypes r = reqmax;
  char type = 0;
  if (req[1] != "")
  {
    for (int i = 0; i < reqmax; i++)
    {
      if (groups[i] == req[1])
      {
        r = (reqtypes)i;
      }
    }
  }
  type = regtypes[r];
  if (req[0] == "read")
  {
    int address = 0;
    int nums = 0;
    char result = -1;
    address = regaddresses[r];
    nums = regsizes[r];
 
    result = ReadModbus(address, nums, rsbuffer, type & 1);
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
            str += (char)(rsbuffer[i] & 0x00ff);
            str += (char)(rsbuffer[i] >> 8);
            root[name] = str;
          }
          else if (type & 8)
          {
            root[name] = (rsbuffer[i] - 300.0) /10.0;
          }
          else
          {
            root[name] = rsbuffer[i];
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
    for (int i = 0; i < reqmax; i++)
    {
      root[groups[i]] = 0;
    }
  }
  root["operation"] = req[0];
  root["group"] = req[1];
  return root;
}

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6]  = "1883";
char mqtt_user[40];
char mqtt_pass[40];

//default custom static IP
char static_ip[16] = "10.0.1.56";
char static_gw[16] = "10.0.1.1";
char static_sn[16] = "255.255.255.0";

//flag for saving data
bool shouldSaveConfig = false;
void tick()
{
  //toggle state
  digitalWrite(26, !digitalRead(26));     // set pin to the opposite state
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

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);

         

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
}

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
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt_user", mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt_pass", mqtt_pass, 40);

  //add all your parameters here
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_pass);
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
  if (!wm.autoConnect("GenvexAP")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    // if we still have not connected restart and try all over again
    ESP.restart();
    delay(5000);
 
  }
  // always start configportal for a little while
  wm.setConfigPortalTimeout(60);
  wm.startConfigPortal("GenvexAP");

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  ticker.detach(); 
  digitalWrite(26, 1);
  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonDocument json(1024);
    //JsonObject json = jsonDocument.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"]   = mqtt_port;
    json["mqtt_user"]   = mqtt_user;
    json["mqtt_pass"]   = mqtt_pass;

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

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.subnetMask());

  Serial2.begin(19200, SERIAL_8E1);
  node.begin(1, Serial2);
  node.clearResponseBuffer();
  node.clearTransmitBuffer();
  Serial.println("DBG - serverbegin");
  server.begin();
  Serial.println("DBG - mqtt set server is next");
  mqttclient.setServer(mqtt_server, 1883);
  Serial.println("DBG - mqtt Callback is next");
  mqttclient.setCallback(mqttcallback);
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
      Serial.println("DBG - MQTT all good");
      mqttclient.subscribe("ventilation/control/SpeedMode");
      mqttclient.subscribe("ventilation/control/Timer");
      mqttclient.subscribe("ventilation/control/TempTarget");
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
    //Serial.println("DBG - MQTT all good");
    mqttclient.loop();
    long now = millis();
    if (now - lastMsg > SENDINTERVAL)
    {
      //Serial.println("DBG - Doing some requests");
      //reqtypes rr[] = {reqtemp, reqruninfo, reqcontrol, reqtempcontrol, reqoutput, reqspeed, reqhumidity, reqinputairtemp, reqprogram, requser, reqdisplay, reqinfo}; // put another register in this line to subscribe
      reqtypes rr[] = {reqtemp, reqruninfo, reqspeed, reqtempcontrol, reqtime, reqhumidity, reqversion}; // put another register in this line to subscribe
      for (int i = 0; i < (sizeof(rr)/sizeof(rr[0])); i++)
      {
        reqtypes r = rr[i];
        //Serial.println("DBG - Reading registers");
        //Serial.println(groups[r]);
        //Serial.println(regaddresses[r]);
        //Serial.println(regsizes[r]);
        //Serial.println(regtypes[r]);
        char result = ReadModbus(regaddresses[r], regsizes[r], rsbuffer, regtypes[r] & 1);
        if (result == 0)
        {
          //Serial.println("DBG - Got result from Genvex");
          //Serial.println(result);
          mqttclient.publish("ventilation/error/modbus/", "0"); //no error when connecting through modbus
          for (int i = 0; i < regsizes[r]; i++)
          {
            char *name = getName(r, i);
            char numstr[8];
            if (name != NULL && strlen(name) > 0)
            {
              String mqname = "temp/";
              switch (r)
              {
              case reqspeed:
                mqname = "ventilation/speed/"; // Subscribe to the "speed" register
                itoa((rsbuffer[i]), numstr, 10);
                break;
              case reqruninfo:
                mqname = "ventilation/runinfo/"; // Subscribe to the "runinfo" register
                itoa((rsbuffer[i]), numstr, 10);
                break;
              case reqtempcontrol:
                mqname = "ventilation/tempcontrol/"; // Subscribe to the "tempcontrol" register
                //itoa((rsbuffer[i]), numstr, 10);
                dtostrf((rsbuffer[i] + 100.0) /10, 5, 2, numstr);
                break;
              case reqtime:
                mqname = "ventilation/time/"; // Subscribe to the "time" register
                itoa((rsbuffer[i]), numstr, 10);
                break;
              case reqhumidity:
                mqname = "ventilation/humidity/"; // Subscribe to the "humidity" register
                itoa((rsbuffer[i]), numstr, 10);
                break;
              case reqversion:
                mqname = "ventilation/version/"; // Subscribe to the "version" register
                itoa((rsbuffer[i]), numstr, 10);
                break;             
              case reqtemp:
                if (strncmp("RH", name, 2) == 0) {
                  mqname = "ventilation/moist/"; // Subscribe to moisture-level
                } else {
                  mqname = "ventilation/temp/"; // Subscribe to "temp" register
                }
                dtostrf((rsbuffer[i] - 300.0) /10, 5, 2, numstr);
                break;
              }
              mqname += (char *)name;
              mqttclient.publish(mqname.c_str(), numstr);
            }
          }
        }
        else {
          //Serial.println("DBG - Error reading");
          //Serial.println(groups[r]);
          //Serial.println(regaddresses[r]);
          //Serial.println(regsizes[r]);
          //Serial.println(regtypes[r]);
          mqttclient.publish("ventilation/error/modbus/", "1"); //error when connecting through modbus
        }      
      }
 
      // Handle text fields
      reqtypes rr2[] = {}; // put another register in this line to subscribe
      for (int i = 0; i < (sizeof(rr2)/sizeof(rr2[0])); i++) // change value "5" to how many registers you want to subscribe to
      {
        reqtypes r = rr2[i];
 
        char result = ReadModbus(regaddresses[r], regsizes[r], rsbuffer, regtypes[r] & 1);
        if (result == 0)
        {
          String text = "";
          String mqname = "ventilation/text/";
 
          for (int i = 0; i < regsizes[r]; i++)
          {
              char *name = getName(r, i);
 
              if ((rsbuffer[i] & 0x00ff) == 0xDF) {
                text += (char)0x20; // replace degree sign with space
              } else {
                text += (char)(rsbuffer[i] & 0x00ff);
              }
              if ((rsbuffer[i] >> 8) == 0xDF) {
                text += (char)0x20; // replace degree sign with space
              } else {
                text += (char)(rsbuffer[i] >> 8);
              }
              mqname += (char *)name;
          }
          mqttclient.publish(mqname.c_str(), text.c_str());
        }
      }
      lastMsg = now;
    }
  }

 }  