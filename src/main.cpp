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

/*------------------------------------------------------------------------------ 
VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------ 
VARIABLES - Shared
------------------------------------------------------------------------------*/

enum reqtypes {reqtemp, reqruninfo, reqtemptarget, reqspeed, reqtime, reqhumidity, reqversion, reqmax};
String manufacturer;

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
String lastReadValues[600];
// Esp8266 MAC will be added to the device name, to ensure unique topics
// Default is topics like 'heat/floorXXXXXXXXXXXX/3/target', where 3 is the output id and XXXXXXXXXXXX is the mac
const String   MQTT_PREFIX              = "ventilation/";       // include tailing '/' in prefix
const String   MQTT_ONLINE              = "/online";      
const String   MQTT_SUFFIX_CURRENT      = "/current";    // include heading '/' in all suffixes

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6]  = "1883";
char mqtt_user[40];
char mqtt_pass[40];

char charManufacturer[40];
String strManufacturer;


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
byte GenvexRegtypes[] = {8, 0, 1, 1, 1, 1, 0};
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










 


  
/*------------------------------------------------------------------------------ 
FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS FUNCTIONS
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------ 
FUNCTIONS - loop() helper functions
------------------------------------------------------------------------------*/

char *getName(reqtypes type, int address)
{
  if (address >= 0 && address <= GenvexRegsizes[type])
  {
    return GenvexRegnames[type][address];
  }
  return NULL;
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

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          strcpy(charManufacturer, json["charManufacturer"]);
         

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
  node.setTransmitBuffer(0, val);
  char result = 0;
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
  

  return result;
} 

/*------------------------------------------------------------------------------ 
FUNCTIONS - Json requests
------------------------------------------------------------------------------*/

JsonObject HandleRequest(JsonDocument& doc)
{
  JsonObject root = doc.to<JsonObject>();
  reqtypes r = reqmax;
  
  char type = 0;
  if (req[1] != "")
  {
    for (int i = 0; i < reqmax; i++)
    {
      if (GenvexGroups[i] == req[1])
      {
        r = (reqtypes)i;
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
    for (int i = 0; i < reqmax; i++)
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
    if (strcmp(topic, "ventilation/control/setSpeedMode") == 0)
    {
      Serial.println("VentSet MQTT");
      if (length == 1 && payload[0] >= '0' && payload[0] <= '4')
      {
        uint16_t speed = payload[0] - '0';
        WriteModbus(GenvexVENTSET, speed);
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
      Serial.println("Setting temptarget");
      if (length >= 3 && payload[0] >= '0' && payload[0] <= '200')
      {
        Serial.println("Payload");
        Serial.println(payload[0]);
        String str;
        for (int i = 0; i < length; i++)
        {
          if(isdigit((char)payload[i]))
          {
            str += (char)payload[i];
          }
        }
        Serial.println("str");
        Serial.println(str);
      
        int value = ((str.toInt()) - 100.0);
        Serial.println("Value");
        Serial.println(value);
        WriteModbus(GenvexTEMPSET, value);
      }
    }
  }
  if(strManufacturer == "nilan")
  {

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
  
    String tempTopic = String("homeassistant/sensor/" + strManufacturer + "/" + name + "/config");

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
      tempMessage += ("\"unit_of_measurement\": \"°C\", ");
      tempMessage += ("\"device_class\": \"temperature\", ");
    }

    // Add final line of config
    tempMessage += ("\"qos\": \"0\"}");
    // Publish the config
    Serial.println("topic");
    Serial.println(tempTopic);
    Serial.println("message");
    Serial.println(tempMessage);
    Serial.println(mqttclient.publish(tempTopic.c_str(), tempMessage.c_str()));
    Serial.println("-------------- Publish Config End --------------");
  }
  if(strManufacturer = "nilan")
  {}
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

  if(!((lastReadValues[(lastReadValueIndex.toInt())]).equals((String)numstr))) //If numstr is NOT in lastReadValues
  {
    mqttclient.publish(mqname.c_str(), numstr); // Publish new value to mqtt
    lastReadValues[(lastReadValueIndex.toInt())] = (String)numstr; // Add/update the value in lastReadValues with the unique lastReadValueIndex
  }
}

// Publish config topic to MQTT
void publishConfigToMQTT(int rrint, String mqname, char *name, int iRegSize, reqtypes r)
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
    {}
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
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt_user", mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt_pass", mqtt_pass, 40);
  WiFiManagerParameter custom_manufacturer("manufacturer", "charManufacturer", charManufacturer, 40);

  //add all your parameters here
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_pass);
  wm.addParameter(&custom_manufacturer);  
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
  strcpy(charManufacturer, custom_manufacturer.getValue());
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonDocument json(1024);
    //JsonObject json = jsonDocument.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"]   = mqtt_port;
    json["mqtt_user"]   = mqtt_user;
    json["mqtt_pass"]   = mqtt_pass;
    json["charManufacturer"]        = charManufacturer;

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

  // Configuring mqttclient (PubSubClient)
  mqttclient.setServer(mqtt_server, 1883);
  // Making sure the buffer is large enough for the MQTT discovery configurations
  mqttclient.setBufferSize(512);
  mqttclient.setCallback(mqttcallback);

  // Converting charManufacturer to String and save in strManufacturer (and convert to lower case)
  strManufacturer = (String)charManufacturer;
  strManufacturer.toLowerCase();
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
      mqttclient.subscribe("ventilation/control/setSpeedMode");
      mqttclient.subscribe("ventilation/control/setTimer");
      mqttclient.subscribe("ventilation/control/setTempTarget");

      // Publish availability topic - True
      String will = String(MQTT_PREFIX + "genvex" + MQTT_ONLINE);
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
      // if(!configurationPublished[0])
      //{
      //  Serial.println("Sleeping.............");
      //  sleep(2);
      //}
      
      // Iterating over the reqtypes
      if(strManufacturer == "genvex")
      {
        reqtypes rr[] = {reqtemp, reqruninfo, reqspeed, reqtemptarget, reqtime, reqhumidity, reqversion}; // put another register in this line to subscribe
        for (int rrint = 0; rrint < (sizeof(rr)/sizeof(rr[0])); rrint++)
        {
          reqtypes r = rr[rrint];

          char result = ReadModbus(GenvexRegaddresses[r], GenvexRegsizes[r], Genvexrsbuffer, GenvexRegtypes[r] & 1);
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



            // Iterating through the regsize for the reqtype
            for (int iRegSize = 0; iRegSize < GenvexRegsizes[r]; iRegSize++)
            {
              String unit = "";
              char *name = getName(r, iRegSize);
              char numstr[8];
              if   (name != NULL && strlen(name) > 0)
              { 
                String mqname = "temp/";
                switch (r)
                { 
                  case reqspeed:
                    mqname = "ventilation/speed/"; // Subscribe to the "speed" register
                    itoa((Genvexrsbuffer[iRegSize]), numstr, 10);
                    break;
                  case reqruninfo:
                    mqname = "ventilation/runinfo/"; // Subscribe to the "runinfo" register
                    itoa((Genvexrsbuffer[iRegSize]), numstr, 10);
                    break;
                  case reqtemptarget:
                    mqname = "ventilation/temp/"; // Subscribe to the "tempcontrol" register
                    dtostrf((Genvexrsbuffer[iRegSize] + 100.0) /10, 5, 2, numstr);
                    break;
                  case reqtime:
                    mqname = "ventilation/time/"; // Subscribe to the "time" register
                    itoa((Genvexrsbuffer[iRegSize]), numstr, 10);
                    break;
                  case reqhumidity:
                    mqname = "ventilation/humidity/"; // Subscribe to the "humidity" register
                    itoa((Genvexrsbuffer[iRegSize]), numstr, 10);
                    break;
                  case reqversion:
                    mqname = "ventilation/version/"; // Subscribe to the "version" register
                    itoa((Genvexrsbuffer[iRegSize]), numstr, 10);
                    break;             
                  case reqtemp:
                    if (strncmp("RH", name, 2) == 0) {
                      mqname = "ventilation/moist/"; // Subscribe to moisture-level
                    } else {
                      mqname = "ventilation/temp/"; // Subscribe to "temp" register
                    }
                    dtostrf((Genvexrsbuffer[iRegSize] - 300.0) /10, 5, 2, numstr);
                  break;
                }
                mqname += (char *)name;

                publisToMQTT(rrint, iRegSize, numstr, mqname);

                publishConfigToMQTT(rrint, mqname, name, iRegSize, r);
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
        // Handle text fields
        reqtypes rr2[] = {}; // put another register in this line to subscribe
        for (int i = 0; i < (sizeof(rr2)/sizeof(rr2[0])); i++) // change value "5" to how many registers you want to subscribe to
        {
          reqtypes r = rr2[i];
 
          char result = ReadModbus(GenvexRegaddresses[r], GenvexRegsizes[r], Genvexrsbuffer, GenvexRegtypes[r] & 1);
          if (result == 0)
          {
            String text = "";
            String mqname = "ventilation/text/";
 
            for (int i = 0; i < GenvexRegsizes[r]; i++)
            {
              char *name = getName(r, i);
 
              if ((Genvexrsbuffer[i] & 0x00ff) == 0xDF) {
                text += (char)0x20; // replace degree sign with space
              } else {
                text += (char)(Genvexrsbuffer[i] & 0x00ff);
              }
              if ((Genvexrsbuffer[i] >> 8) == 0xDF) {
                text += (char)0x20; // replace degree sign with space
              } else {
                text += (char)(Genvexrsbuffer[i] >> 8);
              }
              mqname += (char *)name;
            }
            mqttclient.publish(mqname.c_str(), text.c_str());
          }
        }
      }
      lastMsg = now;
    }
  }

 }  