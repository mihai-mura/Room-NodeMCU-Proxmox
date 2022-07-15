#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266Ping.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <env.cpp>

// WiFi config
const IPAddress ip(192, 168, 100, 54);
const IPAddress gateway(192, 168, 100, 1);
const IPAddress bcastAddr(192, 168, 100, 255);
const IPAddress subnet(255, 255, 255, 0);
const IPAddress dns(192, 168, 100, 1);

// WOL device config
const IPAddress device_ip(192, 168, 100, 200);
byte macAddr[6] = {0x00, 0x23, 0x24, 0xBA, 0x55, 0x2F};
const char device_name[] = "Server";
const uint16_t boot_time = 200;
const int startStopTimeMargin = 10; // in seconds

// WebSocket config
WebSocketsClient webSocket;

DynamicJsonDocument outDoc(1024);
DynamicJsonDocument outStateDoc(1024);
DynamicJsonDocument inDoc(1024);

void sendData(String type, String data)
{
  outDoc["sender"] = "node-proxmox";
  outDoc["type"] = type;
  outDoc["data"] = data;
  String json;
  serializeJson(outDoc, json);
  webSocket.sendTXT(json);
}
void sendState(String ping, String rssi, String serverState)
{
  outStateDoc["sender"] = "node-proxmox";
  outStateDoc["type"] = "state";
  outStateDoc["ping"] = ping;
  outStateDoc["rssi"] = rssi;
  outStateDoc["serverState"] = serverState;
  String json;
  serializeJson(outStateDoc, json);
  webSocket.sendTXT(json);
}

// WOL
#define MAGIC_PACKET_LENGTH 102
#define PORT_WAKEONLAN 9
byte magicPacket[MAGIC_PACKET_LENGTH];
unsigned int localPort = 9;
WiFiUDP udp;

// timer
const unsigned long timerInterval = 1000;
unsigned long timerPreviousTime = 0;

// state
struct WOLServerState
{
  bool IsOnline;
  uint16_t boot_time;
  bool boot_error;
  uint16_t ping;
  uint32_t previousMillis;
  uint32_t interval;
  // first start & stop event notification
  bool firstStart;
  bool firstStop;
  bool stoppedCountDown;
  int stoppedTime;
};
WOLServerState currentState = {false, 0, false, 0, 0, 5000UL, true, true, false, 0};

void startServer()
{
  if (!currentState.IsOnline && currentState.boot_time == 0)
  {
    udp.beginPacket(bcastAddr, PORT_WAKEONLAN);
    udp.write(magicPacket, MAGIC_PACKET_LENGTH);
    udp.endPacket();

    currentState.boot_time = boot_time;
    currentState.interval = 1000UL;
  }
}

void timerFunction()
{

  if (currentState.IsOnline)
  {
    sendState(String(currentState.ping), String(WiFi.RSSI()), "online");
  }
  else if (!currentState.IsOnline && currentState.boot_time > 0)
  {
    sendState(String(currentState.ping), String(WiFi.RSSI()), "waiting");
  }
  else if (!currentState.IsOnline && currentState.boot_time == 0 && currentState.boot_error)
  {
    sendState(String(currentState.ping), String(WiFi.RSSI()), "error");
  }
  else
  {
    sendState(String(currentState.ping), String(WiFi.RSSI()), "offline");
  }

  if (currentState.stoppedCountDown && currentState.stoppedTime < startStopTimeMargin)
  {
    currentState.stoppedTime++;
  }
}

// Generate magic packet
void buildMagicPacket()
{
  memset(magicPacket, 0xFF, 6);
  for (int i = 0; i < 16; i++)
  {
    int ofs = i * sizeof(macAddr) + 6;
    memcpy(&magicPacket[ofs], macAddr, sizeof(macAddr));
  }
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.println("WebSocket Disconnected!");
    break;
  case WStype_CONNECTED:
    Serial.printf("WebSocket Connected");

    // send message to server when Connected
    Serial.println("WebSocket SENT: NodeMCU Connected");
    sendData("register-node-mcu", device_name);

    break;
  case WStype_TEXT:
    deserializeJson(inDoc, payload);
    const char *type = inDoc["type"];
    const char *data = inDoc["data"];
    Serial.println("WebSocket Received: " + String(type) + " " + String(data));
    if (String(type) == "power")
    {
      Serial.println("Start server");
      startServer();
    }
    break;
  }
}

void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.hostname("NodeMCU-Proxmox");
  WiFi.config(ip, dns, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int count = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(250);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);

    count++;
    if (count > 20)
    {
      delay(500);
      ESP.restart();
    }
  }

  Serial.println("Wifi connected, ip: " + WiFi.localIP().toString());
}

void connectCloud()
{
  webSocket.begin(SERVER_HOST, SERVER_PORT, "/");
  webSocket.onEvent(webSocketEvent);
}

void setup()
{
  Serial.begin(115200);

  connectWiFi();
  connectCloud();

  if (udp.begin(localPort) == 1)
  {
    Serial.println("UDP started");
    buildMagicPacket();
  }
  else
  {
    delay(500);
    ESP.restart();
  }
}

void loop()
{
  webSocket.loop();

  // Reconnect WiFi
  if (WiFi.status() != WL_CONNECTED)
  {
    connectWiFi();
    return;
  }

  // set local server state
  uint32_t currentMillis = millis();
  if (currentMillis - currentState.previousMillis >= currentState.interval)
  {
    currentState.previousMillis = currentMillis;

    if (currentState.boot_time == 0)
    {
      currentState.interval = 5000UL;
    }
    else
    {
      currentState.boot_time--;
      if (currentState.boot_time == 0)
      {
        currentState.boot_error = true;
        sendData("event", "boot_error");
      }
    }

    if (Ping.ping(device_ip, 1))
    {
      currentState.IsOnline = true;
      currentState.boot_error = false;
      currentState.boot_time = 0;
      currentState.ping = Ping.averageTime();
      currentState.firstStop = true;
      // first start notification
      if (currentState.firstStart)
      {

        sendData("event", "server_on");
        currentState.firstStart = false;
      }
      currentState.stoppedCountDown = false;
      currentState.stoppedTime = 0;
    }
    else
    {
      currentState.stoppedCountDown = true;
      if (currentState.stoppedTime == startStopTimeMargin)
      {
        currentState.IsOnline = false;
        currentState.ping = 0;
        currentState.firstStart = true;
        // first stop notification
        if (currentState.firstStop)
        {
          sendData("event", "server_off");
          currentState.firstStop = false;
        }
      }
    }
  }

  // timer run
  unsigned long currentTime = millis();
  if (currentTime - timerPreviousTime >= timerInterval)
  {
    timerFunction();
    timerPreviousTime = currentTime;
  }
}
