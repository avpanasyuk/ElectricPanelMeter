#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <stdint.h>

const char *ssid = "T2_4";
const char *password = "group224";

//////////////////////
// WiFi Definitions //
//////////////////////
const char WiFiAPPSK[] = "esp";

/////////////////////
// Pin Definitions //
/////////////////////
const int LED_PIN = 5;      // Thing's onboard, green LED
const int ANALOG_PIN = A0;  // The only analog pin on the Thing
const int DIGITAL_PIN = 12; // Digital pin to be read

WiFiServer server(80);

void setupWiFi() {
  WiFi.mode(WIFI_AP);

  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "ESP8266 Thing " + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i = 0; i < AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, WiFiAPPSK);
}

void initHardware() {
  Serial.begin(115200);
  Serial.println();

  // pinMode(DIGITAL_PIN, INPUT_PULLUP);
  // pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, LOW);
  // Don't need to set ANALOG_PIN as input,
  // that's all it can be.
}

static constexpr uint16_t BUFFER_SIZE = 60;

static struct {
  uint16_t ADC;
  uint32_t Time;
} MeasBuffer[BUFFER_SIZE], *CurEntry = MeasBuffer;

static void measurement() {
  if (CurEntry != MeasBuffer + BUFFER_SIZE) {
    CurEntry->ADC = analogRead(ANALOG_PIN);
    CurEntry->Time = micros();
    CurEntry++;
  }
} // measurement

void setup() {
  initHardware();
  delay(3000);

  setupWiFi();
  Serial.printf("Connecting to %s ", ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  server.begin();
  Serial.printf("Web server started, open %s in a web browser\n",
                WiFi.localIP().toString().c_str());
  server.begin();

  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
}

static WiFiClient client;

static void send_back(const String &message) {
  String s(F("HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n\r\n"
             "<!DOCTYPE HTML>\r\n<html>\r\n"));
  s += message;
  s += "</html>\n";
  client.print(s);
  delay(1);
  Serial.println("Client disconnected.");
  client.stop();
} // send_back

static void Set74HC4051_code(uint8_t c) {
  analogWrite(D0, c & 1);
  analogWrite(D1, (c >> 1) & 1);
  analogWrite(D2, (c >> 2) & 1);
} // Set74HC4051_code

void loop() {
  if (client) { // client is still waiting data
    // let's see whether we have finished measurement
    if (CurEntry == MeasBuffer + BUFFER_SIZE) {
      String s;
      for (uint16_t si = 0; si < BUFFER_SIZE; si++) {
        s += String(MeasBuffer[si].Time) + ", " + String(MeasBuffer[si].ADC) +
             "<br>";
      }
      CurEntry = MeasBuffer;
      send_back(s);
    } else
      measurement();
  } else {
    // Check if a client has connected
    client = server.available();
    if (client) {
      // Read the first line of the request
      String req = client.readStringUntil('\r');
      Serial.println(req);
      client.flush();

      // parse request
      static int Port = -1;

      if ((Port = req.lastIndexOf("/port")) != -1) {
        // let's read port number
        Port = (req.charAt(Port + strlen("/port/")) - '0');
        if (Port < 0 || Port > 7) {
          send_back(String(F("Wrong port number! <br>")));
        } else
          Set74HC4051_code(uint8_t(Port));
      } else {
        send_back(String(F("Invalid Request.<br> Try /port/<port>.")));
      }
    }
  }
}
