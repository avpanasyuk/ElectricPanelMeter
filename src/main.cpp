#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <stdint.h>
#include <stdio.h>

const char *ssid = "T2_4";
const char *password = "group224";

//////////////////////
// WiFi Definitions //
//////////////////////
const char WiFiAPPSK[] = "esp";

/////////////////////
// Pin Definitions //
/////////////////////
const int LED_PIN = 5;     // Thing's onboard, green LED
const int ANALOG_PIN = A0; // The only analog pin on the Thing
static constexpr uint16_t BUFFER_SIZE = 60, NUM_PORTS = 8, VOLTAGE_PORT = 7,
                          SamplingTime = 3000 /* ms */;
static constexpr uint16_t SAMPLES_IN_WF = 600;

static uint16_t Waveform[SAMPLES_IN_WF];
static float ProdIntegral[NUM_PORTS], VoltIntegral[NUM_PORTS],
    CrntIntegral[NUM_PORTS];

static constexpr uint16_t GetDelay(uint8_t DelIndex) { return 600.*pow(2,DelIndex/4.); }

static uint16_t SampleCount, Delay = GetDelay(5);
static uint8_t CurPort;

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

int debug_vprintf(const char *format, va_list a) {
  static constexpr uint8_t BUFFER_SIZE = 255;
  static char Buffer[BUFFER_SIZE];
  Serial.println(vsnprintf(Buffer, BUFFER_SIZE, format, a));
  return 1;
} // debug_vprintf

static void Set74HC4051_code(uint8_t c) {
  digitalWrite(D0, c & 1);
  digitalWrite(D1, (c >> 1) & 1);
  digitalWrite(D2, (c >> 2) & 1);
  // Serial.print(F("Port = "));
  // Serial.println(int(c));
  // delay(Delay);
  delayMicroseconds(Delay);
} // Set74HC4051_code

static void sample_waveform() {
  Set74HC4051_code(VOLTAGE_PORT);
  Set74HC4051_code(CurPort);
  Waveform[SampleCount++] = analogRead(ANALOG_PIN);
  yield();
} // sample_waveform

static void sample_all_ports() {
  // each time we go over all the ports and add product to ProdIntergral
  Set74HC4051_code(VOLTAGE_PORT);
  float PrevVoltage = analogRead(ANALOG_PIN);
  for (uint8_t PortI = 0; PortI < NUM_PORTS; PortI++) {
    Set74HC4051_code(PortI);
    float Current = analogRead(ANALOG_PIN);
    Set74HC4051_code(VOLTAGE_PORT);
    float Voltage = analogRead(ANALOG_PIN);
    float MeanVoltage = (Voltage + PrevVoltage)/2.; // compensating for time delay
    // between voltage and current sampling
    PrevVoltage = Voltage;
    ProdIntegral[PortI] += MeanVoltage * Current;
    VoltIntegral[PortI] += MeanVoltage;
    CrntIntegral[PortI] += Current;
  }
  yield();
  ++SampleCount;
} // sample_all_ports

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

  Set74HC4051_code(VOLTAGE_PORT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);
} // setup

static WiFiClient client;

static void reply(const String &message) {
  String s(F("HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n\r\n"
             "<!DOCTYPE HTML>\r\n<html>\r\n"));
  s += message;
  s += "</html>\n";
  client.print(s);
  delay(1);
  Serial.println("Client disconnected.");
  client.stop();
} // reply

static enum { SamplingPorts, SamplingWF } State;

void loop() {
  static uint32_t SamplingEnd;

  if (client) { // client is still waiting for data
    // let's see whether we have finished sample_all_ports
    switch (State) {
    case SamplingPorts:
      if (millis() - SamplingEnd < (1UL << 31)) {
        String s(String(SampleCount) +
                 "<br>----------------------------------<br>");
        for (uint8_t PortI = 0; PortI < NUM_PORTS; PortI++) {
#if 1
          float Pwr =
              (ProdIntegral[PortI] -
               CrntIntegral[PortI] * VoltIntegral[PortI] / SampleCount) /
              SampleCount;
          s += String(Pwr) + "<br>";
#else
          s += String(ProdIntegral[PortI]) + " " String(VoltIntegral[PortI]) +
               " " + String(CrntIntegral[PortI]) + "<br>";
#endif
          ProdIntegral[PortI] = CrntIntegral[PortI] = VoltIntegral[PortI] = 0;
        }
        SampleCount = 0;
        reply(s);
      } else
        sample_all_ports();
      break;
    case SamplingWF:
      if (SampleCount < SAMPLES_IN_WF)
        sample_waveform();
      else {
        String s;
        for (uint16_t SampleI = 0; SampleI < SAMPLES_IN_WF; ++SampleI)
          s += String(Waveform[SampleI]) + " ";
        SampleCount = 0;
        reply(s);
      }
      break;
    }
  } else {
    // Check if a client has connected
    client = server.available();
    if (client) {
      // Read the first line of the request
      String req = client.readStringUntil('\r');
      Serial.println(req);
      client.flush();

      // parse request
      int Port;

      if (req.lastIndexOf("/sample") != -1) {
        // let's read port number
        SamplingEnd = millis() + SamplingTime;
        State = SamplingPorts;
      } else if ((Port = req.lastIndexOf("/port")) != -1) {
        // let's read port number
        Port = (req.charAt(Port + strlen("/port/")) - '0');
        if (Port < 0 || Port > 7) {
          reply(String(F("Wrong port number! <br>")));
        } else {
          Set74HC4051_code(uint8_t(Port));
          String s("Port  =");
          s += String(Port) + ", " + String(analogRead(ANALOG_PIN));
          reply(s);
        }
      } else if ((Port = req.lastIndexOf("/wave")) != -1) {
        CurPort = (req.charAt(Port + strlen("/wave/")) - '0');
        if (CurPort < 0 || CurPort > 7) {
          reply(String(F("Wrong port number! <br>")));
        } else {
          // Set74HC4051_code(uint8_t(Port));
          State = SamplingWF;
        }
      } else if ((Port = req.lastIndexOf("/delay")) != -1) {
        Delay = GetDelay(req.charAt(Port + strlen("/delay/")) - '0');
        reply(String(F("Delay is set to ")) + String(Delay) + " microseconds.<br>");
      } else
        reply(String(
            F("Invalid Request.<br> Try /sample or /port/? or /wave/?.")));
    }
  }
} // loop
