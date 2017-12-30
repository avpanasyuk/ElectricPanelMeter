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
static constexpr uint16_t BUFFER_SIZE = 60, NUM_PORTS = 8, VOLTAGE_PORT = 7;

static struct Integrals_ {
  float Power, Voltage, Current;
} Integrals[NUM_PORTS][2]; // two alterrnating buffers
uint32_t SampleCount[2];

static uint8_t ActiveBuffer = 0; // togggles between 0 and 1
static enum {
  Idle,
  SwitchingToVoltage,
  SwitchingToCurrent
} SamplingState = Idle;

static uint32_t Delay = GetDelay(5);
static uint32_t EndDelay = micros();

static constexpr uint32_t GetDelay(uint8_t DelIndex) {
  return 600UL. * pow(2, DelIndex / 4.);
} // GetDelay

static void Set74HC4051_code(uint8_t c) {
  digitalWrite(D0, c & 1);
  digitalWrite(D1, (c >> 1) & 1);
  digitalWrite(D2, (c >> 2) & 1);
} // Set74HC4051_code

static bool is_past(uint32_t time_us) { return micros() - EndDelay) < (1UL << 31); }

static uint8_t PortI;
static float Voltage0;

static void start_sampling() {
  Set74HC4051_code(VOLTAGE_PORT);
  delayMicroseconds(Delay);
  Voltage0 = analogRead(ANALOG_PIN);
  Set74HC4051_code(PortI = 0);
  SamplingState = SwitchingToCurrent;
} // start_sampling

static void sample_all_ports() {
  if (is_past(EndDelay)) { // delay is over
    static float Current;
    float Voltage;

    Integrals_ *pIntegral = &Integrals[PortI][ActiveBuffer];

    switch (SamplingState) {
    case SwitchingToCurrent:
      Current = analogRead(ANALOG_PIN);
      Set74HC4051_code(VOLTAGE_PORT);
      SamplingState = SwitchingToVoltage;
      break;
    case SwitchingToVoltage:
      Voltage = analogRead(ANALOG_PIN);
      Voltage0 = (Voltage + Voltage0) / 2.; // compensating for time delay
      // between voltage and current sampling
      pIntegral->Power += Voltage0 * Current;
      pIntegral->Voltage += Voltage0;
      pIntegral->Current += Current;
      Voltage0 = Voltage;
      if (PortI++ == NUM_PORTS) PortI = 0;
      Set74HC4051_code(PortI);
      SamplingState = SwitchingToCurrent;
      break;
    }
    EndDelay = micros() + Delay;
  }
} // sample_all_ports

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

    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, 0);
  } // initHardware

int debug_vprintf(const char *format, va_list a) {
  static constexpr uint8_t BUFFER_SIZE = 255;
  static char Buffer[BUFFER_SIZE];
  Serial.println(vsnprintf(Buffer, BUFFER_SIZE, format, a));
  return 1;
} // debug_vprintf

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

static uint16_t WF_sampling_count;

static void sample_waveform() {
  static constexpr uint16_t SAMPLES_IN_WF = 600;
  static uint16_t Waveform[SAMPLES_IN_WF];

    Waveform[--WF_sampling_count] = analogRead(ANALOG_PIN);
    if(WF_sampling_count == 0) {
      String s;
      for (uint16_t SampleI = 0; SampleI < SAMPLES_IN_WF; ++SampleI)
        s += String(Waveform[SampleI]) + " ";
      reply(s);
    }
} // sample_waveform


void loop() {
  if(WF_sampling_count) sample_waveform();
  else
    if(SamplingState != Idle && WF_sampling_count == 0) sample_all_ports();

  static uint32_t SamplingEnd;

  if (client) { // client is waiting for data
    switch (State) {
    case SamplingPorts:
    if(PortI == 0) { // all ports are sampled
    // transmit  buffer
    uint32_t &sc = SampleCount[ActiveBuffer];
        String s(String(sc) +
                 "<br>----------------------------------<br>");
        for (uint8_t p = 0; p < NUM_PORTS; p++) {
          pIntegral = &Integrals[p][ActiveBuffer];
          float Pwr =
              (pIntegral->Power -
               pIntegral->Current * pIntegral->Voltage / sc) /
              sc;
          s += String(Pwr) + "<br>";
          pIntegral->Power = pIntegral->Current = pIntegral->Voltage = 0;
        }
        sc = 0;
        reply(s);
        // switch active buffer
        ActiveBuffer = 1 - ActiveBuffer;
            }

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
        reply(String(F("Delay is set to ")) + String(Delay) +
              " microseconds.<br>");
      } else
        reply(String(
            F("Invalid Request.<br> Try /sample or /port/? or /wave/?.")));
    }
  }
} // loop
