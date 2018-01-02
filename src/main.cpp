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
static constexpr uint8_t NUM_PORTS = 8, VOLTAGE_PORT = 7;

static struct Integrals_ {
  float Power, Voltage, Current;
} Integrals[NUM_PORTS]; // two alterrnating buffers

static uint32_t SampleCount;
static uint32_t SamplingStarted; // milliseconds

static constexpr uint32_t GetDelay(uint8_t DelIndex) {
  return 10UL * pow(2, DelIndex);
} // GetDelay

static constexpr uint16_t SAMPLES_IN_WF = 600;
static constexpr uint32_t WF_SAMPLE_INTERVAL =
    1000000UL / SAMPLES_IN_WF; // microseconds

static uint32_t Delay = GetDelay(5); // tuned for no visible phase shift
static uint32_t NewSample = micros();

static void Set74HC4051_code(uint8_t c) {
  digitalWrite(D0, c & 1);
  digitalWrite(D1, (c >> 1) & 1);
  digitalWrite(D2, (c >> 2) & 1);
  delayMicroseconds(Delay);
} // Set74HC4051_code

static inline bool is_past(uint32_t time_us) {
  return (micros() - time_us) < (1UL << 31);
} // is_past

static void start_sampling() {
  NewSample = micros();
  SamplingStarted = millis();
} // start_sampling

static void sample_all_ports() {
  for (uint8_t PortI = 0; PortI < NUM_PORTS; ++PortI) {
    uint32_t Voltage, Current;
    Set74HC4051_code(PortI);
    Current = analogRead(ANALOG_PIN);
    Set74HC4051_code(VOLTAGE_PORT);
    Voltage = analogRead(ANALOG_PIN);

    Integrals[PortI].Power += Voltage * Current;
    Integrals[PortI].Voltage += Voltage;
    Integrals[PortI].Current += Current;
  }
  SampleCount++;
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
  start_sampling();
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
    static uint16_t Waveform[SAMPLES_IN_WF];
  // Serial.printf("%lu-%lu..", micros(), EndDelay);

  if (is_past(NewSample)) {
    Waveform[--WF_sampling_count] = analogRead(ANALOG_PIN);
    // Serial.printf("%d..", WF_sampling_count);
    if (WF_sampling_count == 0) {
      String s;
      for (uint16_t SampleI = 0; SampleI < SAMPLES_IN_WF; ++SampleI)
        s += String(Waveform[SampleI]) + " ";
      reply(s);
    }
    NewSample = micros() + WF_SAMPLE_INTERVAL;
  }
} // sample_waveform

void loop() {
  if (client) {
    if (WF_sampling_count)
      sample_waveform();
  } else {
    if (is_past(NewSample)) { // delay is over
      sample_all_ports();
      NewSample = micros() + WF_SAMPLE_INTERVAL*10;
    }

    // Check if a client has connected
    client = server.available();
    if (client) {
      uint8_t LEDstate = 1;
      digitalWrite(LED_PIN, LEDstate = 1 - LEDstate);

      // Read the first line of the request
      String req = client.readStringUntil('\r');
      Serial.println(req);
      client.flush();

      // parse request
      int Port;

      if (req.lastIndexOf("/sample") != -1) {
        float SamplingTime = millis() - SamplingStarted;
        String s(String(SamplingTime / SampleCount) + " ms per sample over " +
                 String(SamplingTime) +
                 " ms<br>----------------------------------<br>");
        for (uint8_t p = 0; p < NUM_PORTS; p++) {
          Integrals_ *pInt = &Integrals[p];
          float Pwr =
              (pInt->Power - pInt->Current * pInt->Voltage / SampleCount) /
              SampleCount;
          s += String(Pwr) + "<br>";
          pInt->Power = pInt->Current = pInt->Voltage = 0;
        }
        reply(s);
        SampleCount = 0;
        SamplingStarted = millis();
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
        Port = (req.charAt(Port + strlen("/wave/")) - '0');
        if (Port < 0 || Port > 7) {
          reply(String(F("Wrong port number! <br>")));
        } else {
          Set74HC4051_code(uint8_t(Port));
          WF_sampling_count = SAMPLES_IN_WF;
        }
      } else if ((Port = req.lastIndexOf("/delay")) != -1) {
        Delay = GetDelay(req.charAt(Port + strlen("/delay/")) - '0');
        reply(String(F("Delay is set to ")) + String(Delay) +
              " microseconds.<br>");
      } else if ((Port = req.lastIndexOf("/test")) != -1) {
        Port = (req.charAt(Port + strlen("/test/")) - '0');
        uint32_t Start = micros();
        for (uint16_t n = 10000; n; --n) {
          analogRead(ANALOG_PIN);
        }
        reply(String(F("10000 loops over ")) + String(micros() - Start) +
              " us");
      } else
        reply(String(
            F("Invalid Request.<br> Try /sample or /port/? or /wave/?.")));
    }
  }
} // loop
