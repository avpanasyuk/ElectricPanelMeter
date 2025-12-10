
#include <stdint.h>
#include <stdio.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "C_General/Macros.h"
#include "C_General/MyMath.hpp"
#include "C_General/MyTime.hpp"
#include "C_ESP/WebServer.h"

static std::unique_ptr<avp::WebServer> a;

#define DEBUG_SERIAL Serial

extern "C" {
  int debug_puts(const char *s) {
    if(a != nullptr) a->AddToLog(s);
#ifdef DEBUG
    if(DEBUG_SERIAL) {
      DEBUG_SERIAL.print(s);
      DEBUG_SERIAL.flush();
    }
#endif
    return 0;
  }
}

/////////////////////
// Pin Definitions //
/////////////////////
constexpr uint8_t LED_PIN = D8; // Thing's onboard, green LED
constexpr int ANALOG_PIN = A0;  // The only analog pin on the Thing
constexpr uint8_t NumCtrlLines_74HC4051 = 3;
constexpr uint8_t Num74HC4051 = 2;

static constexpr uint32_t GetDelay(uint8_t DelIndex) { // exponentially scaled delay
  return 10UL * pow(2, DelIndex);                      // us
} // GetDelay

// WF acronim is WaveForm, one 60 Hz period
static constexpr uint16_t SAMPLES_IN_WF = 600;
static constexpr uint32_t WF_SAMPLE_PERIOD = 1000000UL / SAMPLES_IN_WF; // microseconds

static uint32_t Delay = GetDelay(4); // tuned for no visible phase shift. Hmm, HOW?
static uint32_t NewSample = micros();
static inline void Set74HC4051_code(uint8_t c, uint8_t SwitchNum);

#if VERSION == 2
#include "V2.incl"
#else
#include "V1.incl"
#endif

constexpr uint8_t VoltagePortI = 0;
constexpr uint8_t GND_PortI = NUM_ports - 1;

static inline void Set74HC4051_code(uint8_t c, uint8_t SwitchNum) {
  for(uint8_t BitI = 0; BitI < NumCtrlLines_74HC4051; BitI++)
    digitalWrite(ControlLines[SwitchNum][BitI], (c >> BitI) & 1);
} // Set74HC4051_code

static inline bool is_past(uint32_t time_us) { return (micros() - time_us) < (1UL << 31); } // is_past

// integration data
static struct {
  float Power, Voltage, Current;
  uint32_t NumSamples;
} Integral[NUM_ports];

static uint32_t NumScans;

/**
samples samples over one 60 Hz Wave alternatively from CurPort
port and from VoltagePort and then accumulates them into Power[CurPort] and
NumSamples[NUM_ports] and moves CurPort to the next port
*/
static inline void sample_port_and_go_to_next() {} // sample_port_and_go_to_next

static const String &samples2string() {
  static String s;
  s.reserve(200); // reserve buffer for response to avoid dynamic memory allocation
  s = "";
  for(auto &I : Integral) {
    float Power = (I.Power - I.Current * I.Voltage / I.NumSamples) / I.NumSamples;
    s += String(Power);
    s += "<br>";
    I.Power = I.Current = I.Voltage = 0.;
    I.NumSamples = 0;
  }
  return (s);
} // samples2string

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Serial port connected!");

  SetPins();

  auto Opts = avp::WebServer::DefaultOpts();

  Opts.Name = NAME; // NAME should be specified in platformio.ini, so it is in sync with upload_port in espota
  Opts.Version = "2.04";
  Opts.AddUsage = F("<li> read - returns column of power value for each port</li>"
                    "<li> scan - returns all samples collected so far</li>"
                    "<li> port?i=n - reads port n and returns its value</li>");
  Opts.status_indication_func_ = avp::WebServer::BlinkerFunc<LED_PIN>;

  a = avp::WebServer::Create(Opts);
  while(!WiFi.isConnected()) {
    Serial.print(".");
    a->TryToConnect();
  }
  // try to connect to WiFi
  debug_puts("Logging here...");

  a->on("/read", [](avp::WebServer::Request_t &&rReq) {
    static String s;
    s.reserve(200); // reserve buffer for response to avoid dynamic memory allocation
    s = "<html>";
    s += samples2string();
    s += "<br></html>";
    rReq.send("text/html", s);
  });
  a->on("/scan", [](avp::WebServer::Request_t &&rReq) {
    static String Resp;
    Resp.reserve(200);

    Resp += "<html>Scan N: ";
    Resp += NumScans;
    Resp += F("<br>----------------------------------<br>");
    Resp += samples2string();
    Resp += "<br></html>";
    rReq.send("text/html", Resp);
  });
  a->on("/port", [](avp::WebServer::Request_t &&rReq) {
    if(rReq.hasArg("i")) {
      auto Port = rReq.arg("i").toInt();
      if(Port < 0 || Port >= NUM_ports) rReq.send("text/plain", F("Wrong port number!"));
      else {
        ConnectPort(Port);
        static String Content;
        Content.reserve(80);
        Content = "Port  = ";
        Content += Port;
        Content += ", ";
        Content += analogRead(ANALOG_PIN);

        rReq.send("text/plain", Content);
      }
    } else rReq.send("text/plain", F("Usage: /port?i=n where n is port number"));
  });
  a->begin();
  // wifi_set_sleep_type(NONE_SLEEP_T);
} // setup

IGNORE_WARNING(-Wdangling - else)

void loop() {
  yield();

  // integrating power over one 60Hz wave period
  static avp::TimePeriod1<1000000UL / 60, micros> TP;
  static uint8_t CurPort = 0;
  static uint16_t Counter; ///< we are counting 60 Hz WL periods. During Counter == 0
  // we are reading one port as fast as possible, then we skip WIFI_timeRatio WLs for WiFi to do its thing
  static constexpr uint16_t WIFI_timeRatio = 10; // ratio of WiFi time to analog read time

  if((Counter == 0) && !a->OTA_IsInProgress) {
    ConnectPort(CurPort);
    uint16_t Current = analogRead(ANALOG_PIN);
    ConnectPort(VoltagePortI);
    uint16_t Voltage = analogRead(ANALOG_PIN);

    Integral[CurPort].Power += Voltage * Current;
    Integral[CurPort].Voltage += Voltage;
    Integral[CurPort].Current += Current;
    Integral[CurPort].NumSamples++;
  } else a->call_in_loop();
  if(TP.Expired())
    if(Counter == 0) {
      if(++CurPort == NUM_ports) {
        CurPort = 0;
        ++NumScans;
      }
      Counter = WIFI_timeRatio;
    } else --Counter;
} // loop
