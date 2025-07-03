#include "C_ESP/board_sync_server.h"
#include "C_General/Macros.h"
#include "C_General/MyMath.hpp"
#include "C_General/MyTime.hpp"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <stdint.h>
#include <stdio.h>

// where to direct debug_ output
static ESP_board_sync_server *a;

#define DEBUG_SERIAL Serial

extern "C" {
  int debug_puts(const char *s) {
#ifdef DEBUG
    if(a != nullptr) a->AddToLog(s);
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

// evidently ADC (analogRead) is severely interfering with WiFi, aspecially if called too often
// so let's try not to call it continuosly, but NumSamplesPerWL times per Wave Length
constexpr uint32_t NumSamplesPerWL = 8;

/**
samples samples over one 60 Hz Wave alternatively from CurPort
port and from VoltagePort and then accumulates them into Power[CurPort] and
NumSamples[NUM_ports] and moves CurPort to the next port
*/
static inline void sample_port_and_go_to_next() {} // sample_port_and_go_to_next

static const String &samples2string() {
  static String s; s.reserve(512); s = ""; // reserve buffer for response to avoid dynamic memory allocation
  for(uint8_t CurPort = 0; CurPort < NUM_ports; ++CurPort) {
    auto &I = Integral[CurPort];
    float Power = (I.Power - I.Current * I.Voltage / I.NumSamples) / I.NumSamples / NumSamplesPerWL;
    s += String(Power) + "<br>";
    I.Power = I.Current = I.Voltage = 0.;
    I.NumSamples = 0;
  }
  return (s);
} // samples2string

void setup() {
  Serial.begin(115200);
  Serial.println();
  static String r; // reserve buffer for responses to avoid dynamic memory allocation
  r.reserve(2048);

  SetPins();
  delay(3000);

  auto Opts = ESP_board_sync_server::Default();

  Opts.Name = NAME; // NAME should be specified in platformio.ini, so it is in sync with upload_port in espota
  Opts.Version = "5.00";
  Opts.AddUsage = F("<li> read - returns <em>\"Voltage Current Power Energy </li>");
  Opts.status_indication_func_ = ESP_board_sync_server::BlinkerFunc<LED_PIN>;

  a = new ESP_board_sync_server(Opts);
  a->TryToConnect(); // try to connect to WiFi
  debug_puts("Logging here...");

  auto &w = a->server;

  w.on("/read", HTTP_GET, [&]() { w.send(200, "text/html", samples2string()); });
  w.on("/scan", HTTP_GET, [&]() {
    r = F("Scan N: "); r += String(NumScans);  r += F("<br>----------------------------------<br>");
    r += samples2string();  
    
    w.send(200, "text/html",r);
  });
  w.on("/port", HTTP_GET, [&]() {
    if(w.hasArg("i")) {
      auto Port = w.arg("i").toInt();
      if(Port < 0 || Port >= NUM_ports) w.send(200, "text/plain", String(F("Wrong port number!")));
      else {
        ConnectPort(Port);
        w.send(200, "text/plain", "Port  = " + String(Port) + ", " + String(analogRead(ANALOG_PIN)));
      }
    } else w.send(200, "text/plain", "Usage: /port?i=n where n is port number");
  });
  w.on("/delay", HTTP_GET, [&]() {
    if(w.hasArg("t")) {
      Delay = GetDelay(w.arg("t").toInt());
      Delay = w.arg("t").toInt();
      w.send(200, "text/plain", "Delay is set to " + String(Delay) + " microseconds.");

    } else w.send(200, "text/plain", "Usage: /delay?t=n where n is 0..9");
  });
  wifi_set_sleep_type(NONE_SLEEP_T); 
} // setup

void loop() {
  yield();

  // integrating power over one 60Hz wave period
  static avp::TimePeriod1<1000000UL / 60 / NumSamplesPerWL, micros> TP;
  static uint8_t CurPort = 0;
  static uint8_t SampleI = 0;
  static bool GiveLoopToWiFi;

  if(TP.Expired() && (GiveLoopToWiFi = !GiveLoopToWiFi)) {
    ConnectPort(CurPort);
    uint16_t Current = analogRead(ANALOG_PIN);
    ConnectPort(VoltagePortI);
    uint16_t Voltage = analogRead(ANALOG_PIN);

    Integral[CurPort].Power += Voltage * Current;
    Integral[CurPort].Voltage += Voltage;
    Integral[CurPort].Current += Current;
    Integral[CurPort].NumSamples++;
    if(++SampleI == NumSamplesPerWL) {
      SampleI = 0;
      if(++CurPort == NUM_ports) {
        CurPort = 0;
        ++NumScans;
      }
    }
  } else a->loop();
} // loop
