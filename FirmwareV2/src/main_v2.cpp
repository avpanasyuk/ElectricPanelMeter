
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <mDNSResolver.h>
#include "C_General/Macros.h"
#include "C_General/MyMath.hpp"
#include "C_General/MyTime.hpp"
#include "C_ESP/HTML_Log.hpp"
#include "C_ESP/StaticWebServer.hpp"
#include "C_ESP/fast_gpio.hpp"
#include "C_ESP/RemoteLog.hpp"

using WebSrv = avp::StaticWebServer; // 'Server' collides with ESP8266 core's global class Server
using BSDLog = avp::RemoteLog<>;     // POSTs <filename>,<csv> to bsd's http_server.py

#ifndef CONF_VERSION
#define CONF_VERSION 0 // configuration-version digit; overridden per-env in platformio.ini
#endif
#ifndef NTP_TZ
// US Eastern with DST. Change in platformio.ini build_flags if you're elsewhere.
#define NTP_TZ "EST5EDT,M3.2.0/2,M11.1.0/2"
#endif

#define DEBUG_SERIAL Serial

extern "C" {
  int debug_puts(const char *s) {
    avp::HTML_Log::Add(s, /*AddBreak=*/true); // <br> between log entries so /log is readable
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
    float Power = (I.NumSamples > 0) ? (I.Power - I.Current * I.Voltage / I.NumSamples) / I.NumSamples : 0;
    s += String(Power);
    s += "<br>";
    I.Power = I.Current = I.Voltage = 0.;
    I.NumSamples = 0;
  }
  return (s);
} // samples2string

// CSV-formatted twin of samples2string(); consumes+resets accumulators just like /read.
// During the transitional period both readout_*.py polling /read AND this push are active;
// whichever fires first gets a full interval, the other gets a near-empty one. After the
// readout_*.py services are retired, push owns the data path cleanly.
static const String &samples2csv_and_reset() {
  static String s;
  s.reserve(200);
  s = "";
  for(auto &I : Integral) {
    if(s.length()) s += ",";
    float Power = (I.NumSamples > 0) ? (I.Power - I.Current * I.Voltage / I.NumSamples) / I.NumSamples : 0;
    s += String(Power);
    I.Power = I.Current = I.Voltage = 0.;
    I.NumSamples = 0;
  }
  return s;
} // samples2csv_and_reset

// Try hostByName(bare), then hostByName(bare.test), then madpilot mDNS for bare.local.
static IPAddress resolveHost(const char *bare) {
  IPAddress ip;
  if(WiFi.hostByName(bare, ip)) return ip;
  if(WiFi.hostByName((String(bare) + ".test").c_str(), ip)) return ip;
  WiFiUDP udp;
  mDNSResolver::Resolver r(udp);
  ip = r.search((String(bare) + ".local").c_str());
  return (ip != INADDR_NONE) ? ip : IPAddress();
} // resolveHost

// Build the rotating logfile name: PowerMonitor.v<CONF_VERSION>.<MM.YY>.<main|sub>.csv.
// Matches the existing readout_main.py / readout_sub.py output naming. Returns NULL if
// NTP hasn't synced yet (caller should skip the push and try next interval).
static const char *current_logfile_name() {
  static char fname[64];
  time_t now = time(nullptr);
  struct tm *t = localtime(&now);
  if(t->tm_year + 1900 < 2020) return nullptr; // NTP not synced
  const char *type = (VERSION == 1) ? "main" : "sub";
  snprintf(fname, sizeof(fname), "PowerMonitor.v%d.%02d.%02d.%s.csv",
           CONF_VERSION, t->tm_mon + 1, (t->tm_year + 1900) % 100, type);
  return fname;
} // current_logfile_name

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Serial port connected!");

  SetPins();

  // tell the connection-status blinker which pin to drive (default Blinken in DefaultOpts())
  avp::StaticWiFi_Conn::LED_pin = LED_PIN;

  auto Opts = WebSrv::DefaultOpts();
  Opts.Name = NAME; // NAME should be specified in platformio.ini, so it is in sync with upload_port in espota
  Opts.Version = "5.03"; // re-resolve bsd on a cadence; /read zero-guard; C_ESP OTA-stall watchdog
  Opts.AddUsage = F("<li><a href='/read'>read</a> - returns column of power value for each port</li>"
                    "<li><a href='/scan'>scan</a> - returns all samples collected so far</li>"
                    "<li> port?i=n - reads port n and returns its value</li>");

  WebSrv::on("/read", []() {
    static String s;
    s.reserve(200); // reserve buffer for response to avoid dynamic memory allocation
    s = "<html>";
    s += samples2string();
    s += "<br></html>";
    WebSrv::send("text/html", s);
    avp::TogglePin<LED_PIN>();
 });
  WebSrv::on("/scan", []() {
    static String Resp;
    Resp.reserve(200);

    Resp = "<html>Scan N: ";
    Resp += NumScans;
    Resp += F("<br>----------------------------------<br>");
    Resp += samples2string();
    Resp += "<br></html>";
    WebSrv::send("text/html", Resp);
   });
  WebSrv::on("/port", []() {
    if(WebSrv::s.hasArg("i")) {
      auto Port = WebSrv::s.arg("i").toInt();
      if(Port < 0 || Port >= NUM_ports) WebSrv::send("text/plain", F("Wrong port number!"));
      else {
        ConnectPort(Port);
        static String Content;
        Content.reserve(80);
        Content = "Port  = ";
        Content += Port;
        Content += ", ";
        Content += analogRead(ANALOG_PIN);

        WebSrv::send("text/plain", Content);
      }
    } else WebSrv::send("text/plain", F("Usage: /port?i=n where n is port number"));
  });

  WebSrv::begin(Opts); // sets up WiFi (state machine), OTA, default handlers, then s.begin()

  // Disable WiFi modem-sleep so the chip reliably receives unicast traffic
  // (the AiMesh node loses unicast packets to sleeping clients).
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  // NTP -- needed to construct the monthly-rotating filename for the BSD push.
  // configTime is async; loop's push checks time(nullptr) and skips until sync lands.
  configTime(NTP_TZ, "pool.ntp.org");

  // Don't call BSDLog::begin yet -- StaticWiFi_Conn is async, WiFi.localIP isn't
  // valid yet, and we need to resolve "bsd" first. loop() handles it once WiFi
  // is up. RemoteLog::postf is a no-op while URL is unset, so no race.

  debug_puts("Logging here...");
} // setup

void loop() {
  yield();

  // integrating power over one 60Hz wave period
  static avp::TimePeriod1<1000000UL / 60, micros> TP;
  static uint8_t CurPort = 0;
  static uint16_t Counter; ///< we are counting 60 Hz WL periods. During Counter == 0
  // we are reading one port as fast as possible, then we skip WIFI_timeRatio WLs for WiFi to do its thing
  static constexpr uint16_t WIFI_timeRatio = 10; // ratio of WiFi time to analog read time

  if((Counter == 0) && !WebSrv::OTA_IsInProgress) {
    ConnectPort(CurPort);
    uint16_t Current = analogRead(ANALOG_PIN);
    ConnectPort(VoltagePortI);
    uint16_t Voltage = analogRead(ANALOG_PIN);

    Integral[CurPort].Power += Voltage * Current;
    Integral[CurPort].Voltage += Voltage;
    Integral[CurPort].Current += Current;
    Integral[CurPort].NumSamples++;
  } else WebSrv::call_in_loop();
  if(TP.Expired()) {
    if(Counter == 0) {
      if(++CurPort == NUM_ports) {
        CurPort = 0;
        ++NumScans;
      }
      Counter = WIFI_timeRatio;
    } else --Counter;
  }

  // Resolve bsd and keep the IP fresh. DHCP can move bsd, and a meter may run
  // for months, so we re-resolve on a cadence (never cache the IP for the whole
  // boot) and re-point the logger whenever the IP changes. A stale cached IP
  // makes every 5s push fail, and the repeated failed connects eventually wedge
  // the heap -- so the device must self-heal when bsd moves.
  static String bsdURL;
  static bool firstTry = true;
  static avp::TimePeriod1<15000, millis> resolveTO; // first try is immediate; then re-check every 15s
  if(WiFi.status() == WL_CONNECTED && (firstTry || resolveTO.Expired())) {
    firstTry = false;
    IPAddress ip = resolveHost("bsd");
    if((uint32_t)ip != 0) {
      String url = "http://" + ip.toString() + ":8000/";
      if(url != bsdURL) { // first resolve, or bsd's IP changed
        bsdURL = url;
        BSDLog::begin(bsdURL.c_str());
        debug_printf("bsd -> %s", bsdURL.c_str());
      }
    }
  }

  // POST a CSV row to bsd every 5 s, matching readout_*.py polling cadence.
  // Skipped while OTA is in progress (no network competition) and while NTP
  // hasn't synced (no valid date to build the filename) and while BSDLog URL
  // hasn't been resolved yet (postf is also a no-op in that case).
  static avp::TimePeriod1<5000, millis> PushTP;
  if(PushTP.Expired() && !WebSrv::OTA_IsInProgress && bsdURL.length() > 0) {
    if(const char *fname = current_logfile_name()) {
      BSDLog::postf(fname, "%s", samples2csv_and_reset().c_str());
      avp::TogglePin<LED_PIN>(); // visual heartbeat for the push, mirrors /read
    }
  }
} // loop
