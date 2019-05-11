#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <General/Macros.h>
#include <General/Math.h>
#include <stdint.h>
#include <stdio.h>

/////////////////////
// Pin Definitions //
/////////////////////
const uint8_t LED_PIN = D8; // Thing's onboard, green LED
const int ANALOG_PIN = A0;  // The only analog pin on the Thing
constexpr uint8_t NumCtrlLines_74HC4051 = 3;
constexpr uint8_t Num74HC4051 = 2;
static bool RunSampling = true;

static constexpr uint32_t GetDelay(uint8_t DelIndex) {      // exponentially scaled delay
  return 10UL * pow(2, DelIndex); // us
} // GetDelay

// WF acronim is WaveForm, one 60 Hz period
static constexpr uint16_t SAMPLES_IN_WF = 600;
static constexpr uint32_t WF_SAMPLE_PERIOD =
    1000000UL / SAMPLES_IN_WF; // microseconds

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
  for (uint8_t BitI = 0; BitI < NumCtrlLines_74HC4051; BitI++)
    digitalWrite(ControlLines[SwitchNum][BitI], (c >> BitI) & 1);
} // Set74HC4051_code

static inline bool is_past(uint32_t time_us) {
  return (micros() - time_us) < (1UL << 31);
} // is_past

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
static inline void sample_port_and_go_to_next() {
  static uint8_t CurPort = 0;

  ConnectPort(VoltagePortI);
  uint32_t Voltage = analogRead(ANALOG_PIN); // it is 32-bit to avoid overflow on multiplication
  uint32_t SampleUntil = micros() + 1000000UL / 60; // sample for one 60Hz wave period
  while (SampleUntil - micros() < (uint32_t(0) - 1) / 2) {
    ConnectPort(CurPort);
    uint16_t Current = analogRead(ANALOG_PIN);
    ConnectPort(VoltagePortI);
    auto PrevVoltage = Voltage;
    Voltage = analogRead(ANALOG_PIN);
    PrevVoltage = (PrevVoltage + Voltage) / 2;
    // we are trying to remove phase shift between
    // current and voltage by linearlky interpolating voltage sample before and
    // after current
    Integral[CurPort].Power += PrevVoltage * Current;
    Integral[CurPort].Voltage += PrevVoltage;
    Integral[CurPort].Current += Current;
    Integral[CurPort].NumSamples++;
    // yield();
  }
  if (++CurPort == NUM_ports) {
    CurPort = 0;
    ++NumScans;
  }
} // sample_port_and_go_to_next

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

  for (uint8_t i = 0; i < AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, WiFiAPPSK);
}

namespace AVP {
//! writes or reads byte by byte in sequence
class EEPROM_ {
  static uint16_t CurPos;
  static uint8_t CS; //!< is written to EEPROM as bitwise NOT, so if EEPROM is
  //! all 0 check fails
public:
  static void SetPos(uint16_t Pos = 0) {
    CurPos = Pos;
    CS = 0;
  }
  static void write(uint8_t value) {
    // Serial.print(value,DEC); Serial.print(',');
    EEPROM.write(CurPos++, value);
    // delay(100);
    CS += value;
  } // update
  static uint8_t read() {
    uint8_t Out = EEPROM.read(CurPos++);
    // Serial.print(Out,DEC);  Serial.print(',');
    CS += Out;
    return Out;
  } // read
  static void WriteString(const String &str) {
    write(str.length());
    for (uint8_t ByteI = 0; ByteI < str.length(); ByteI++)
      write(str.charAt(ByteI));
    write(~CS);
  } // WriteString
  static String ReadString() {
    String Out;
    uint8_t Size = read();
    for (; Size; Size--)
      Out.concat(char(read()));
    uint8_t InvCS = ~CS;
    return InvCS == read() ? Out : String(); // we have to do it this way
    // because read() changes CS
  } // readString
};  // EEPROM
uint16_t EEPROM_::CurPos;
uint8_t EEPROM_::CS;
} // namespace AVP

void setup() {
  Serial.begin(115200);
  Serial.println();

SetPins();
  delay(3000);

  EEPROM.begin(514);
  AVP::EEPROM_::SetPos();
  String SSID = AVP::EEPROM_::ReadString();
  String Pass = AVP::EEPROM_::ReadString();
  EEPROM.end();
  if (!SSID.length() || !Pass.length()) {
    Serial.println("Can not read from EEPRON, setting to default!");
    SSID = "T2_4";
    Pass = "group224";
  }

  setupWiFi();
  Serial.printf("Connecting to %s ...", SSID.c_str());

  WiFi.begin(SSID.c_str(), Pass.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  server.begin();
  Serial.printf("Web server started, open %s in a web browser\n",
                WiFi.localIP().toString().c_str());
  server.begin();
  ConnectPort(VoltagePortI);
} // setup

static WiFiClient client;

static void reply_and_stop(const String &message) {
  String s(F("HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n"
             "Connection: close\r\n" // the connection will be closed after
                                     // completion of the response
             "Refresh: 5\r\n\r\n" // refresh the page automatically every 5 sec
             "<!DOCTYPE HTML>\r\n<html>\r\n"));
  s += message;
  s += "<br></html>";
  client.println(s);
  // client.flush();
  delay(1);
#ifdef DEBUG
  Serial.println("Sent:" + message);
  Serial.println("Client disconnected.");
#endif
  client.stop();
} // reply_and_stop

static uint16_t WF_sampling_count = 0;

static void sample_waveform() {
  static uint16_t Waveform[SAMPLES_IN_WF];
  // Serial.printf("%lu-%lu..", micros(), EndDelay);

  if (is_past(NewSample)) {
    NewSample = micros() + WF_SAMPLE_PERIOD; // to keep rate
    Waveform[--WF_sampling_count] = analogRead(ANALOG_PIN);
    // Serial.printf("%d..", WF_sampling_count);
    if (WF_sampling_count == 0) {
      String s;
      for (uint16_t SampleI = 0; SampleI < SAMPLES_IN_WF; ++SampleI)
        s += String(Waveform[SampleI]) + " ";
      reply_and_stop(s);
    }
  }
} // sample_waveform

static String samples2string() {
  String s;
  for (uint8_t CurPort = 0; CurPort < NUM_ports; ++CurPort) {
    auto &I = Integral[CurPort];
    float Power =
        (I.Power - I.Current * I.Voltage / I.NumSamples) / I.NumSamples;
    s += String(Power) + "<br>";
    I.Power = I.Current = I.Voltage = 0.;
    I.NumSamples = 0;
  }
  return (s);
} // samples2string

void loop() {
  yield();
  if (client) { // client did not disconnect in previuos loop, it means that we
    // are sampling WF, at most one sample per loop
    if (WF_sampling_count) sample_waveform();
  } else { // we are not sampling WF
    delay(5);
    if (RunSampling) sample_port_and_go_to_next();

    // Check if a client has a Request
    client = server.available();
    if (client) {
      static uint8_t LEDstate = 1;
      digitalWrite(LED_PIN, LEDstate = 1 - LEDstate);

      // Read the first line of the request
      String req = client.readStringUntil('\r');
      if(req.length() == 0) {
        delay(100);
        req = client.readStringUntil('\r');
      }
      if(req.length() == 0) {
        client.stop();
        return;
      }

#ifdef DEBUG
      Serial.println("Received: " + req);
#endif
      client.flush();

      // parse request
      int Port;

      if (req.lastIndexOf("/read") != -1)
        reply_and_stop(samples2string());
      else if (req.lastIndexOf("/scan") != -1) {
        String s("Scan N: " + String(NumScans) +
                 "<br>----------------------------------<br>");
        s += samples2string();
        reply_and_stop(s);
      } else if ((Port = req.lastIndexOf("/port")) != -1) {
        // let's read port number
        Port = req.charAt(Port + strlen("/port/")) - '0';
        if (Port < 0 || Port > NUM_ports)
          reply_and_stop(String(F("Wrong port number!")));
        else {
          ConnectPort(Port);
          String s("Port  = ");
          s += String(Port) + ", " + String(analogRead(ANALOG_PIN));
          reply_and_stop(s);
        }
      } else if ((Port = req.lastIndexOf("/wave")) != -1) {
        Port = (req.charAt(Port + strlen("/wave/")) - '0');
        if (Port < 0 || Port > NUM_ports)
          reply_and_stop(String(F("Wrong port number!")));
        else {
          ConnectPort(Port);
          WF_sampling_count = SAMPLES_IN_WF;
        }
      } else if ((Port = req.lastIndexOf("/delay")) != -1) {
        Delay = GetDelay(req.charAt(Port + strlen("/delay/")) - '0');
        reply_and_stop(String(F("Delay is set to ")) + String(Delay) +
                       " microseconds.");
      } else if ((Port = req.lastIndexOf("/ctrl")) != -1) {
        Port = (req.charAt(Port + strlen("/ctrl/")) - '0');
        switch (Port) {
        case 2: { // how fast analogRead is. Seems to be 100 us.
          uint32_t Start = micros();
          for (uint16_t n = 10000; n; --n) {
            analogRead(ANALOG_PIN);
          }
          reply_and_stop(String(F("10000 loops over ")) +
                         String(micros() - Start) + " us");
          break;
        }
        case 0:
        case 1:
          RunSampling = Port;
          reply_and_stop(String(F("Continuos sampling turned ")) +
                         String(Port ? "on" : "off"));
          NewSample = micros();
          break;
        default:
          reply_and_stop(String(F("Unknown control command!")));
          break;
        }
      } else if ((Port = req.lastIndexOf("/wifi")) != -1) {
        String IDs = req.substring(Port + strlen("/wifi/"));
        int Colon = IDs.indexOf(':');
        int End = IDs.indexOf(' ');
        String SSID = IDs.substring(0, Colon);
        String Pass = IDs.substring(Colon + 1, End);
        EEPROM.begin(514); // 255 + 1 /* size */ + 1 /* CS */ )*2
        AVP::EEPROM_::SetPos();
        AVP::EEPROM_::WriteString(SSID);
        AVP::EEPROM_::WriteString(Pass);
        // EEPROM.commit();
        EEPROM.end();
        reply_and_stop(String(F("WIFI is set to ")) + SSID + ":" + Pass);
      } else if (req.lastIndexOf("/favicon.ico") != -1) client.stop();
      else reply_and_stop(
            String("Invalid Request:<") + req + F(">. Try /read or /scan or /port/? or /wave/? "
              "or /ctrl/? or /wifi/ssid:password."));
    } // new client request
  }   // client was closed on previous loop
} // loop
