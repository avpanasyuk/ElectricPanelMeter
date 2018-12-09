#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <General/Macros.h>
#include <General/Math.h>
#include <stdint.h>
#include <stdio.h>

//////////////////////
// WiFi Definitions //
//////////////////////
const char WiFiAPPSK[] = "esp54321";

/////////////////////
// Pin Definitions //
/////////////////////
const uint8_t LED_PIN = D8;     // Thing's onboard, green LED
const int ANALOG_PIN = A0; // The only analog pin on the Thing
constexpr uint8_t NUM_I_ports = 14;

// static constexpr float CalibCoeff = 22.; // that's for calibration with heaters

// FIXME what is the "theoretical" value?
// transformer under load 122.4/12.52
// divider J1 to ADC = 0.025, divider outlet to ADC = 2.56e-3
// divider current to ADC = 0.16824
// ADC 1024 units to 1V, so there is 2.62 adc units for 1 outlet V, and
// 172.3 unit per 1V Vcurrent
// with SCT013 20A/1V scale, it is 8.61 ADC units per outlet A.
// so, taken together it is 8.61*2.62 = 22.6. We do not need to convert
// amplitude to RMS, happens automatically during integration
static constexpr float CalibCoeff = 22.6;


static constexpr uint8_t NumCtrlLines_74HC4051 = 3;
static constexpr uint8_t ControlLines[2][NumCtrlLines_74HC4051] = {
    {D2, D1, D0}, {D5, D7, D6}}; // numbers of NodeMCU IO ports
static constexpr uint8_t Disable74HC4051_port[] = {D3, D4};

static bool RunSampling = true;

static constexpr uint32_t GetDelay(uint8_t DelIndex) {      // exponentially scaled delay
  return 10UL * pow(2, DelIndex); // us
} // GetDelay

// WF acronim is WaveForm, one 60 Hz period
static constexpr uint16_t SAMPLES_IN_WF = 600;
static constexpr uint32_t WF_SAMPLE_PERIOD =
    1000000UL / SAMPLES_IN_WF; // microseconds

static uint32_t Delay = GetDelay(5); // tuned for no visible phase shift. Hmm, HOW?
static uint32_t NewSample = micros();

static inline void Set74HC4051_code(uint8_t c, uint8_t IC_num) {
  for (uint8_t BitI = 0; BitI < NumCtrlLines_74HC4051; BitI++)
    digitalWrite(ControlLines[IC_num][BitI], (c >> BitI) & 1);
} // Set74HC4051_code

static inline void ConnectPort(uint8_t port, uint8_t IC_num) {
  digitalWrite(Disable74HC4051_port[1 - IC_num],1); // disable other switch
  Set74HC4051_code(port, IC_num); // connect the line we want
  digitalWrite(Disable74HC4051_port[IC_num],0); // enable this switch
  delayMicroseconds(Delay);
  yield();
} // ConnectPort

static inline void ConnectI_Port(uint8_t port) {
  if(port >= (NUM_I_ports >> 1)) ConnectPort(port - (NUM_I_ports >> 1),1);
  else ConnectPort(port + 1,0);
} // ConnectCurrentPort

static void ConnectGND_Port() { ConnectPort(7,1); }
static void ConnectV_Port() { ConnectPort(0,0); }

static inline bool is_past(uint32_t time_us) {
  return (micros() - time_us) < (1UL << 31);
} // is_past

static uint32_t ScanTime; // ms
static float Power[NUM_I_ports];
// integration data
static struct { float Power, Voltage, Current; } Integral;
static uint8_t NumSamplesPerPort = 200;
static uint32_t NumScans;

static void sample_port() {
  static uint8_t CurPort = 0;
  static uint16_t Current;
  static uint32_t Voltage; // it is 32-bit to avoid overflow on multiplication
  static uint32_t ScanStartedMark;
  static float GND_Pwr = 0;

  ScanStartedMark = millis();
  ConnectV_Port();
  Voltage = analogRead(ANALOG_PIN);
  for (uint8_t SamplI = 0; SamplI < NumSamplesPerPort; SamplI++) {
    ConnectI_Port(CurPort);
    Current = analogRead(ANALOG_PIN);
    ConnectV_Port();
    auto PrevVoltage = Voltage;
    Voltage = analogRead(ANALOG_PIN);
    PrevVoltage = (PrevVoltage + Voltage) / 2; // we are trying to remove phase shift between
    // current and voltage by linearlky interpolating voltage sample before and after current
    Integral.Power += PrevVoltage * Current;
    Integral.Voltage += PrevVoltage;
    Integral.Current += Current;
  }

  // let's evaluate noise on GND line
  ConnectGND_Port();
  uint16_t GND_I = analogRead(ANALOG_PIN);
  ConnectV_Port();
  Voltage = (Voltage + analogRead(ANALOG_PIN))/2;
  GND_Pwr = GND_Pwr?GND_I*Voltage:(GND_Pwr*0.999 + GND_I*Voltage*0.001); // averaging

  Power[CurPort] = (Integral.Power -
                    Integral.Current * Integral.Voltage / NumSamplesPerPort) /
                   NumSamplesPerPort - GND_Pwr; // subtracting noise

  if(Power[CurPort] < 0) Power[CurPort] = 0;

  Integral.Power = Integral.Current = Integral.Voltage = 0.;
  if (++CurPort == NUM_I_ports) {
    CurPort = 0;
    ++NumScans;
    ScanTime = millis() - ScanStartedMark;
  }
} // sample_port

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

  for (uint8_t PinI = 0; PinI < 6; PinI++)
    pinMode(ControlLines[0][PinI], OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(Disable74HC4051_port[0], OUTPUT);
  pinMode(Disable74HC4051_port[1], OUTPUT);
  digitalWrite(LED_PIN, 0);
} // initHardware

int debug_vprintf(const char *format, va_list a) {
  static constexpr uint8_t BUFFER_SIZE = 255;
  static char Buffer[BUFFER_SIZE];
  Serial.println(vsnprintf(Buffer, BUFFER_SIZE, format, a));
  return 1;
} // debug_vprintf

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
  }                                          // readString
};                                           // EEPROM
uint16_t EEPROM_::CurPos;
uint8_t EEPROM_::CS;
} // namespace AVP

void setup() {
  initHardware();
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
  ConnectV_Port();
} // setup

static WiFiClient client;

static void reply_and_stop(const String &message) {
  String s(F("HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n"
             "Refresh: 5\r\n\r\n" // refresh the page automatically every 5 sec
             "<!DOCTYPE HTML>\r\n<html>\r\n"));
  s += message;
  s += "</html>\r\n";
  client.print(s);
  delay(1);
  Serial.println("Client disconnected.");
  client.stop();
} // reply_and_stop

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
      reply_and_stop(s);
    }
    NewSample = micros() + WF_SAMPLE_PERIOD;
  }
} // sample_waveform

static String samples2string(bool Calibrate) {
  String s;
  for (uint8_t p = 0; p < NUM_I_ports; p++)
    s += String(Calibrate ? Power[p] / CalibCoeff : Power[p]) + "<br>";
  return (s);
} // samples2string

void loop() {
  if (client) { // client did not disconnect in previuos loop, it means that we
    // are sampling WF, at most one sample per loop
    if (WF_sampling_count) sample_waveform();
  } else { // we are not sampling WF
    delay(10);
    if (RunSampling) sample_port();

    // Check if a client has a Request
    client = server.available();
    if (client) {
      static uint8_t LEDstate = 1;
      digitalWrite(LED_PIN, LEDstate = 1 - LEDstate);

      // Read the first line of the request
      String req = client.readStringUntil('\r');
      Serial.println(req);
      client.flush();

      // parse request
      int Port;

      if (req.lastIndexOf("/get") != -1) {
        client.print(samples2string(true) + "\n");
        delay(1);
        client.stop();
        Serial.println("Client disconnected.");
      } else if (req.lastIndexOf("/read") != -1) {
        reply_and_stop(samples2string(true));
      } else if (req.lastIndexOf("/sample") != -1) {
        String s("Scan N: " + String(NumScans) + ", Time: " + String(ScanTime) +
                 " ms<br>----------------------------------<br>");
        s += samples2string(false);
        reply_and_stop(s);
      } else if ((Port = req.lastIndexOf("/port")) != -1) {
        // let's read port number
        Port = (req.charAt(Port + strlen("/port/")) - '0');
        if (Port < 0 || Port > 15) {
          reply_and_stop(String(F("Wrong port number! <br>")));
        } else {
          if(Port == 0) ConnectV_Port(); else if(Port == 15) ConnectGND_Port();
          else ConnectI_Port(Port);
          String s("Port  =");
          s += String(Port) + ", " + String(analogRead(ANALOG_PIN));
          reply_and_stop(s);
        }
      } else if ((Port = req.lastIndexOf("/wave")) != -1) {
        Port = (req.charAt(Port + strlen("/wave/")) - '0');
        if (Port < 0 || Port > 15) {
          reply_and_stop(String(F("Wrong port number! <br>")));
        } else {
          if(Port == 0) ConnectV_Port(); else if(Port == 15) ConnectGND_Port();
          else ConnectI_Port(Port);
          WF_sampling_count = SAMPLES_IN_WF;
        }
      } else if ((Port = req.lastIndexOf("/delay")) != -1) {
        Delay = GetDelay(req.charAt(Port + strlen("/delay/")) - '0');
        reply_and_stop(String(F("Delay is set to ")) + String(Delay) +
              " microseconds.<br>");
      } else if ((Port = req.lastIndexOf("/ctrl")) != -1) {
        Port = (req.charAt(Port + strlen("/ctrl/")) - '0');
        switch (Port) {
        case 2: { // how fast analogRead is. Seems to be 100 us.
          uint32_t Start = micros();
          for (uint16_t n = 10000; n; --n) {
            analogRead(ANALOG_PIN);
          }
          reply_and_stop(String(F("10000 loops over ")) + String(micros() - Start) +
                " us");
          break;
        }
        case 0:
        case 1:
          RunSampling = Port;
          reply_and_stop(String(F("Continuos sampling turned ")) +
                String(Port ? "off" : "on"));
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
        reply_and_stop(String(F("WIFI is set to ")) + SSID + ":" + Pass + "<br>");
      } else
        reply_and_stop(String(
            F("Invalid Request.<br> Try /read or /sample or /port/? or /wave/? "
              "or /ctrl/? or /wifi/ssid:password.")));
    }
  }
} // loop
