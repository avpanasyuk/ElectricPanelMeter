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
const char WiFiAPPSK[] = "esp12345";

/////////////////////
// Pin Definitions //
/////////////////////
const int LED_PIN = 5;     // Thing's onboard, green LED
const int ANALOG_PIN = A0; // The only analog pin on the Thing

// hardware information about analog switched connections
static constexpr uint8_t NUM_Inputs = 14;
static constexpr struct tSwitchInput {
  uint8_t PortNum;
  uint8_t IC_num; //!< U2 -> 0, U3 -> 1
} SwitchInput[] = //! ports are in order they are located on P1
    {{2, 1},      // input connected to ground
     {3, 0},      // input connected to Voltage pickup
     {4, 1}, {6, 1}, {7, 1}, {0, 1}, {5, 1}, {3, 1}, // connected to P1
     {4, 0}, {6, 0}, {0, 0}, {1, 0}, {7, 0}, {5, 0}};

static constexpr uint8_t FirstP1input = 2;

// Calibration is done using "Kill-A-Watt" with 0, 300, 750 and 1000 loads
// Calibration data:
#if 0
[[-432 0 -444 0 -421 0  6451 313  6467 312  6440 310 22160 1010 22060 1013 22060 1011 15880 731 15828 732 15827 732],...
[16048 732 16108 731 16111 730 22450 1016 22424 1022 22390 1014 6727 311 6701 311 6705 310 -143 0 -172 0 -191 0],...
[127 0 130 0 85 0 7213 310 7006 308 6979 307 22535 1000 22336 1010 22387 1001 16059 721 16210 723 16153 722],...
[16344 725 16693 724 16614 723 23021 1000 22931 999 22872 999 7345 308 7290 307 7320 306 370 0 435 0 431 0],...
[650 0 738 0 700 0 -6225 307 -6128 305 -6164 305 -21527 999 -21599 1000 -21632 1009 -15881 721 -15454 722 -15436 721],...
[17056 721 17129 723 17194 724 23374 998 23480 1000 23505 1000 7827 306 7847 306 7789 306 1291 0 984 0 990 0],...
[1409 0 1169 0 1205 0 8305 309 8116 307 8167 306 23533 997 23734 1008 23709 996 17815 720 17469 718 17487 722],...
[17793 720 17596 720 17616 721 24120 1003 23940 1001 23910 1001 8206 307 8248 306 8265 307 1267 0 1293 0 1333 0],...
[1426 0 1331 0 1329 0 8370 307 8247 307 8317 307 23936 999 23897 998 23923 999 17439 719 17609 721 17535 719],...
[-15180 718 -15266 719 -15236 719 -21330 997 -21598 996 -21622 994 -5591 307 -5765 306 -5795 308 1207 0 1220 0 1193 0],...
[1028 0 992 0 1034 0 -5955 307 -5949 306 -5968 306 -21911 999 -21724 996 -21725 998 -15592 731 -15637 729 -15652 728],...
[-15740 729 -15767 729 -15817 723 -22108 1004 -21943 1004 -21983 998 -6113 308 -6208 307 -6207 307 1164 0 706 0 783 0]]
#endif

// Power is linearly  proportional to the corresponding "sample" output with
// coeffients
static constexpr float CalCoeff[][2] = {
    {20.543, 0.044872},  {7.875, 0.045003},   {-6.9238, 0.045074},
    {-17.445, 0.044473}, {29.171, -0.04485},  {-46.005, 0.044697},
    {-57.662, 0.044537}, {-57.244, 0.044098}, {-60.402, 0.044346},
    {54.271, -0.043735}, {45.115, -0.043748}, {37.943, -0.043748},
};

; // units = Power*CalCoeffs[1] + CalCoeffs[2]

static constexpr tSwitchInput VoltagePort = SwitchInput[1];
static constexpr uint8_t OpenPort[2] = {2, 1};
static constexpr tSwitchInput GND_port = SwitchInput[0];
// static constexpr tSwitchInput *P1_input = SwitchInput + 2;

static constexpr uint8_t ControlLines[2][3] = {
    {D2, D1, D0}, {D7, D6, D5}}; // numbers of NodeMCU IO ports

static bool RunSampling = true;

static_assert(N_ELEMENTS(SwitchInput) == NUM_Inputs, "Just checking!");

// integration data
static struct Integrals_ {
  float Power, Voltage, Current;
} Integrals[NUM_Inputs]; // two alterrnating buffers

static struct Integrals_ VoltageSqr, GND_integ;

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

static void Set74HC4051_code(uint8_t c, uint8_t IC_num) {
  for (uint8_t BitI = 0; BitI < 3; BitI++)
    digitalWrite(ControlLines[IC_num][BitI], (c >> BitI) & 1);
} // Set74HC4051_code

static void ConnectInput(tSwitchInput n) {
  Set74HC4051_code(n.PortNum, n.IC_num);
  Set74HC4051_code(OpenPort[1 - n.IC_num], 1 - n.IC_num);
  delayMicroseconds(Delay);
}

static inline bool is_past(uint32_t time_us) {
  return (micros() - time_us) < (1UL << 31);
} // is_past

static void start_sampling() {
  NewSample = micros();
  SamplingStarted = millis();
} // start_sampling

static void sample_all_ports() {
  ConnectInput(VoltagePort);
  uint32_t MidVoltage, Voltage = analogRead(ANALOG_PIN);
  for (uint8_t PortI = 0; PortI < NUM_Inputs; ++PortI) {
    uint32_t Current;
    ConnectInput(SwitchInput[PortI]);
    Current = analogRead(ANALOG_PIN);
    ConnectInput(VoltagePort);
    // to compensate for phase shift let's average voltage
    MidVoltage = Voltage;
    Voltage = analogRead(ANALOG_PIN);
    MidVoltage = (MidVoltage + Voltage) / 2;

    Integrals[PortI].Power += MidVoltage * Current;
    Integrals[PortI].Voltage += MidVoltage;
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

  for (uint8_t PinI = 0; PinI < 6; PinI++)
    pinMode(ControlLines[0][PinI], OUTPUT);
  pinMode(LED_PIN, OUTPUT);
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

static String samples2string(uint8_t FirstPort, bool Calibrate) {
  String s;
  for (uint8_t p = FirstPort; p < NUM_Inputs; p++) {
    Integrals_ *pInt = &Integrals[p];
    float Pwr = (pInt->Power - pInt->Current * pInt->Voltage / SampleCount) /
                SampleCount;
    if (Calibrate)
      Pwr = Pwr*CalCoeff[p-FirstPort][1] + CalCoeff[p-FirstPort][0];
    s += String(Pwr) + "<br>";
    pInt->Power = pInt->Current = pInt->Voltage = 0;
  }
  SampleCount = 0;
  return (s);
} // samples2string

void loop() {
  if (client) {
    if (WF_sampling_count)
      sample_waveform();
  } else {
    if (RunSampling && is_past(NewSample)) { // delay is over
      sample_all_ports();
      NewSample = micros() + WF_SAMPLE_INTERVAL * 10;
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

      if (req.lastIndexOf("/read") != -1) {
        reply(samples2string(FirstP1input, true));
      } else if (req.lastIndexOf("/sample") != -1) {
        float SamplingTime = millis() - SamplingStarted;
        String s(String(SamplingTime / SampleCount) + " ms per sample over " +
                 String(SamplingTime) +
                 " ms<br>----------------------------------<br>");
        s += samples2string(0, false);
        SamplingStarted = millis();
        reply(s);
      } else if ((Port = req.lastIndexOf("/port")) != -1) {
        // let's read port number
        Port = (req.charAt(Port + strlen("/port/")) - '0');
        if (Port < 0 || Port > 7) {
          reply(String(F("Wrong port number! <br>")));
        } else {
          ConnectInput(SwitchInput[Port]);
          String s("Port  =");
          s += String(Port) + ", " + String(analogRead(ANALOG_PIN));
          reply(s);
        }
      } else if ((Port = req.lastIndexOf("/wave")) != -1) {
        Port = (req.charAt(Port + strlen("/wave/")) - '0');
        if (Port < 0 || Port > 7) {
          reply(String(F("Wrong port number! <br>")));
        } else {
          ConnectInput(SwitchInput[Port]);
          WF_sampling_count = SAMPLES_IN_WF;
        }
      } else if ((Port = req.lastIndexOf("/delay")) != -1) {
        Delay = GetDelay(req.charAt(Port + strlen("/delay/")) - '0');
        reply(String(F("Delay is set to ")) + String(Delay) +
              " microseconds.<br>");
      } else if ((Port = req.lastIndexOf("/ctrl")) != -1) {
        Port = (req.charAt(Port + strlen("/ctrl/")) - '0');
        switch (Port) {
        case 2: {
          uint32_t Start = micros();
          for (uint16_t n = 10000; n; --n) {
            analogRead(ANALOG_PIN);
          }
          reply(String(F("10000 loops over ")) + String(micros() - Start) +
                " us");
          break;
        }
        case 0:
        case 1:
          RunSampling = Port;
          reply(String(F("Continuos sampling turned ")) +
                String(Port ? "off" : "on"));
          NewSample = micros();
          break;
        default:
          reply(String(F("Unknown control command!")));
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
        reply(String(F("WIFI is set to ")) + SSID + ":" + Pass + "<br>");
      } else
        reply(String(
            F("Invalid Request.<br> Try /read or /sample or /port/? or /wave/? "
              "or /ctrl/? or /wifi/ssid:password.")));
    }
  }
} // loop
