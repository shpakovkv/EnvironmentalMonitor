#include <Arduino.h>
#include <GyverTM1637.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
// #include <string.h>

// pins
#define THERM_PIN A1
#define THERM_REF_T 25 // T0 [°K]
#define THERM_REF_R 10000 // R0 [Ohm]
#define THERM_B_COEF 3950 // B coeficient [°K]
#define OTHER_R 11110 // [Ohm]

#define T_SMOOTH_READS 5  //
#define T_SMOOTH_READS_DELAY 10   // delay between reads in milliseconds

#define DEGREE_CHAR 0 // index of custom character
#define LCD_LINES 2
#define LCD_COLS 16
#define LCD_ADDRESS 0x27
#define LCD_BRIGHT_BUT_PIN 3
#define LCD_BRIGHT_SET_PIN 6

#define DEBOUNCE_DELAY 20 // milliseconds

#define UPDATE_PERIOD 500

#define ONE_WIRE_BUS 2 // dstemp

// degree character hex code
const uint8_t DEGREE_HEX[] {0x0C, 0x12, 0x12, 0x0C, 0x00, 0x00, 0x00, 0x00};

class NTC
{
  // HARDWARE:
  // connect Vcc to external voltage reference pin and set analogReference(EXTERNAL)
  // connect reference resistance to Vcc and NTC thermistor
  // then connect other leg of NTC thermistor to ground
  // then connect middle point (between resistance and NTC) to analog input pin

  public:
    NTC(int analogPin, uint32_t referenceResistance, double referenceTemperature, double coefficientB, uint32_t otherResistance, int smoothReadsNumber=5, int smoothReadsDelay=5)
    {
      ntcPin = analogPin;
      ref_R = referenceResistance;      // reference resistance in Ohms (at reference temperature)
      ref_T = referenceTemperature;     // reference temperature in °C
      coef_B = coefficientB;            // NTC thermistor B coefficient for Steinhart-Hart equation
      other_R = otherResistance;        // the other resistance in the NTC circuit
      smoothReads = smoothReadsNumber;  // number of read operations to average
      smoothDelay = smoothReadsDelay;   // delay in milliseconds between reads
      pinMode(analogPin, INPUT);
      // TODO: check analog reference
    }

    // ~NTC()
    // {
    //   delete samples;
    // }

    double Read()
    {
      // slow function
      // reads and returns NTC temperature in °C
      ReadRawAverage();
      ResistanceFromRaw(currentRaw);
      return TempFromResistance(currentResistance);
    }

    void PrintStats()
    {
      // prints to serial:
      // raw, resistance, temperature
      String message = String(currentRaw);
      message += ",  " + String(currentResistance) + " Ohm";
      message += ",  " + String(currentTemperature);
      message += " " + String('\xB0') + "C";
      Serial.println(message);
    }

    double ReadRawAverage()
    {
      // slow function
      // reads NTC voltage in arduino ADC raw values
      // repeats reads smoothReads times and return average value
      currentRaw = 0;
      for (int i=0; i<smoothReads; i++)
      {
        currentRaw += analogRead(ntcPin);
        delay(smoothDelay);
      }
      currentRaw /= smoothReads;
      return currentRaw;
    }

    double ResistanceFromRaw(double raw)
    {
      // calculates NTC resistance from arduino ADC raw value
      currentResistance = 1023.0 / raw - 1.0;
      currentResistance = other_R / currentResistance;
      // currentResistance = raw * other_R;
      // currentResistance /= (1023 - raw);
      return currentResistance;
    }

    double TempFromResistance(double resistance)
    {
      // calculates NTC temperature from measured resistance
      //double temp = 1.0/(1/To + log(R/Ro)/B);
      currentTemperature = resistance / ref_R;          // R/Ro
      currentTemperature = log(currentTemperature);     // ln(R/Ro)
      currentTemperature /= coef_B;                     // ln(R/Ro)/B
      currentTemperature += 1.0 / (ref_T + 273.15);     // + (1/To)
      currentTemperature = 1.0 / currentTemperature;    // 1/T
      currentTemperature -= 273.15;                     // convert to °C

      // steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
      // steinhart = log(steinhart);                  // ln(R/Ro)
      // steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
      // steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
      // steinhart = 1.0 / steinhart;                 // Invert
      // steinhart -= 273.15;                         // convert to C

      return currentTemperature;
    }

  private:
    int ntcPin;                // analog pin
    uint32_t ref_R;         // reference resistance in Ohms (at reference temperature)
    double ref_T;           // reference temperature in °C
    double coef_B;          // NTC thermistor B coefficient for Steinhart-Hart equation
    uint32_t other_R;       // the other resistance in the NTC circuit
    int smoothReads=5;      // number of read operations to average
    int smoothDelay=10;     // delay in milliseconds between reads

  public:
    int currentRaw;
    double currentResistance;
    double currentTemperature;
};


// function declaration
void displayLine(LiquidCrystal_I2C display, int lineNumber, String line);
void setLCDBrightness(unsigned int level, int brightnessPin=LCD_BRIGHT_SET_PIN);
void printStats(unsigned int rawRead,           // raw value
                unsigned long long resistance,  // NTC measured resistance [Ohm]
                double temperature);            // NTC temperature [°C]
void printAddress(DeviceAddress deviceAddress);

// objects
LiquidCrystal_I2C lcd_display(LCD_ADDRESS, LCD_COLS, LCD_LINES);
// Set the LCD address to 0x27 for a 16 chars and 2 line display

long brightLastPress = 0;
bool brightIsPressed = false;
bool stringComplete = false;
unsigned int lcdBright = 255;
long tempLastReadTime;
String serialCommand = "";
int serialLedStatus = LOW;
bool updateLCD = false;
bool UpdateByTime = true;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature DSSensors(&oneWire);

// arrays to hold device address
DeviceAddress DSThermometer;

NTC NTCSensor(THERM_PIN, THERM_REF_R, THERM_REF_T, THERM_B_COEF, OTHER_R, T_SMOOTH_READS, T_SMOOTH_READS_DELAY);

void setup() {
  analogReference(EXTERNAL);
  pinMode(LCD_BRIGHT_BUT_PIN, INPUT);   // buttom input
  pinMode(LCD_BRIGHT_SET_PIN, OUTPUT);  // PWM output for LCD power
  setLCDBrightness(lcdBright);          // initialize PWM output
  // brightLastPress = millis()

	lcd_display.begin();           // initialize the LCD
	lcd_display.backlight();       // Turn on the blacklight
	lcd_display.print("Hello!");   // and print a message.

  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);

  // create degree character
  lcd_display.createChar(DEGREE_CHAR, DEGREE_HEX);

  serialCommand.reserve(100);
  Serial.begin(9600);
  delay(100);

  DSSensors.begin();

  // Serial.print("Locating devices...");
  DSSensors.begin();
  // Serial.print("Found ");
  // Serial.print(DSSensors.getDeviceCount(), DEC);
  // Serial.println(" devices.");
  DSSensors.setResolution(DSThermometer, 12);
  // if (!DSSensors.getAddress(DSThermometer, 0)) Serial.println("Unable to find address for Device 0");

  // Serial.print("1-Wire device 0 Address: ");
  // printAddress(DSThermometer);
  // Serial.println();
}

void loop() {

  if (UpdateByTime && millis() - tempLastReadTime > UPDATE_PERIOD)
  {
    DSSensors.requestTemperatures();
    NTCSensor.Read();
    updateLCD = true;
    tempLastReadTime = millis();
  }

  if (updateLCD)
  {
    String line;
    line.reserve(LCD_COLS);

    line = "Int. ";
    line += String(NTCSensor.currentTemperature, 2);
    line += " ";
    // '\xB0' == '°' and don't use "\xB0SomeText" it is not working here
    line += '\xB0';
    line += "C";
    displayLine(lcd_display, 0, line);

    // display stats in LCD
    line = "Ext. ";
    line += String(DSSensors.getTempCByIndex(0), 2);
    line += " ";
    line += '\xB0';
    line += "C";
    displayLine(lcd_display, 1, line);

    updateLCD = false;
  }

  // button press event
  if (digitalRead(LCD_BRIGHT_BUT_PIN) == HIGH && millis() - brightLastPress > DEBOUNCE_DELAY)
  {
    brightIsPressed = true;
    brightLastPress = millis();
    lcdBright -= 15;
    if (lcdBright < 15) lcdBright = 255;
    setLCDBrightness(lcdBright);
    Serial.print("Brightness = ");
    Serial.println(lcdBright);
  }

  if (stringComplete)
  {
    // temperature in °C request
    if (serialCommand == "TC")
    {
      NTCSensor.Read();
      DSSensors.requestTemperatures();
      Serial.print(NTCSensor.currentTemperature, 3);
      Serial.print(" ");
      Serial.println(DSSensors.getTempCByIndex(0), 3);
      updateLCD = true;
    }
    // switch to "update values by serial command" mode
    else if (serialCommand == "EXT") UpdateByTime = false;
    // switch to "update values continuously by time" mode
    else if (serialCommand == "INT") UpdateByTime = true;
    // identification string request
    else if (serialCommand == "IDN") Serial.println("Arduino env. monitor v1.0");
    // all other inputs
    else Serial.println("Unknown command.");

    serialCommand = "";
    stringComplete = false;
    serialLedStatus = LOW;
  }

  digitalWrite(11, serialLedStatus);
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

double tempFromResistance(unsigned long long resistance)
{
  //double temp = 1.0/(1/THERM_REF_T + log(resistance / THERM_REF_R) / THERM_B_COEF);
  double temp;
  temp = log(float(resistance) / THERM_REF_R);
  temp /= THERM_B_COEF;
  temp += 1.0 / THERM_REF_T;
  temp = 1.0 / temp;
  temp -= 273.15;
  return temp;
}

void displayLine(LiquidCrystal_I2C display, int lineNumber, String line)
{
  // if (line.length() > ???) {TRIM LINE}  TODO
  if (lineNumber > LCD_LINES - 1) { return;}

  char * buffer = new char [line.length()+1];
  strcpy (buffer, line.c_str());
  display.setCursor(0, lineNumber);
  display.printstr(buffer);

  for (int i = 0; i < line.length(); i++)
  {
    if (line[i] == '\xB0'){
      display.setCursor(i, lineNumber);
      display.write(DEGREE_CHAR);
    }
  }
  delete buffer;
}

void setLCDBrightness(unsigned int level, int brightnessPin)
{
  if (level < 256) { analogWrite(brightnessPin, level);}
}

void serialEvent() {
  serialLedStatus = HIGH;
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '?' || inChar == '!') stringComplete = true;
    // add all other bytes to the inputString:
    else serialCommand += inChar;
  }
}
