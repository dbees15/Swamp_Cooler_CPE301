#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal.h>
#include <DHT.h>
#include <Servo.h>

class SwampCooler;
class StateInterface;
class DisabledState;
class IdleState;
class RunningState;
class ErrorState;

int getWaterLevel();
int getTemperature();
int getHumidity();

void disableAll();
void setDisabledOutputs();
void setIdleOutputs();
void setRunningOutputs();
void setErrorOutputs();
void updateLCDStats();
void printRTCTime();
void adc_init();
unsigned int adc_read(unsigned char adc_channel_num);

///PORTA Registers
volatile unsigned char* porta = (unsigned char*) 0x22;
volatile unsigned char* pina = (unsigned char*) 0x20;
volatile unsigned char* ddra = (unsigned char*) 0x21;
///Position of LEDs on PORTA
const unsigned char PORTA_YELLOW = 1;
const unsigned char PORTA_GREEN = 3;
const unsigned char PORTA_BLUE = 5;
const unsigned char PORTA_RED = 7;

///Analog read registers
volatile unsigned char *admux = (unsigned char*) 0x7C;
volatile unsigned char *adcsra = (unsigned char*) 0x7A;
volatile unsigned char *adcsrb = (unsigned char*) 0x7B;
volatile unsigned int *adcdata = (unsigned int*) 0x78;

/// True if adc_init() has been called
volatile bool analogInitialized = 0;

/// The pin for the water sensor
const unsigned char WATER_SENSOR_PIN = A0;

/// The pin for the disable button
const unsigned char BUTTON_PIN = 18;

/// The pin for the dht sensor
const unsigned char DHT_PIN = 19;

/// The pin for the servo
const unsigned char SERVO_PIN = 7;

/// The yellow led pin
const unsigned char YELLOW_LED_PIN = 23;
/// The green led pin
const unsigned char GREEN_LED_PIN = 25;
/// The blue led pin
const unsigned char BLUE_LED_PIN = 27;
/// The red led pin
const unsigned char RED_LED_PIN = 29;

/// The motor pin
const unsigned char MOTOR_PIN = 6;

/// The time between lcd updates in ms. The DHT sensor only updates around 1Hz. The LCD also cannot display constantly.
const unsigned long LCD_UPDATE_INTERVAL = 1000;

/// The time between servo updates in ms.
const unsigned long SERVO_UPDATE_INTERVAL = 50;

/// The low water analog reading limit
int lowWaterThreshold = 70;

/// The upper limit on the temperature, in degrees celcius
float tempHighThreshold = 21.0;

/// The lower limit on the temperature, in degrees celcius
float tempLowThreshold = 19.0;

/// The lcd display for DHT sensor readings
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

/// The DHT Temperature and Humidity sensor
DHT dht(DHT_PIN, DHT11);

/// Real time clock module
RTC_DS1307 rtc;

//Current Time
DateTime now;
Servo servo;

/// Holds latest water level
int waterLevel = 0;

/// Holds latest temperature reading. Is 0.0 before readings are taken.
float temperature = 0.0;

/// Holds latest humidity reading. Is 0.0 before readings are taken.
float humidity = 0.0;

/// Is true when the button has been pressed. Must be manually reset with `buttonPressed = false`.
volatile bool buttonPressed = false;

/// The time of the last lcd update
unsigned long lastLCDUpdate = 0;

/// The time of the last servo update
unsigned long lastServoUpdate = 0;

/// An enum of every possible state of a `SwampCooler`
enum State
{
  Disabled = 0,
  Idle = 1,
  Running = 2,
  Error = 3,
};

class StateInterface
{
protected:
  SwampCooler *sc;

public:
  virtual void disable_enable() = 0;
  virtual void checkWater() = 0;
  virtual void checkTemp() = 0;
  virtual void updateLCD() = 0;
};

class DisabledState : public StateInterface
{
public:
  DisabledState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
  void updateLCD();
};

class IdleState : public StateInterface
{
public:
  IdleState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
  void updateLCD();
};

class RunningState : public StateInterface
{
public:
  RunningState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
  void updateLCD();
};

class ErrorState : public StateInterface
{
public:
  ErrorState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
  void updateLCD();
};

class SwampCooler
{
  StateInterface *currentstate;
  DisabledState disabled;
  IdleState idle;
  RunningState running;
  ErrorState error;

public:
  State state;

  SwampCooler();
  void update();
  void setDisabled();
  void setIdle();
  void setRunning();
  void setError();
};

//********************DisabledState Methods********************
DisabledState::DisabledState(SwampCooler *s)
{
  sc = s;
}

void DisabledState::disable_enable()
{
  sc->setIdle();
}

void DisabledState::checkWater()
{
}

void DisabledState::checkTemp()
{
}

/// Updates the LCD for the disabled state
void DisabledState::updateLCD()
{
  // Do nothing. LCD was populated during state transition.
}

//********************IdleState Methods********************
IdleState::IdleState(SwampCooler *s)
{
  sc = s;
}

void IdleState::disable_enable()
{
  sc->setDisabled();
}

void IdleState::checkWater()
{
  if (getWaterLevel() < lowWaterThreshold)
    sc->setError();
}

void IdleState::checkTemp()
{
  // Update humidity stat
  getHumidity();

  if (getTemperature() > tempHighThreshold)
    sc->setRunning();
}

/// Update LCD for the idle state
void IdleState::updateLCD()
{
  updateLCDStats();
}

//********************RunningState Methods********************
RunningState::RunningState(SwampCooler *s)
{
  sc = s;
}

void RunningState::disable_enable()
{
  sc->setDisabled();
}

void RunningState::checkWater()
{
  if (getWaterLevel() < lowWaterThreshold)
    sc->setError();
}

void RunningState::checkTemp()
{
  if (getTemperature() < tempLowThreshold)
    sc->setIdle();
}

/// Update LCD for the Running state
void RunningState::updateLCD()
{
  // Update humidity stat
  getHumidity();
  updateLCDStats();
}

//********************ErrorState Methods********************
ErrorState::ErrorState(SwampCooler *s)
{
  sc = s;
}

void ErrorState::disable_enable()
{
  sc->setDisabled();
}

void ErrorState::checkWater()
{
  if (getWaterLevel() > lowWaterThreshold)
    sc->setIdle();
}

void ErrorState::checkTemp()
{
}

/// Update the lcd for the Error state
void ErrorState::updateLCD()
{
  // Do nothing. LCD was written to during state transition.
}

//********************SwampCooler Methods********************

SwampCooler::SwampCooler() : disabled{this}, idle{this}, running{this}, error{this}
{
  setIdle();
}

void SwampCooler::update()
{
  if (buttonPressed)
  {
    currentstate->disable_enable();
    buttonPressed = false;
  }

  currentstate->checkWater();
  currentstate->checkTemp();

  unsigned long time = millis();
  if (time - lastLCDUpdate > LCD_UPDATE_INTERVAL)
  {
    currentstate->updateLCD();
    lastLCDUpdate = time;
  }

  if (time - lastServoUpdate > SERVO_UPDATE_INTERVAL)
  {
    long reading = analogRead(A1);
    reading = (reading*180) / 527;
    //Serial.print("pot: ");
    //Serial.println(reading);
    // TODO: Get pot reading
    // TODO: Normalize pot reading on [0, 180]

    servo.write(reading);
  }
}

/// Transition to the disabled state
void SwampCooler::setDisabled()
{
  setDisabledOutputs();
  state = State::Disabled;
  currentstate = &disabled;
  currentstate->updateLCD();
}

/// Transition to the idle state
void SwampCooler::setIdle()
{
  state = State::Idle;
  setIdleOutputs();
  currentstate = &idle;
}

/// Transition to the running state
void SwampCooler::setRunning()
{
  setRunningOutputs();
  state = State::Running;
  currentstate = &running;
}

/// Transition to the error state
void SwampCooler::setError()
{
  setErrorOutputs();
  state = State::Error;
  currentstate = &error;
}

//***************OUTPUT FUNCTIONS****************
void disableAll()
{
  *porta &= 0b01010101;
  //digitalWrite(RED_LED_PIN, LOW);
  //digitalWrite(GREEN_LED_PIN, LOW);
  //digitalWrite(BLUE_LED_PIN, LOW);
  //digitalWrite(YELLOW_LED_PIN, LOW);
  //digitalWrite(MOTOR_PIN, LOW);
  //Serial.println("Motor off");
}

void setDisabledOutputs()
{
  disableAll();
  *porta |= (1<<PORTA_YELLOW);
  //digitalWrite(YELLOW_LED_PIN, HIGH);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Disabled");
}

void setIdleOutputs()
{
  disableAll();
  *porta |= (1<<PORTA_GREEN);
  //digitalWrite(GREEN_LED_PIN, HIGH);
  Serial.print("Changed state to idle on: ");
  printRTCTime();
}

void setRunningOutputs()
{
  disableAll();
  Serial.print("Changed state to running on: ");
  printRTCTime();
  *porta |= (1<<PORTA_BLUE);
  //digitalWrite(BLUE_LED_PIN, HIGH);
  digitalWrite(MOTOR_PIN, HIGH);
}

void setErrorOutputs()
{
  disableAll();
  *porta |= (1<<PORTA_RED);
  //digitalWrite(RED_LED_PIN, HIGH);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Water Low");
}

/// Get the water level and cache it the result in `waterlevel`
int getWaterLevel()
{
  int reading = adc_read(0);

  int diff = waterLevel > reading ? waterLevel - reading : reading - waterLevel;
  if (diff > 5)
    waterLevel = reading;

  return waterLevel;
}

/// Get the temperature and cache the result in `temperature`
int getTemperature()
{
  temperature = dht.readTemperature();
  return temperature;
}

/// Get the temperature and cache the result in `humidity`
int getHumidity()
{
  humidity = dht.readHumidity();
  return humidity;
}

/// Update the lcd with humidity and temperature stats
void updateLCDStats()
{
  lcd.clear();
  lcd.setCursor(0, 0);

  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("\xDF"
            "C");

  lcd.setCursor(0, 1);

  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");
}

///Print time using RTC
void printRTCTime()
{
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" at ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}

void adc_init() //Analog read initialization
{
  *adcsra |= 0b10000000; //enable bit 7
  *adcsra &= 0b11011111; //disable adc trigger
  *adcsra &= 0b11110111; //disable adc interrupt
  *adcsra &= 0b11111000; //set prescaler selection to slow reading

  *adcsrb &= 0b11110111; //reset channel
  *adcsrb &= 0b11111000; //set free running mode
  
  *admux &= 0b01111111; // clear bit 7 for AVCC analog reference
  *admux |= 0b01000000; // set bit 6 for AVCC analog reference
  *admux &= 0b11011111; //clear bit 5 for right adjust result
  *admux &= 0b11100000; //clear bit 4-0 to reset channel and gain
  analogInitialized = true;
}

unsigned int adc_read(unsigned char adc_channel_num)  //Analog Read from pins A0 to A7
{
  if(analogInitialized) //only gets value if adc_init() is called
  {
    *admux &= 0b11100000; //clear channel selection bits (MUX 4:0)
    *adcsrb &= 0b11110111;  //clear channel seleciton bits (MUX 5)
    
    if(adc_channel_num > 7) //set channel number
    {
      adc_channel_num -= 8; //set channel selection bits but ignore most significant bit
      *adcsrb |= 0b00001000;  //set MUX 5 bit
    }
    *admux += adc_channel_num;  //set channel selection bits
  
    *adcsra |= 0b01000000;  // set bit 6 of ADCSRA to 1 to start conversion
    while((*adcsra & 0b00000100) != 0); //wait for conversion to complete
    return *adcdata;  //return data register contents
  }
  return 0;
}


//************MAIN***************

SwampCooler swampcooler;

/// The time in millis since arduino startup of the last button press. Used for debouncing.
volatile unsigned long lastButtonPressTime = 0;
/// The time between button presses.
volatile unsigned long buttonPressDebounceThreshold = 200;

/// ISR handler for button presses
void processButtonPressISR()
{
  unsigned long currentButtonPressTime = millis();

  if (currentButtonPressTime - lastButtonPressTime > buttonPressDebounceThreshold)
    buttonPressed = true;

  lastButtonPressTime = currentButtonPressTime;
}

void setup()
{
  
  Serial.begin(9600);

  //initialize analog read
  adc_init();

  //initialize RTC module
  Wire.begin();
  rtc.begin();
  now = rtc.now();
  
  pinMode(MOTOR_PIN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  
  //Set LED pinmode
  *ddra |= 0b10101010;

  //Attach button ISR
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), processButtonPressISR, FALLING);

  //Initialize lcd
  lcd.begin(16, 2);

  //Initialize Temp/Humidity sensor
  dht.begin();

  //Initialize servo
  servo.attach(SERVO_PIN);
}

void loop()
{
  swampcooler.update();
  now = rtc.now();    //load current time from rtc into "now"
}
