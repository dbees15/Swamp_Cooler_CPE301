#include <LiquidCrystal.h>
#include <DHT.h>

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
void printLCDStats();

/// The pin for the disable button
const unsigned char BUTTON_PIN = 20;

/// The pin for the dht sensor
const unsigned char DHT_PIN = 21;

/// The yellow led pin
const unsigned char YELLOW_LED_PIN = 23;
/// The green led pin
const unsigned char GREEN_LED_PIN = 25;
/// The blue led pin
const unsigned char BLUE_LED_PIN = 27;
/// The red led pin
const unsigned char RED_LED_PIN = 29;

int waterthreshold = 3;

/// The upper limit on the temperature, in degrees celcius
float tempHighThreshold = 38.0;

/// The lower limit on the temperature, in degrees celcius
float tempLowThreshold = 140;

/// The lcd display for DHT sensor readings
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

/// The DHT Temperature and Humidity sensor
DHT dht(DHT_PIN, DHT11);

/// Holds latest water level
int waterlevel = 0;

/// Holds latest temperature reading. Is 0.0 before readings are taken.
float temperature = 0.0;

/// Holds latest humidity reading. Is 0.0 before readings are taken.
float humidity = 0.0;

/// Is true when the button has been pressed. Must be manually reset with `buttonPressed = false`.
volatile bool buttonPressed = false;

/// The time of the last lcd update
unsigned long lastLCDUpdate = 0;

/// The time between lcd updates. The DHT sensor only updates around 1Hz. The lcd also cannot display constantly.
const unsigned long LCD_UPDATE_INTERVAL = 1000;

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
  virtual void disable_enable();
  virtual void checkWater();
  virtual void checkTemp();
};

class DisabledState : public StateInterface
{
public:
  DisabledState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
};

class IdleState : public StateInterface
{
public:
  IdleState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
};

class RunningState : public StateInterface
{
public:
  RunningState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
};

class ErrorState : public StateInterface
{
public:
  ErrorState(SwampCooler *s);
  void disable_enable();
  void checkWater();
  void checkTemp();
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
  if (getWaterLevel() < waterthreshold)
    sc->setError();
}

void IdleState::checkTemp()
{
  // Update humidity stat
  getHumidity();

  if (getTemperature() > tempHighThreshold)
    sc->setRunning();
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
  if (getWaterLevel() < waterthreshold)
    sc->setError();
}

void RunningState::checkTemp()
{
  // Update humidity stat
  getHumidity();

  if (getTemperature() < tempLowThreshold)
    sc->setIdle();
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
  if (getWaterLevel() > waterthreshold)
    sc->setIdle();
}

void ErrorState::checkTemp()
{
  // Error State should update lcd stats
  getTemperature();
  getHumidity();
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

  if (state != State::Disabled)
    printLCDStats();
}

/// Transition to the disabled state
void SwampCooler::setDisabled()
{
  setDisabledOutputs();
  state = State::Disabled;
  currentstate = &disabled;
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
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  Serial.println("Motor off");
  //Serial.println("Servo to closed state");
}

void setDisabledOutputs()
{
  disableAll();
  digitalWrite(YELLOW_LED_PIN, HIGH);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Disabled");
}

void setIdleOutputs()
{
  disableAll();
  digitalWrite(GREEN_LED_PIN, HIGH);
  Serial.print("Changed state to idle at: ");
  Serial.println(millis() / 1000.0f);
}

void setRunningOutputs()
{
  disableAll();
  Serial.print("Changed state to running at: ");
  Serial.println(millis() / 1000.0f);
  digitalWrite(BLUE_LED_PIN, HIGH);
}

void setErrorOutputs()
{
  disableAll();
  digitalWrite(RED_LED_PIN, HIGH);
}

/// Get the water level and cache it the result in `waterlevel`
int getWaterLevel()
{
  waterlevel = analogRead(A0);
  return waterlevel;
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

/// Show stats on the LCD.
void printLCDStats()
{
  unsigned long time = millis();
  if (time - lastLCDUpdate > LCD_UPDATE_INTERVAL)
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

    lastLCDUpdate = time;
  }
}

//************MAIN***************

SwampCooler swampcooler;
//int state = 0;
//states:
//0- disabled
//1-idle
//2-running
//3-error

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
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), processButtonPressISR, FALLING);

  Serial.begin(9600);
  lcd.begin(16, 2);
  dht.begin();
}

void loop()
{
  swampcooler.update();
}
