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
float tempLowThreshold = 20.0;

/// The lcd display for DHT sensor readings
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

/// The DHT Temperature and Humidity sensor
DHT dht(DHT_PIN, DHT11);

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
    // TODO: Get pot reading
    // TODO: Normalize pot reading on [0, 180]

    servo.write(90);
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
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);
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
  digitalWrite(MOTOR_PIN, HIGH);
}

void setErrorOutputs()
{
  disableAll();
  digitalWrite(RED_LED_PIN, HIGH);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Water Low");
}

/// Get the water level and cache it the result in `waterlevel`
int getWaterLevel()
{
  int reading = analogRead(WATER_SENSOR_PIN);

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
  pinMode(MOTOR_PIN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), processButtonPressISR, FALLING);

  Serial.begin(9600);
  lcd.begin(16, 2);
  dht.begin();
  servo.attach(SERVO_PIN);
}

void loop()
{
  swampcooler.update();
}
