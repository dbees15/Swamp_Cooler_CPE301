#include <LiquidCrystal.h>

class SwampCooler;
class StateInterface;
class DisabledState;
class IdleState;
class RunningState;
class ErrorState;

int getWaterLevel();
int getTemperature();

void disableAll();
void setDisabledOutputs();
void setIdleOutputs();
void setRunningOutputs();
void setErrorOutputs();

const unsigned char BUTTON_PIN = 20;

int waterthreshold = 3;
int temphighthreshold = 143;
int templowthreshold = 140;
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

int waterlevel;  //holds latest water level
int temperature; //holds latest temperature

volatile bool buttonPressed = false;

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
  int state;

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
  sc->state = 1;
  setIdleOutputs();
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
  setDisabledOutputs();
  sc->state = 0;
  sc->setDisabled();
}
void IdleState::checkWater()
{
  if (getWaterLevel() < waterthreshold)
  {
    setErrorOutputs();
    sc->state = 3;
    sc->setError();
  }
}
void IdleState::checkTemp()
{
  if (getTemperature() > temphighthreshold)
  {
    setRunningOutputs();
    sc->state = 2;
    sc->setRunning();
  }
}

//********************RunningState Methods********************
RunningState::RunningState(SwampCooler *s)
{
  sc = s;
}

void RunningState::disable_enable()
{
  setDisabledOutputs();
  sc->state = 0;
  sc->setDisabled();
}
void RunningState::checkWater()
{
  if (getWaterLevel() < waterthreshold)
  {
    setErrorOutputs();
    sc->state = 3;
    sc->setError();
  }
}
void RunningState::checkTemp()
{
  if (getTemperature() < templowthreshold)
  {
    setIdleOutputs();
    sc->state = 1;
    sc->setIdle();
  }
}

//********************ErrorState Methods********************
ErrorState::ErrorState(SwampCooler *s)
{
  sc = s;
}

void ErrorState::disable_enable()
{
  setDisabledOutputs();
  sc->state = 0;
  sc->setDisabled();
}
void ErrorState::checkWater()
{
  if (getWaterLevel() > waterthreshold)
  {
    sc->state = 1;
    setIdleOutputs();
    sc->setIdle();
  }
}
void ErrorState::checkTemp()
{
}

//********************SwampCooler Methods********************

SwampCooler::SwampCooler() : disabled{this}, idle{this}, running{this}, error{this}
{
  state = 1;
  currentstate = &idle;
}

void SwampCooler::update()
{
  if (buttonPressed)
  {
    currentstate->disable_enable();
    buttonPressed = false;
  }

  //Serial.println(state);
  currentstate->checkWater();
  currentstate->checkTemp();
}

void SwampCooler::setDisabled()
{
  currentstate = &disabled;
}
void SwampCooler::setIdle()
{
  currentstate = &idle;
}
void SwampCooler::setRunning()
{
  currentstate = &running;
}
void SwampCooler::setError()
{
  currentstate = &error;
}

//***************OUTPUT FUNCTIONS****************
void disableAll()
{
  Serial.println("Red off");
  Serial.println("Green off");
  Serial.println("Blue off");
  Serial.println("Yellow off");
  Serial.println("Motor off");
  //Serial.println("Servo to closed state");
}

void setDisabledOutputs()
{
  disableAll();
  Serial.println("Yellow on");
}

void setIdleOutputs()
{
  disableAll();
  Serial.println("Green on");
  Serial.print("Changed state to idle at: ");
  Serial.println(millis() / 1000.0f);
}

void setRunningOutputs()
{
  disableAll();
  Serial.print("Changed state to running at: ");
  Serial.println(millis() / 1000.0f);
  Serial.println("Blue on");
}

void setErrorOutputs()
{
  disableAll();
  Serial.println("Red on");
}

int getWaterLevel()
{
  waterlevel = analogRead(A0);
  return waterlevel;
}

int getTemperature()
{
  temperature = analogRead(A1);
  return temperature;
}

//************MAIN***************

SwampCooler swampcooler;
//int state = 0;
//states:
//0- disabled
//1-idle
//2-running
//3-error

void processButtonPressISR()
{
  buttonPressed = true;
}

void setup()
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), processButtonPressISR, FALLING);

  Serial.begin(9600);
  lcd.begin(16, 2);
}

void loop()
{
  delay(200);

  // lcd.print(getWaterLevel());
  swampcooler.update();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Water Level: ");
  lcd.print(waterlevel);
  lcd.setCursor(0, 1);
  lcd.print("Temperature: ");
  lcd.print(temperature);
}
