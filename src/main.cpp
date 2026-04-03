
//#include <QueueArray.h>
#include <math.h>
#include "GoodStepper.h"
#include "CommandManager.h"
#include "AD5593R.h"

#include <Arduino.h>
/// COMPACT OOP CIRULAR

struct PointF {
  float x;
  float y;
};

struct PointLI {
  long int x;
  long int y;
};


struct PointLI3 {
  long int x;
  long int y;
  long int z;
};


struct PointD {
  double x;
  double y;
};

struct CommandResult {
  char firstLetter;
  float firstNumber;
  String remainingString;
  bool isValid;
};

String S = "";

struct movement {
  int mode;
  long int posn[3] = { 0, 0, 0 };
  long int cent[3] = { 0, 0, 0 };
};

struct dose {
  int pump = 1;
  float pwr = 0;
};

volatile long int lastStep;
long int lastMessage = 0;

///////////// STEPPER AND DRIVER RELATED PARAMETERS - to be changed ONLY after switching levers on corresponding stepper drivers
// Microstep setting for x and y
const int mst = 2;
// Steps for complete rotation
const int spr = 200;

//Microstep setting for z

const int mstZ = 4;

//mm to steps conversion coefficient for X and Y
long int mtos = long(spr) * mst / 8;

//mm to steps conversion coefficient for Z
long int mtosZ = long(spr) * mstZ / 8;

//%
float purgePWR = 80;
////////////////////PINS

//AXIS Y
const int dirYPin = 2;
const int pulYPin = 3;

//AXIS X
const int dirXPin = 4;
const int pulXPin = 5;

//AXIS Z
const int dirZPin = 6;
const int pulZPin = 7;

//PINS
const int ESPin9 = 9;
const int ESPin10 = 10;
const int ESPin11 = 11;
const int ESPin12 = 12;

//BUTTONS
const int HomePin = 19;
const int PurgePin = 18;

//LED
const int LED1 = 16;

//SERVO CONTROL PINS
// Switch on motor (low off, high on)
const int SRVONPin = 26;

// Check state
const int SRDY = 24;


//SECURITY PINS FOR BUTTONS
const int sPin = 17;

//RECHARGE CONTROL PANEL PINS
//recharge button pin. Normally closed
const int cPin;
const int ledFullPin;
const int ledEmptyPin;

/////////////////////////////STATE
//actual position in steps - main position. Position in mm is calculated as function

long int actstep[3] = { 0, 0, 0 };

//target position

long int dpos[3] = { 0, 0, 0 };

long int actcent[3] = { 0, 0, 0 };

//arc-related parameters

long int rad;

//arc angle of opening
float ao;

//////////////////////////////
int value;

//1 - mm, 0 - in
bool units = 1;

bool incr = true;

//DEFAULT SPEED SETTINGS //////////////////////////////
//Default feedrate in steps/s
int feedrate = 1000;

//jogging speed in steps/s
int jspeed = 1900;

//rapid deplacement command, also jogging
int G0Speed = 1000;
int G0SpeedZ = G0Speed * 4;

//jogging timeout
unsigned long jto = round(1000000 / jspeed);

unsigned long jtoZ = round(jto * 0.6);

//homingspeed in steps/s

const int homeSp = 1000;

////////////////////////////////////////////////////////
//debouncing after homing
const long int db = 150;

//step - speed, mm
int sps = 1;
//step - EcoPen, %
float eps = 1;

float stepEco = 3;
float stepPump = 3;
float stepMix = 3;

//pressure

//Coefficient to transform V into bar
// 5V max instead of 10V, 1/2 interval so 0-20bar =

// Conversion coefficient for sensor data in un/V

// Pa/V
float pressCoef = 4.0;
float pressure;

// N*m/V
float torqCoef = 1.0;
float torque;

///SUBSYSTEMS
AD5593R AD5593RR(23);

VStepper stepperX(dirXPin, pulXPin);
VStepper stepperY(dirYPin, pulYPin);
VStepper stepperZ(dirZPin, pulZPin);

CommandManager cm;

Dosage EcoPen(0, 0, stepEco);
Dosage PumpPro(1, 0, stepPump);
Dosage Mixer(2, 0, stepMix);

volatile bool PurgePressed = false;  // Volatile variable for button state
volatile bool HomePressed = false;

volatile bool waitON = true;

unsigned long lastPurgeInterruptTime = 0;  // Shared timestamp for debounce

movement Movement;
dose Dose;

//Indicates if a move is needed
bool flagM = 0;

//Indicates if the move is rotation
bool flagR = 0;

//Indicates if a dosage is needed
bool flagD = 0;

//Indicates if the relative centre coordinates are transmitted
bool flagCent = 0;

//Indicates if the dosage pwr is transmitted
bool flagPwr = 0;

//Indicates if the table is zeroed
bool flagZero = 0;

//Indicates if the radius is sent
bool flagRad = 0;

//Indicates if the home button is pressed
bool flagH = 0;

//Indicates if an update of ADAC values is needed
bool flagI = 0;

bool flagDwell = 0;

//Indicates what coordinates were altered
bool flagX = 0;
bool flagY = 0;
bool flagZ = 0;

//Indicates if purge is in process
bool flagP = 0;
bool activP = 0;

// Indicates if a motor is active
bool flagMix = 0;

//flag indicates if any move command was on the list. If there was no move, there is no printing, so the exchange is simplified.
bool moveList = false;
//Dosage state

//Dwell time
int timeDwell = -1;

String state;

/////////////////////////////////////////V2 upgrade - now the speed is registered for axes X and Y. It is marked for the last iteration

//Current target speed
int targetV;

bool serialInitialized = false;
bool firstSerialRead = true;

void inputCommand();

//limit velocity - it is a highest safe speed to start from

int limspeed = 1500;

//Accelerating mode will make the interval smaller during this number of steps
//Default value, will be recalculated
int accsteps = 1200;
///////////////////////////////////////////

#define QUEUE_SIZE 100  // Размер очереди команд

//x move

String commandQueue[QUEUE_SIZE];  // Массив для хранения команд
int queueHead = 0;                // Указатель на начало очереди
int queueTail = 0;                // Указатель на конец очереди
int queueCount = 0;               // Текущее количество команд в очереди

//Coordinate limits, steps


long int lim[3] = { 375 * mtos, 375 * mtos, 975 * mtosZ };

/// Special serial commands

// if a run is active, following commands can be used to modify the dosing parameters and speed

// '(' - increases the EcoPen power
// ')' - reduces the EcoPen power
// '+' - increases the speed
// '-' - reduces the speed

float oldValue;

//////////////////////////////////////////////////////////////////////////////

volatile long int timeout;

//current coord
volatile long int posX;
volatile long int posY;
volatile long int posZ;

//goalcoord
volatile long int dposX;
volatile long int dposY;
volatile long int dposZ;

//allowed displacement (for calibration)
volatile bool pX;
volatile bool pY;
volatile bool pZ;

//checks if timer is running

volatile bool timerRun;

///ISR LOGIC
volatile bool tickWas = 0;  // becomes 1 when a timer just triggered. Reset to 0 when a calculation is done.
volatile bool mX;
volatile bool mY;
volatile bool mZ;
///

// communication - state string sending

// generated string

String stateString;

// index on a char position
int ssIndex = 0;

volatile bool allowSend = true;

bool getNewState = true;

void stopTimer();
void setupTimer(unsigned long intervalMicroseconds);

class Octant {
public:
  using DFunction = double (*)(long int&, long int&, double&);      // Decision making functions
  using DR2Function = double (*)(long int&, long int&, long int&);  // Decision making functions
  using BFunction = bool (*)(long int&, long int&);                 // Boundary checking functions


  Octant(bool crcw, DR2Function DF0, DFunction DF1, DFunction DF2, BFunction b, bool xc, bool yc, bool xd, int nxO, bool msing, bool toX) {
    _DFuncInitial = DF0;
    _DFuncSingle = DF1;
    _DFuncDouble = DF2;
    _boundary = b;
    _nextOctant = nxO;
    _xcorr = xc;
    _ycorr = yc;
    _xd = xd;
    _crw = crcw;
    _minusSing = msing;
    _toX = toX;
  }

  unsigned long circleTO(long int xy, long int r, int s) {

    //Serial.print("xy is ");
    //Serial.println(xy);

    float d = float(xy) / float(r);

    double Vxy = s * d;

    //Serial.print("X/R is ");
    //Serial.println(d);

    //Serial.print("Vxy is ");
    //Serial.println(Vxy);


    unsigned long to = round(1000000 / abs(Vxy));

    //Serial.print("Calculated timeout is ");
    //Serial.print(to);
    //Serial.println(" microseconds");

    return to;
  }

  void updateOCR(long int intrvl) {
    unsigned long cmv = (16000000 / (1024 * (1000000 / intrvl))) - 1;
    OCR3A = cmv;
  }

  int getNextOct() {
    return _nextOctant;
  }

  // This function formulates corrections and start the timer.
  PointLI3 passOctant(long int& xstart, long int& ystart, long int& zstart, long int& xend, long int& yend, int& frate, int& r, long int& r2) {

    //set directions of motors
    int sgnx;
    int sgny;

    // X,Y,Z are coordinates in relation to the center xc, yc, zc

    long int x = xstart;
    long int y = ystart;
    long int z = zstart;

    PointLI3 crd;

    bool dbl;  // indicates if a step is double or single

    mX = false;
    mY = false;
    mZ = false;

    if (_xcorr) {
      sgnx = 1;
      stepperX.setDirection(1);
    } else {
      sgnx = -1;
      stepperX.setDirection(0);
    }
    //y is inverted
    if (_ycorr) {
      sgny = 1;
      stepperY.setDirection(0);
    } else {
      sgny = -1;
      stepperY.setDirection(1);
    }

    //Get initial F-value
    double F = _DFuncInitial(xstart, ystart, r2);

    // Gets initial move scenario, and updates coordinates
    if (_minusSing) {
      if (F < 0) {
        if (_xd) {
          mX = true;
          mY = false;
          x += sgnx;
        } else {
          mX = false;
          mY = true;
          y += sgny;
        }
        dbl = false;
      } else {
        mX = true;
        mY = true;
        dbl = true;
        x += sgnx;
        y += sgny;
      }
    } else {
      if (F > 0) {
        if (_xd) {
          mX = true;
          mY = false;
          x += sgnx;
        } else {
          mX = false;
          mY = true;
          y += sgny;
        }
        dbl = false;
      } else {
        mX = true;
        mY = true;
        dbl = true;
        x += sgnx;
        y += sgny;
      }
    }
    // setting the ticker to ready
    tickWas = false;

    //starting timer

    if (_toX) {
      setupTimer(circleTO(x, r, frate));
    } else {
      setupTimer(circleTO(y, r, frate));
    }

    while (_boundary(x, y) && !(abs(x - xend) <= 1 && abs(y - yend) <= 1) && !flagH) {
      // This loops runs all the time until we arrive at goal
      // once the ticker signals (so the step was done)

      inputCommand();
      //Serial.print("Boundary : ");
      //Serial.println(_boundary(x,y));
      if (tickWas) {

        mX = 0;
        mY = 0;
        mZ = 0;

        //Setting a correct new timeout
        updateOCR(circleTO(x, r, frate));
        //Correcting F
        if (dbl) {
          F = _DFuncDouble(x, y, F);
        } else {
          F = _DFuncSingle(x, y, F);
        }

        //Preparing next step
        // Gets initial move scenario, and updates coordinates
        if (_minusSing) {
          if (F < 0) {
            if (_xd) {
              mX = true;
              mY = false;
              x += sgnx;
            } else {
              mX = false;
              mY = true;
              y += sgny;
            }
            dbl = false;
          } else {
            mX = true;
            mY = true;
            dbl = true;
            x += sgnx;
            y += sgny;
          }
        } else {
          if (F > 0) {
            if (_xd) {
              mX = true;
              mY = false;
              x += sgnx;
            } else {
              mX = false;
              mY = true;
              y += sgny;
            }
            dbl = false;
          } else {
            mX = true;
            mY = true;
            dbl = true;
            x += sgnx;
            y += sgny;
          }
        }
        //Serial.print("Coordinate updated :");
        //Serial.print(x);
        //Serial.print(" ");
        //Serial.println(y);
        //new step is ready. So now the timer signal is awaited
        tickWas = false;
      }
    }
    //Switch off timer
    stopTimer();

    //save x and y coordinates
    crd.x = x;
    crd.y = y;
    crd.z = z;

    return crd;  //Define end condition - end of the way, special condition or next quadrant
  }

private:
  VStepper& _stepperX;  // Ссылка на объект двигателя по оси X
  VStepper& _stepperY;  // Ссылка на объект двигателя по оси Y
  VStepper& _stepperZ;

  bool _crw;  // Direction of movement. We create 16 class instances. true - counterclockwise
  DR2Function _DFuncInitial;
  DFunction _DFuncSingle;  // Коррекция при обновлении одной координаты
  DFunction _DFuncDouble;  // Коррекция при обновлении обеих координат

  BFunction _boundary;  // Функция проверки границ по X

  int _nextOctant;  // Индекс следующего октанта

  bool _xcorr;      // Направление по X: true - x+=1, false - x-=1 Мы указываем это для направления против часовой стрелки
  bool _ycorr;      // Направление по Y: true - y+=1, false - y-=1
  bool _xd;         // Говорит, какая координата обновляется всегда. Если true - x, false - y
  bool _minusSing;  // If true, single update is called when F<0. Else, if F>0;
  bool _toX;        // if true, TO(x), false - TO(y)
};


Octant CCWOctant0(
  true,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x - 0.5) + sq(y + 1) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += 2 * y + 3;                                   // Пример: увеличиваем x
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (y - x) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return x > y;
  },
  false,  // Направление по X
  true,   // Направление по Y
  false,  // всегда меняется y
  1,      // Номер следующего октанта
  true,
  true);

Octant CCWOctant1(
  true,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x - 1) + sq(y + 0.5) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += -2 * x + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (y - x) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return x > 0;
  },
  false,  // Направление по X
  true,   // Направление по Y
  true,   // всегда меняется x
  2,      // Номер следующего октанта
  false,
  false  // F<0 for single
);

Octant CCWOctant2(
  true,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x - 1) + sq(y - 0.5) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += -2 * x + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += -2 * (x + y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return abs(y) > abs(x);
  },
  false,  // Направление по X
  false,  // Направление по Y
  true,   // всегда меняется x
  3,      // Номер следующего октанта
  true,
  false);

Octant CCWOctant3(
  true,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x - 0.5) + sq(y - 1) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += -2 * y + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += -2 * (x + y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return y > 0;
  },
  false,  // Направление по X
  false,  // Направление по Y
  false,  // всегда меняется x
  4,      // Номер следующего октанта
  false,
  true);

Octant CCWOctant4(
  true,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x + 0.5) + sq(y - 1) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += -2 * y + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (x - y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return abs(x) > abs(y);
  },
  true,   // Направление по X
  false,  // Направление по Y
  false,  // всегда меняется x
  5,      // Номер следующего октанта
  true,
  true);

Octant CCWOctant5(
  true,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x + 1) + sq(y - 0.5) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += 2 * x + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (x - y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return abs(x) > 0;
  },
  true,   // Направление по X
  false,  // Направление по Y
  true,   // всегда меняется x
  6,      // Номер следующего октанта
  false,
  false);

Octant CCWOctant6(
  true,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x + 1) + sq(y + 0.5) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += 2 * x + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (x + y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return abs(y) > abs(x);
  },
  true,  // Направление по X
  true,  // Направление по Y
  true,  // всегда меняется x
  7,     // Номер следующего октанта
  true,
  false);

Octant CCWOctant7(
  true,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x + 0.5) + sq(y + 1) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += 2 * y + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (x + y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return abs(y) > 0;
  },
  true,   // Направление по X
  true,   // Направление по Y
  false,  // всегда меняется x
  0,      // Номер следующего октанта
  false,
  true);


//clockwise octants

Octant CWOctant0(
  false,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x + 0.5) + sq(y - 1) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += -2 * y + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (x - y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return y > 0;
  },
  true,   // Направление по X
  false,  // Направление по Y
  false,  // всегда меняется x
  7,      // Номер следующего октанта
  false,
  true);

Octant CWOctant1(
  false,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x + 1) + sq(y - 0.5) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += 2 * x + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (x - y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return y > x;
  },
  true,   // Направление по X
  false,  // Направление по Y
  true,   // всегда меняется x
  0,      // Номер следующего октанта
  true,
  false);

Octant CWOctant2(
  false,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x + 1) + sq(y + 0.5) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += 2 * x + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (x + y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return x < 0;
  },
  true,  // Направление по X
  true,  // Направление по Y
  true,  // всегда меняется x
  1,     // Номер следующего октанта
  false,
  false);

Octant CWOctant3(
  false,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x + 0.5) + sq(y + 1) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += 2 * y + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (x + y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return abs(x) > abs(y);
  },
  true,   // Направление по X
  true,   // Направление по Y
  false,  // всегда меняется x
  2,      // Номер следующего октанта
  true,
  true);

Octant CWOctant4(
  false,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x - 0.5) + sq(y + 1) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += 2 * y + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (y - x) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return abs(y) > 0;
  },
  false,  // Направление по X
  true,   // Направление по Y
  false,  // всегда меняется x
  3,      // Номер следующего октанта
  false,
  true);

Octant CWOctant5(
  false,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x - 1) + sq(y + 0.5) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += -2 * x + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += 2 * (y - x) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return abs(x) < abs(y);
  },
  false,  // Направление по X
  true,   // Направление по Y
  true,   // всегда меняется x
  4,      // Номер следующего октанта
  true,
  false);

Octant CWOctant6(
  false,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x - 1) + sq(y - 0.5) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += -2 * x + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += -2 * (x + y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return x > 0;
  },
  false,  // Направление по X
  false,  // Направление по Y
  true,   // всегда меняется x
  5,      // Номер следующего октанта
  false,
  false);

Octant CWOctant7(
  false,
  [](long int& x, long int& y, long int& r2) -> double {  // F-function
    double F;
    F = sq(x - 0.5) + sq(y - 1) - r2;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки одной координаты
    F += -2 * y + 3;
    return F;
  },
  [](long int& x, long int& y, double F) -> double {  // Лямбда для корректировки обеих координат
    F += -2 * (x + y) + 5;
    return F;
  },
  [](long int& x, long int& y) -> bool {  // Лямбда для проверки границы по Y
    return abs(x) > abs(y);
  },
  false,  // Направление по X
  false,  // Направление по Y
  false,  // всегда меняется x
  6,      // Номер следующего октанта
  true,
  true);
///////

// timestamp for state strings
long int tmt;

void setup() {
  Serial.begin(1000000);
  Serial.println(logM("Printer restarted, ready"));
  Serial.print("jTO : ");
  Serial.println(jto);

  pinMode(HomePin, INPUT_PULLUP);
  pinMode(PurgePin, INPUT_PULLUP);  // Set the button pin as an input
  attachInterrupt(digitalPinToInterrupt(HomePin), HomeISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PurgePin), PurgeISR, RISING);
  stepperX.invertDir();
  //stepperY.invertDir();
  pinMode(ESPin11, INPUT);
  pinMode(ESPin12, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(sPin, OUTPUT);
  pinMode(SRVONPin, OUTPUT);
  pinMode(SRDY, INPUT);

  //INITIALLY ALL ACTIONS ARE AVAILABLE
  digitalWrite(sPin, HIGH);
  digitalWrite(LED1, LOW);

  Serial.println("Preparation done");
  initializeDAC();
  //initializing timers
  connectUpdate();

  //delay(2000);
  //motor on by default
  //startMotor();

  // Start and stop timers to ensure their correct state
  setupTimerLin(jtoZ);
  stopTimerLin();

  setupTimerCal(jtoZ);
  stopTimerCal();

  setupTimer(jtoZ);
  stopTimer();

  delay(100);

  // Полностью очищаем буфер от случайных символов
  while (Serial.available()) {
    Serial.read();
  }

  Serial.println(logM("Ready for commands"));

  // set a timestamp for loop state update
  tmt = millis();


  setupTimerInfo(10);
  Serial.println(logM("EEE"));
}

void loop() {

  inputCommand();
  //no steps - always has time to send a command
  reportLoop(10);

  //getPress();
  if (flagP && !activP && Mixer.getValue() == 0) {
    Serial.println("Purge activated");
    oldValue = EcoPen.getValue();

    EcoPen.setValue(purgePWR);
    AD5593RR.write_DAC(EcoPen.getId(), 5 * (EcoPen.getValue()) / 100);
    activP = true;
  } else if (!flagP && activP) {
    Serial.println("Purge disactivated");
    EcoPen.setValue(oldValue);
    AD5593RR.write_DAC(EcoPen.getId(), 5 * (EcoPen.getValue()) / 100);
    activP = false;
  }
}

// Interrupt Service Routine (ISR) for button press
void HomeISR() {
  HomePressed = true;
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 2000) {
    Serial.println("Button pressed - HOME");
    //going up shortest way
    evacuate();
  }
  clearQueue();
  last_interrupt_time = interrupt_time;
}

void PurgeISR() {
  unsigned long interrupt_time = millis();
  if (waitON == true && interrupt_time - lastPurgeInterruptTime > 200) {
    Serial.println("Button pressed - PURGE");
    // Set DAC at purge voltage
    //AD5593R.write_DAC(0, 5 * ( purgePWR/ 100));
    flagP = true;
    lastPurgeInterruptTime = interrupt_time;
    waitON = false;
  } else if (interrupt_time - lastPurgeInterruptTime > 200) {
    Serial.println("Button released - PURGE stopped");
    flagP = false;
    waitON = true;
    //AD5593R.write_DAC(0, 5 * (eco/ 100));
    lastPurgeInterruptTime = interrupt_time;
  }
}

// Command parsing and recognition functions

bool isAlpha(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

bool isDigit(char c) {
  return c >= '0' && c <= '9';
}

CommandResult parse(String input) {
  // Once a command line is generated, gets its letter and value
  CommandResult result;
  result.isValid = false;  // Initialize as invalid

  int len = input.length();
  int i = 0;

  // Find the first letter
  while (i < len && !isAlpha(input[i])) {
    i++;
  }

  if (i < len) {
    result.firstLetter = input[i];
    i++;
  } else {
    return result;  // No letter found, return invalid result
  }

  // Find the first number (integer or float)
  String numberStr = "";
  bool hasDecimalPoint = false;
  bool hasSign = false;

  // Check for a negative sign
  if (i < len && input[i] == '-') {
    numberStr += input[i];
    i++;
    hasSign = true;
  }

  while (i < len && (isDigit(input[i]) || (input[i] == '.' && !hasDecimalPoint))) {
    if (input[i] == '.') {
      hasDecimalPoint = true;
    }
    numberStr += input[i];
    i++;
  }

  // Ensure there's at least one digit after the sign or decimal point
  if (numberStr.length() > (hasSign ? 1 : 0)) {
    result.firstNumber = numberStr.toFloat();
  } else {
    return result;  // No valid number found, return invalid result
  }

  // Remaining string after the first letter and number
  result.remainingString = input.substring(i);
  result.isValid = true;  // All values are valid
  return result;
}


String floatArrayToString(float* floatArray, int arraySize) {
  String result = "";

  for (int i = 0; i < arraySize; ++i) {
    result += String(floatArray[i], 2);  // Convert float to string with 2 decimal places
    if (i < arraySize - 1) {
      result += ", ";  // Add a delimiter if needed, here a comma and a space
    }
  }

  return result;
}

// Command list functions

void queueBuild() {
  //Builds a list of commands. *** to start, *** to end
  //*** serves as a begin-end identificator to build list
  delay(10);
  while (true) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command == "***") {
        Serial.println("List end triggered");
        break;
      }
      if (command.length() > 0 && queueCount < QUEUE_SIZE) {
        commandQueue[queueTail] = removeSpaces(command);
        queueTail = (queueTail + 1) % QUEUE_SIZE;
        queueCount++;
        //Serial.println("Command added to list : " + removeSpaces(command));
      }
    }
  }
  Serial.println("List completed");
  //listens to a command from COM, reads it and adds to instruction list
}

void clearQueue() {
  queueHead = 0;
  queueTail = 0;
  queueCount = 0;
  Serial.println("Очередь команд очищена.");
}

//main function
void queueProcess() {
  //iterating command list. Reads each of them.
  Serial.println("Parsing the command list : ");
  for (int i = 0; i < queueCount; i++) {
    Serial.println("------------------------------");
    int index = (queueHead + i) % QUEUE_SIZE;
    //Serial.println(logM(commandQueue[index]));
    commandQueue[index].replace(',', '.');
    //Serial.println(logM(commandQueue[index]));

    CommandResult r;

    //resetting command variables

    r = parse(commandQueue[index]);
    while (r.isValid) {
      Serial.println(r.firstLetter);
      //Serial.println(logM(String(r.firstNumber)));
      identifyCommand(r.firstLetter, r.firstNumber);
      r = parse(r.remainingString);
    }

    //In case of conflict, centre coordinate is taken, R is ignored

    if ((flagM && !flagR && (flagX || flagY || flagZ)) || (flagM && flagR && (flagX || flagY || flagZ) && (flagCent || flagRad))) {
      //Serial.println("The command is a G-command to move");
      //Serial.print("The move is ");
      //Serial.println(Movement.mode);
      Serial.print("Movement.posn is ");
      Serial.print(Movement.posn[0]);
      Serial.print(':');
      Serial.print(Movement.posn[1]);
      Serial.print(':');
      Serial.println(Movement.posn[2]);
      //Serial.print("Relative centre position is ");
      //Serial.println(floatArrayToString(Movement.cent,3));

      //Convert mm to steps and updates target coordinate dpos. If a coordinate is not changed, it is copied from actstep

      if (flagX == 1) {
        dpos[0] = Movement.posn[0];
        Serial.println("X updated!");
      } else {
        dpos[0] = actstep[0];
        Serial.println("X repeated!");
      }

      if (flagY == 1) {
        dpos[1] = Movement.posn[1];
      } else {
        dpos[1] = actstep[1];
      }

      if (flagZ == 1) {
        dpos[2] = Movement.posn[2];
      } else {
        dpos[2] = actstep[2];
      }

      //Centre is calculated in steps using radius, else from relative coordinates
      if (Movement.mode == 2 || Movement.mode == 3) {
        if (flagRad == true) {
          PointLI calcCent = findCircleCenter(actstep[0], actstep[1], dpos[0], dpos[1], rad);
        } else {
          actcent[0] = actstep[0] + Movement.cent[0];
          actcent[1] = actstep[1] + Movement.cent[1];
          actcent[2] = 0;
          rad = calculateRadius0(actstep[0] - actcent[0], actstep[1] - actcent[1]);
          Serial.print("Rad (steps) is ");
          Serial.println(rad);
        }
        if (Movement.mode == 2) {
          ao = opening(false);
        } else {
          ao = opening(true);
        }
      }

      if (flagZero) {
        Serial.println("Move is OK. Executing");
        if (Movement.mode == 0 || Movement.mode == 1) {
          lineWrapper();
          Serial.println("Move completed");
        } else if (Movement.mode == 2 || Movement.mode == 3) {
          circleWrapper();
        }

      } else {
        Serial.println("Printer not zeroed - move ignored");
      }
    } else if (flagD) {
      Serial.println("The command is a M-command to dose");
      Serial.print("Active pump is ");
      Serial.println(Dose.pump);
      Serial.print("Power is ");
      Serial.println(Dose.pwr);
      String signal;
      switch (Dose.pump) {
        case 1:
          EcoPen.setValue(Dose.pwr);
          recalcEco();
          AD5593RR.write_DAC(EcoPen.getId(), 5 * (EcoPen.getValue() / 100));
          Serial.println(logM("Ecopen got updated!"));
          break;
        case 2:
          PumpPro.setValue(Dose.pwr);
          //PUMP RATE CHANGED, RECALCULATE ACC dosage
          recalcEco();
          AD5593RR.write_DAC(EcoPen.getId(), 5 * (EcoPen.getValue() / 100));
          AD5593RR.write_DAC(PumpPro.getId(), 5 * (PumpPro.getValue()) / 100);
          Serial.println(logM("Pump got updated!"));
          if (PumpPro.getValue() > 0) {
            digitalWrite(sPin, LOW);
          } else {
            digitalWrite(sPin, HIGH);
          }
          break;
        case 3:
          Mixer.setValue(Dose.pwr);
          AD5593RR.write_DAC(Mixer.getId(), 5 * (Mixer.getValue()) / 100);
          Serial.println(logM("Mixer updated"));
          if (Mixer.getValue() > 0) {
            digitalWrite(sPin, LOW);
          } else {
            digitalWrite(sPin, HIGH);
          }
          break;
      }
      Serial.println("Dosage updated");
    } else if (flagDwell && timeDwell > 0) {
      delay(timeDwell);
    }

    else {
      if (flagM || flagD) {
        Serial.println("Incomplete command");
      }
    }
    resetCommand();
  }
}

void queueRead() {
  //iterating command list. Reads each of them.
  Serial.println(logM("Reading the command list : "));
  Serial.println(logM("------------------------------"));
  for (int i = 0; i < queueCount; i++) {
    int index = (queueHead + i) % QUEUE_SIZE;
    Serial.println(logM(commandQueue[index]));
  }
  Serial.println(logM("------------------------------"));
  Serial.println(logM("End of command list"));
}
void initializeDAC() {
  // To be placed in startup,
  bool my_DACs[8] = { 1, 1, 1, 0, 0, 0, 0, 0 };
  bool my_ADCs[8] = { 0, 0, 0, 1, 1, 1, 1, 1 };

  AD5593RR.enable_internal_Vref();
  AD5593RR.set_DAC_max_2x_Vref();
  AD5593RR.set_ADC_max_2x_Vref();
  AD5593RR.configure_DACs(my_DACs);
  AD5593RR.write_DAC(0, 0);
  AD5593RR.write_DAC(1, 0);
  AD5593RR.write_DAC(2, 0);
  AD5593RR.configure_ADCs(my_ADCs);
  AD5593RR.read_ADCs();
}

void identifyCommand(char letter, float number) {

  if (letter == 'V' || letter == 'G' || letter == 'F') {
    switch (letter) {
      case 'V':
        Serial.println("V trigger");
        if (flagD) {
          Dose.pwr = number;
          //Serial.println(logM("Value registrated : "+String(number)));
          flagPwr = 1;
        } else {
          Serial.println("Error in assigning - send dosing M-command first");
        }
        break;
      case 'G':
        Serial.println("Processing G command");
        value = round(number);
        switch (value) {
          case 0:
            flagM = true;
            //rapid
            Movement.mode = 0;
            feedrate = homeSp;
            moveList = true;
            break;

          case 1:
            flagM = true;
            //linear
            Movement.mode = 1;
            moveList = true;
            break;
          case 2:
            flagM = true;
            //clockwise
            Movement.mode = 2;
            flagR = true;
            moveList = true;
            break;
          case 3:
            flagM = true;
            //counterclockwise
            Movement.mode = 3;
            flagR = true;
            moveList = true;
            break;
          case 4:
            flagDwell = true;
            break;
          case 20:
            units = 0;
            Serial.println("Units: inches");
            break;
          case 21:
            units = 1;
            Serial.println("Units: millimeters");
            break;
          case 28:
            //home or zeroing
            startZero();
            Serial.println("zzz");
            moveList = true;
            break;
          case 90:
            incr = false;
            break;
          case 91:
            incr = true;
            break;
          default:
            Serial.println("Command not recognized");
            Movement.mode = -1;
        }
        break;
      case 'F':
        Serial.println("F trigger");
        if (!units) {
          number = number * 25.4;
        }
        feedrate = round(number * mtos);

        String cmd;

        cmd = "ff" + String(feedrate / mtos);

        Serial.print(cmd);
        //Serial.println(number);
        //Serial.print("New speed in steps/s : ");
        //Serial.println(feedrate);
        break;
    }
  } else {
    switch (letter) {
      case 'X':
        Serial.println("X triggered");
        if (!units) {
          number = number * 25.4;
        }
        if (flagM) {
          flagX = 1;
          if (!incr) {
            Movement.posn[0] = round(long(number) * mtos);
          } else {
            Movement.posn[0] = actstep[0] + round(long(number) * mtos);
          }
        } else {
          Serial.println("Error in assigning - send movement G-command first");
        }
        break;
      case 'M':
        Serial.println("Processing M command");
        flagD = true;
        value = round(number);
        switch (value) {
          case 201:
            Dose.pump = 1;
            break;
          case 202:
            Dose.pump = 2;
            break;
          case 203:
            Dose.pump = 3;
            break;
          default:
            Serial.println("Command not recognized");
        }
        break;
      case 'Y':
        Serial.println("Y triggered");
        if (!units) {
          number = number * 25.4;
        }
        flagY = 1;
        if (flagM) {
          if (!incr) {
            Movement.posn[1] = round(long(number) * mtos);
          } else {
            Movement.posn[1] = actstep[1] + round(long(number) * mtos);
          }
        } else {
          Serial.println("Error in assigning - send movement G-command first");
        }
        break;
      case 'Z':
        Serial.println("Z triggered");
        if (!units) {
          number = number * 25.4;
        }
        flagZ = 1;
        if (flagM) {
          if (!incr) {
            Movement.posn[2] = round(long(number) * mtosZ);
          } else {
            Movement.posn[2] = actstep[2] + round(long(number) * mtosZ);
          }
        } else {
          Serial.println("Error in assigning - send movement G-command first");
        }
        break;
      case 'I':
        Serial.println("I triggered");
        if (!units) {
          number = number * 25.4;
        }
        if (flagM) {
          Movement.cent[0] = round(long(number) * mtos);
          flagCent = 1;
        } else {
          Serial.println("Error in assigning - send movement G-command first");
        }
        break;
      case 'J':
        Serial.println("J triggered");
        if (!units) {
          number = number * 25.4;
        }
        if (flagM) {
          Movement.cent[1] = round(long(number) * mtos);
          flagCent = 1;
        } else {
          Serial.println("Error in assigning - send movement G-command first");
        }
        break;
      case 'R':
        Serial.println("R triggered");
        //Serial.println("----------------------------------------------------");
        if (!units) {
          number = number * 25.4;
        }
        if (flagM) {
          flagRad = true;
          rad = round(number * mtos);
        } else {
          Serial.println("Error in assigning - send moving G-command first");
        }
        break;
      case 'P':
        Serial.println("P triggered");
        //Serial.println("timeout");
        if (flagDwell) {
          timeDwell = round(number);
        }
        break;
      default:
        Serial.println("Command not recognized");
    }
  }
}

String removeSpaces(String str) {
  String result = "";
  for (int i = 0; i < str.length(); i++) {
    if (str[i] != ' ') {
      result += str[i];
    }
  }
  return result;
}
//////////////////////////////////////////

void resetCommand() {
  // Сброс переменной Movement
  Movement.mode = -1;
  flagM = 0;
  flagD = 0;
  flagR = 0;
  flagCent = 0;
  flagPwr = 0;
  flagRad = 0;
  flagX = 0;
  flagY = 0;
  flagZ = 0;
  flagH = 0;
  flagDwell = 0;
  timeDwell = -1;
}

// gets the angle of opening
float opening(bool ccw) {
  long int posAx = actstep[0] - actcent[0];
  long int posAy = actstep[1] - actcent[1];

  // coordinates of end from centre
  long int posBx = dpos[0] - actcent[0];
  long int posBy = dpos[1] - actcent[1];

  //A - initial, B - final
  float thetaA = atan2(posAy, posAx);
  float thetaB = atan2(posBy, posBx);

  thetaA = normalizeAngle(thetaA);
  thetaB = normalizeAngle(thetaB);

  //Calculate central angle to get the length

  if (ccw) {
    // Если движение против часовой стрелки
    ao = thetaB - thetaA;
  } else {
    // Если движение по часовой стрелке
    ao = thetaA - thetaB;
  }

  if (ao < 0) {
    ao += 2 * PI;
  }

  Serial.print("Angle of opening (rad) is ");
  Serial.println(ao);
}

float normalizeAngle(float angle) {
  while (angle < 0) angle += 2 * PI;
  while (angle >= 2 * PI) angle -= 2 * PI;
  return angle;
}

bool isAngleInArc(float angle, float start, float end, bool counterclockwise) {
  bool clockwise = !counterclockwise;
  if (clockwise) {
    return (angle <= start && angle >= end) || (start < end && (angle <= start || angle >= end));
  } else {
    return (angle >= start && angle <= end) || (start > end && (angle >= start || angle <= end));
  }
}

float calculateRadius0(float x, float y) {
  float R;
  R = sqrt(x * x + y * y);
  Serial.print("Radius is ");
  Serial.println(R);
  return R;
}

// MOVEMENT RELATED COMMANDS

// Makes a transform of mm to steps of stepper motors and starts the move
void lineWrapper() {
  Serial.print("Performing a linear move from ");
  Serial.print(actstep[0]);
  Serial.print(':');
  Serial.print(actstep[1]);
  Serial.print(':');
  Serial.print(actstep[2]);
  Serial.print(" to ");
  Serial.print(dpos[0]);
  Serial.print(':');
  Serial.print(dpos[1]);
  Serial.print(':');
  Serial.println(dpos[2]);


  //start a line movement
  BrsLine();
  Serial.print("Move completed. Actual stepper coordinates are : ");
  Serial.print(actstep[0]);
  Serial.print(":");
  Serial.println(actstep[1]);

  //calculate approximate mm position
}

void BrsLine() {
  unsigned long atime;

  // Если движение только по оси Z
  if (!flagX && !flagY) {
    int iz;
    Serial.println("Move is vertical");
    // Определяем направление движения по оси Z
    if (actstep[2] < dpos[2]) {
      stepperZ.setDirection(0);
      iz = 1;
    } else {
      stepperZ.setDirection(1);
      iz = -1;
    }

    // a Z move
    mX = false;
    mY = false;
    mZ = true;

    //ticker set
    tickWas = false;

    // Starting a timer with constant speed

    setupTimerLin(jtoZ);

    while (actstep[2] != dpos[2]) {

      //Serial.print(actstep[0]);
      //Serial.print(":");
      //Serial.print(actstep[1]);
      //Serial.print(":");
      //Serial.println(actstep[2]);
      //checkInput();

      inputCommand();
      if (tickWas) {
        actstep[2] += iz;
        tickWas = false;
      }
    }
    stopTimerLin();
    Serial.println("Position achieved");

  } else {
    // Комбинированное движение по X, Y и Z
    long int dx = abs(actstep[0] - dpos[0]);
    long int dy = abs(actstep[1] - dpos[1]);
    long int dz = abs(actstep[2] - dpos[2]);

    long int errXY = dx - dy;
    long int errXZ = dx - dz;

    int sx = (actstep[0] < dpos[0]) ? 1 : -1;
    int sy = (actstep[1] < dpos[1]) ? 1 : -1;
    int sz = (actstep[2] < dpos[2]) ? 1 : -1;

    bool xdir = (actstep[0] < dpos[0]) ? true : false;  //stepper directions
    bool ydir = (actstep[1] < dpos[1]) ? false : true;
    bool zdir = (actstep[2] < dpos[2]) ? false : true;

    stepperX.setDirection(xdir);
    stepperY.setDirection(ydir);
    stepperZ.setDirection(zdir);

    //targetV gets assigned
    lineTO(dx, dy, feedrate, atime);

    long int itr = 0;

    // We prepare the first step

    long int e2XY = 2 * errXY;
    long int e2XZ = 2 * errXZ;

    mX = false;
    mY = false;
    mZ = false;

    if (e2XY > -dy && e2XZ > -dz) {
      errXY -= dy;
      errXZ -= dz;
      actstep[0] += sx;
      mX = true;
    }

    if (e2XY < dx) {
      errXY += dx;
      actstep[1] += sy;
      mY = true;
    }

    if (e2XZ < dx) {
      errXZ += dx;
      actstep[2] += sz;
      mZ = true;
    }

    // Starting a timer with constant speed

    //setupTimerLin(timeout);

    //initializing the timer with calculated speed
    //steps counter set to 0
    int accCnt = 0;

    int vdv = initializeAccLin(targetV);

    //setting initial speed value correctly
    int frt;
    if (vdv == 0) {
      frt = targetV;
    } else {
      frt = limspeed;
    }

    while (!(actstep[0] == dpos[0] && actstep[1] == dpos[1] && actstep[2] == dpos[2]) && !flagH) {

      //Serial.print(actstep[0]);
      //Serial.print(":");
      //Serial.print(actstep[1]);
      //Serial.print(":");
      //Serial.println(actstep[2]);

      //checkInput();
      inputCommand();

      if (tickWas) {
        // update speed

        // counting ticks
        accCnt++;
        // updating timeout if needed
        updateAccLin(vdv, frt, targetV, accsteps, accCnt);
        frt += vdv;

        mX = false;
        mY = false;
        mZ = false;

        e2XY = 2 * errXY;
        e2XZ = 2 * errXZ;

        // Движение по оси X
        if (e2XY > -dy && e2XZ > -dz) {
          errXY -= dy;
          errXZ -= dz;
          actstep[0] += sx;
          mX = true;
        }

        // Движение по оси Y
        if (e2XY < dx) {
          errXY += dx;
          actstep[1] += sy;
          mY = true;
        }

        // Движение по оси Z
        if (e2XZ < dx) {
          errXZ += dx;
          actstep[2] += sz;
          mZ = true;
        }
        tickWas = false;
      }
    }
    stopTimerLin();
    Serial.println("Position achieved");
    Serial.print("Position is ");
    Serial.print(actstep[0]);
    Serial.print(" ");
    Serial.println(actstep[1]);
  }
}

// V2 - only calculates speed, later to be initialised properly.
void lineTO(long int dx, long int dy, int s, unsigned long& atime) {
  unsigned long TO;
  float cf;
  float sf;  //time in microseconds
  if (dx != 0) {
    float tg = float(dy) / float(dx);
    float phi = atan(tg);
    //Serial.print("Angle is ");
    //Serial.print(phi);
    //Serial.println(" rad");

    cf = cos(phi);
    sf = sin(phi);
  } else {
    sf = 1;
  }
  float v;
  float mv;
  //unsigned long atime;
  if (dx > dy) {
    v = s * cf;
    //Serial.println("X is updating at a higher rate");
    //Serial.print("|Vx| = ");
    //Serial.print(v);
    //Serial.println(" steps/s");
    mv = v / 1000000;
    atime = round(dx / mv);
  } else {
    v = s * sf;
    //Serial.println("Y is updating at a higher rate");
    //Serial.print("|Vy| = ");
    //Serial.print(v);
    //Serial.println(" steps/s");
    mv = v / 1000000;
    atime = round(dy / mv);
  }
  //setting the target speed
  targetV = v;
  //TO = round(1000000 / v);
  //Serial.print("Movement timeout is ");
  //Serial.println(TO);
  //return TO;
}

void startZero() {

  Serial.println("Initial homing started");
  delay(200);

  unsigned long toz = frTO(G0Speed);
  unsigned long tozZ = frTO(G0SpeedZ);
  Serial.print("Timeout calculated - zeroing : ");
  Serial.println(toz);
  stepperX.setDirection(0);
  stepperY.setDirection(1);
  stepperZ.setDirection(1);
  unsigned long timer = micros();

  bool xt = false;
  bool yt = false;

  //Start with X and Y
  pX = 1;
  pY = 1;
  pZ = 0;

  // Start timer
  setupTimerCal(toz);
  timerRun = 1;

  while (timerRun) {}
  Serial.println(logM("X and Y zeroed !"));

  actstep[0] = 0;
  actstep[1] = 0;
  actstep[2] = 0;

  // After zeroing X,Y the table moves its endstop under the printing head
  // To avoid multiple adjustements, the position is measured in mm from the X and Y endstops

  int mmdbX = 74;
  int mmdbY = 341;

  int dbX = mmdbX * mtos;
  int dbY = mmdbY * mtos;

  Serial.print("dbX is ");
  Serial.println(dbX);

  Serial.print("dbY is ");
  Serial.println(dbY);

  dpos[0] = dbX;
  dpos[1] = dbY;
  dpos[2] = 0;

  Serial.print("Actual position in steps : ");
  Serial.print(actstep[0]);
  Serial.print(":");
  Serial.println(actstep[1]);

  Serial.print("Target position in steps : ");
  Serial.print(dpos[0]);
  Serial.print(":");
  Serial.println(dpos[1]);
  Serial.println(db);
  flagX = 1;
  flagY = 1;
  flagZ = 1;

  BrsLine();

  //Z calibration
  pX = 0;
  pY = 0;
  pZ = 1;

  delay(1000);
  setupTimerCal(tozZ);
  timerRun = 1;

  while (timerRun) {}

  Serial.println(logM("Zeroing done !"));
  actstep[2] = round(1.5 * mtosZ);
  //Delay needed to lower the acceleration
  delay(1000);
  // Vertical debounce

  dpos[0] = actstep[0];
  dpos[1] = actstep[1];
  dpos[2] = 10000;

  flagX = 0;
  flagY = 0;
  flagZ = 1;

  BrsLine();

  Serial.print("Debouncing finished. New coordinates are ");
  Serial.print(actstep[0]);
  Serial.print(":");
  Serial.print(actstep[1]);
  Serial.print(":");
  Serial.println(actstep[2]);

  //actstep[0] = stepperX.position();
  //actstep[1] = stepperY.position();

  flagZero = true;
}

///////// circular
void circleWrapper() {

  bool counterclockwise;

  if (Movement.mode == 2) {
    counterclockwise = false;
  } else {
    counterclockwise = true;
  }

  Serial.print("Performing an arc move from ");
  Serial.print(actstep[0]);
  Serial.print(':');
  Serial.print(actstep[1]);
  Serial.print(':');
  Serial.print(actstep[2]);
  Serial.print(" to ");
  Serial.print(dpos[0]);
  Serial.print(':');
  Serial.print(dpos[1]);
  Serial.print(':');
  Serial.println(dpos[2]);

  if (counterclockwise) {
    Serial.println("Rotation counterclockwise");
  } else {
    Serial.println("Rotation clockwise");
  }

  Serial.print("Centre position is ");
  Serial.print(actcent[0]);
  Serial.print(':');
  Serial.println(actcent[1]);

  Serial.print("Radius is ");
  Serial.print(rad);

  //start a circle movement. All parameters are in steps
  BrsCircle(actcent[0], actcent[1], rad, actstep[0], actstep[1], actstep[2], dpos[0], dpos[1], dpos[2], feedrate, counterclockwise);

  Serial.print("Move completed. Actual stepper coordinates are : ");
  Serial.print(actstep[0]);
  Serial.print(":");
  Serial.println(actstep[1]);
}

void BrsCircle(int xc, int yc, int r, int x_start, int y_start, int z_start, int x_end, int y_end, int z_end, int V, bool counterclockwise) {
  //Z - axis calculation parameters
  //The Z movement can only be to +Z
  bool tcalc = false;
  float dt;
  float zstep;
  float zact = 0;

  stepperZ.setDirection(0);

  //theta to Z coefficient, steps/rad
  float k = (z_end - z_start) / ao;

  Serial.print("Z/T is ");
  Serial.println(k);

  // x,y,z are coordinates around the circle center

  long int x = x_start - xc;
  long int y = y_start - yc;
  long int z = z_start;

  long int x_end_relative = x_end - xc;
  long int y_end_relative = y_end - yc;

  unsigned long t0;

  //Serial.print("r = ");
  //Serial.print(r);
  long r2 = r * long(r);

  //Serial.print("r*r = ");
  //Serial.println(r2);

  //Serial.print("xstart is = ");
  //Serial.println(x);

  double F;
  int octant = getOctant(x, y, counterclockwise);

  Serial.print("Octant : ");
  Serial.println(octant);

  //Serial.print("F = ");
  //Serial.println(F);

  PointLI3 rest;  //result of passing an octant;
  while (!(abs(x - x_end_relative) <= 1 && abs(y - y_end_relative) <= 1) && !flagH) {
    //Serial.println("----------------------------------------------------------------------------");
    //Serial.print("New F is ");
    //Serial.println(F);

    if (counterclockwise) {
      switch (octant) {
        case 0:
          rest = CCWOctant0.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CCWOctant0.getNextOct();
          break;
        case 1:
          rest = CCWOctant1.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CCWOctant1.getNextOct();
          break;
        case 2:
          rest = CCWOctant2.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CCWOctant2.getNextOct();
          break;
        case 3:
          rest = CCWOctant3.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CCWOctant3.getNextOct();
          break;
        case 4:
          rest = CCWOctant4.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CCWOctant4.getNextOct();
          break;
        case 5:
          rest = CCWOctant5.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CCWOctant5.getNextOct();
          break;
        case 6:
          rest = CCWOctant6.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CCWOctant6.getNextOct();
          break;
        case 7:
          rest = CCWOctant7.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CCWOctant7.getNextOct();
          break;
      }
    } else {
      switch (octant) {
        case 0:
          rest = CWOctant0.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CWOctant0.getNextOct();
          break;
        case 1:
          rest = CWOctant1.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CWOctant1.getNextOct();
          break;
        case 2:
          rest = CWOctant2.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CWOctant2.getNextOct();
          break;
        case 3:
          rest = CWOctant3.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CWOctant3.getNextOct();
          break;
        case 4:
          rest = CWOctant4.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CWOctant4.getNextOct();
          break;
        case 5:
          rest = CWOctant5.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CWOctant5.getNextOct();
          break;
        case 6:
          rest = CWOctant6.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CWOctant6.getNextOct();
          break;
        case 7:
          rest = CWOctant7.passOctant(x, y, z, x_end_relative, y_end_relative, V, r, r2);
          octant = CWOctant7.getNextOct();
          break;
      }
    }
    // updating relative coordinates
    x = rest.x;
    y = rest.y;
    z = rest.z;

    // updated stepper global coordinates

    actstep[0] = xc + x;
    actstep[1] = yc + y;
    actstep[2] = z;

    Serial.println("Octant passed");
  }
  //Serial.println("target coordinates");
  // Выводим конечные координаты
  //Serial.print("X: ");
  //Serial.print(xc + x_end_relative);
  //Serial.print(", Y: ");
  //Serial.println(yc + y_end_relative);

  //Serial.println("Stepper coordinates:");
  // Выводим конечные координаты
  //Serial.print("X: ");
  //Serial.print(stepperX.position());
  //Serial.print(", Y: ");
  //Serial.println(stepperY.position());

  //actstep[0] = stepperX.position();
  //actstep[1] = stepperY.position();

  Serial.println("New coordinates after circular move are : ");
  Serial.print(actstep[0]);
  Serial.print(":");
  Serial.print(actstep[1]);
  Serial.print(":");
  Serial.println(actstep[2]);
}


int getOctant(int dx, int dy, bool counterclockwise) {
  //Serial.println("*********************");
  //Serial.print("X,Y is ");
  //Serial.print(dx);
  //Serial.print(";");
  //Serial.println(dy);
  if (counterclockwise) {
    if (dx > 0 && dy >= 0) {
      return (dx > dy) ? 0 : 1;  // Октанты 0 и 1
    } else if (dx <= 0 && dy > 0) {
      return (-dx >= dy) ? 3 : 2;  // Октанты 3 и 2
    } else if (dx < 0 && dy <= 0) {
      return (-dx > -dy) ? 4 : 5;  // Октанты 4 и 5
    } else {
      return (dx >= -dy) ? 7 : 6;  // Октанты 7 и 6
    }
  } else {
    if (dx >= 0 && dy > 0) {
      return (dx >= dy) ? 0 : 1;  // Октанты 0 и 1
    } else if (dx < 0 && dy >= 0) {
      return (-dx > dy) ? 3 : 2;  // Октанты 3 и 2
    } else if (dx <= 0 && dy < 0) {
      return (-dx >= -dy) ? 4 : 5;  // Октанты 4 и 5
    } else {
      return (dx > -dy) ? 7 : 6;  // Октанты 7 и 6
    }
  }
}

PointLI findCircleCenter(long int x1, long int y1, long int x2, long int y2, long int radius) {

  PointLI center = { 0, 0 };

  // Вычисляем середину отрезка AB
  float midX = (x1 + x2) / 2;
  float midY = (y1 + y2) / 2;

  // Вычисляем расстояние между точками
  float dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));

  // Проверяем, что радиус больше половины расстояния между точками
  if (abs(radius) < dist / 2) {
    Serial.println("Ошибка: радиус слишком мал для заданных точек.");
    return;
  }

  // Вычисляем расстояние от середины отрезка до центра окружности
  float h = sqrt(pow(radius, 2) - pow(dist / 2, 2));

  // Вычисляем вектор, перпендикулярный к отрезку AB
  long int perpX = y2 - y1;
  long int perpY = x1 - x2;

  // Нормализуем вектор
  float length = sqrt(pow(perpX, 2) + pow(perpY, 2));
  perpX /= length;
  perpY /= length;

  // Находим возможные центры
  long int centerX1 = round(midX + h * perpX);
  long int centerY1 = round(midY + h * perpY);
  long int centerX2 = round(midX - h * perpX);
  long int centerY2 = round(midY - h * perpY);

  // Выбираем правильный центр в зависимости от направления
  if (radius > 0) {
    Serial.print("Центр окружности слева: (");
    Serial.print(centerX2);
    Serial.print(", ");
    Serial.print(centerY2);
    Serial.println(")");
    center.x = centerX2;
    center.y = centerY2;
  } else {
    Serial.print("Центр окружности справа: (");
    Serial.print(centerX1);
    Serial.print(", ");
    Serial.print(centerY1);
    Serial.println(")");
    center.x = centerX1;
    center.y = centerY1;
  }
  return center;
}

//reports the system state
void info() {
  Serial.println(logM("--------------------------------"));
  Serial.println(logM("3D printer parameters : "));
  if (units) {
    Serial.println(logM("Units: mm"));
  } else {
    Serial.println(logM("Units: in"));
  }
  if (incr) {
    Serial.println(logM("Mode: incremental"));
  } else {
    Serial.println(logM("Mode: absolute"));
  }
  if (flagZero) {
    Serial.println(logM("Zeroing state: zeroed"));
  } else {
    Serial.println(logM("Zeroing state: not zeroed"));
  }

  Serial.println(logM("Feedrate is " + String(float(feedrate) / mtos) + " mm/s"));
  Serial.println(logM("Feedrate is " + String(float(feedrate)) + " steps/s"));

  Serial.println(logM("Coordinates (in steps) are " + String(actstep[0]) + ":" + String(actstep[1]) + ":" + String(actstep[2])));

  Serial.println(logM("Coordinates (in mm) are " + String(float(actstep[0]) / mtos) + ":" + String(float(actstep[1]) / mtos) + ":" + String(float(actstep[2]) / mtosZ)));

  Serial.println(logM("Microsteps per step : " + String(mst)));

  Serial.println(logM("mm to microstep conversion coefficient (axis X and Y): " + String(mtos) + ",    axis Z : " + String(mtosZ)));

  Serial.println(logM("Current limits in steps are " + String(lim[0]) + ":" + String(lim[1]) + ":" + String(lim[2])));

  Serial.println(logM("Dosing :"));

  Serial.println(logM("EcoPen power : " + String(EcoPen.getValue()) + " %"));

  Serial.println(logM("ViscoPro pump power : " + String(PumpPro.getValue()) + " %"));

  Serial.println(logM("Mixing speed : " + String(Mixer.getValue()) + " %"));

  Serial.println(logM("--------------------------------"));
}

//Moving to direction until a stop command '#' arrives
void stepUp() {

  // Updated - timer

  Serial.println("fwd active");

  stepperX.setDirection(0);

  // if zeroed, check if at least 1 step is possible

  if (flagZero) {
    if ((actstep[0] - 1) < 0) {
      Serial.println(logM("AXIS AT LIMIT"));
      return;
    }
  }

  mX = true;
  mY = false;
  mZ = false;

  //steps counter set to 0
  int accCnt = 0;

  // correction value generation, starting timer
  int vdv = initializeAccLin(feedrate);

  //setting initial speed value correctly
  int frt;
  if (vdv == 0) {
    frt = feedrate;
  } else {
    frt = limspeed;
  }
  ///////////////////////////// переделать управление шагами в случае если была калибровка
  while (true) {

    if (Serial.read() == '#') {
      Serial.println(logM("Move ended"));
      stopTimerLin();
      break;
    }
    if (tickWas) {
      //communicate();
      // counting ticks
      accCnt++;
      // updating timeout if needed
      updateAccLin(vdv, frt, feedrate, accsteps, accCnt);
      frt += vdv;

      tickWas = false;

      if (flagZero) {
        if ((actstep[0] - 1) < 0) {
          stopTimerLin();
          Serial.println(logM("AXIS AT LIMIT"));
          break;
        } else {
          actstep[0] -= 1;
        }
      }
    }
  }
}

void stepDown() {

  Serial.println("bwd active");

  stepperX.setDirection(1);

  // if zeroed, check if at least 1 step is possible

  if (flagZero) {
    if ((actstep[0] + 1) > lim[0]) {
      Serial.println(logM("AXIS AT LIMIT"));
      return;
    }
  }

  mX = true;
  mY = false;
  mZ = false;

  //steps counter set to 0
  int accCnt = 0;

  // correction value generation, starting timer
  int vdv = initializeAccLin(feedrate);

  //setting initial speed value correctly
  int frt;
  if (vdv == 0) {
    frt = feedrate;
  } else {
    frt = limspeed;
  }

  ///////////////////////////// переделать управление шагами в случае если была калибровка
  while (true) {

    if (Serial.read() == '#') {
      Serial.println("Move ended");
      stopTimerLin();
      break;
    }
    if (tickWas) {
      //communicate();
      // counting ticks
      accCnt++;
      // updating timeout if needed
      updateAccLin(vdv, frt, feedrate, accsteps, accCnt);
      frt += vdv;

      tickWas = false;

      if (flagZero) {
        if ((actstep[0] + 1) > lim[0]) {
          stopTimerLin();
          Serial.println(logM("AXIS AT LIMIT"));
          break;
        } else {
          actstep[0] += 1;
        }
      }
    }
  }
}

void stepLeft() {

  Serial.println("lft active");

  long int ip;
  unsigned long tst;

  //initial time and position saved
  if (flagZero) {
    ip = actstep[1];
    tst = micros();
  }

  stepperY.setDirection(1);

  // if zeroed, check if at least 1 step is possible

  if (flagZero) {
    if ((actstep[1] - 1) < 0) {
      Serial.println(logM("AXIS AT LIMIT"));
      return;
    }
  }

  mX = false;
  mY = true;
  mZ = false;

  //steps counter set to 0
  int accCnt = 0;

  // correction value generation, starting timer
  int vdv = initializeAccLin(feedrate);
  Serial.println(logM("dV = " + String(vdv) + " steps/s"));
  //setting initial speed value correctly
  int frt;
  if (vdv == 0) {
    frt = feedrate;
  } else {
    frt = limspeed;
  }

  ///////////////////////////// переделать управление шагами в случае если была калибровка
  while (true) {
    if (Serial.read() == '#') {
      Serial.println(logM("Move ended"));
      stopTimerLin();

      //
      if (flagZero) {
        float tme = (micros() - tst);
        float sps = actstep[1] - ip;
        Serial.println(logM("Move duration " + String(tme) + " s, steps made " + String(sps)));
        float result = sps / tme;
        Serial.println(logM("Actual speed steps/s was " + String(result * 1000000)));
        //
      }

      break;
    }
    if (tickWas) {

      //communicate();

      // counting ticks
      accCnt++;
      // updating timeout if needed
      updateAccLin(vdv, frt, feedrate, accsteps, accCnt);
      frt += vdv;

      //updateStateStep();

      tickWas = false;

      if (flagZero) {
        if ((actstep[1] - 1) < 0) {
          stopTimerLin();
          Serial.println(logM("AXIS AT LIMIT"));
          break;
        } else {
          actstep[1] -= 1;
        }
      }
    }
  }
}

void stepRight() {

  Serial.println("rgt active");

  long int ip;
  unsigned long tst;

  //initial time and position saved
  if (flagZero) {
    ip = actstep[1];
    tst = micros();
  }

  stepperY.setDirection(0);

  // if zeroed, check if at least 1 step is possible

  if (flagZero) {
    if ((actstep[1] + 1) > lim[1]) {
      Serial.println(logM("AXIS AT LIMIT"));
      return;
    }
  }

  mX = false;
  mY = true;
  mZ = false;

  //steps counter set to 0
  int accCnt = 0;

  // correction value generation, starting timer
  int vdv = initializeAccLin(feedrate);
  Serial.println(logM("dV = " + String(vdv) + " steps/s"));
  //setting initial speed value correctly
  int frt;
  if (vdv == 0) {
    frt = feedrate;
  } else {
    frt = limspeed;
  }

  ///////////////////////////// переделать управление шагами в случае если была калибровка
  while (true) {
    if (Serial.read() == '#') {
      Serial.println(logM("Move ended"));
      stopTimerLin();


      if (flagZero) {
        float tme = (micros() - tst);
        float sps = actstep[1] - ip;
        Serial.println(logM("Move duration " + String(tme) + " s, steps made " + String(sps)));
        float result = sps / tme;
        Serial.println(logM("Actual speed steps/s was " + String(result * 1000000, 2)));
        //
      }

      break;
    }
    if (tickWas) {

      //communicate();

      // counting ticks
      accCnt++;
      // updating timeout if needed
      updateAccLin(vdv, frt, feedrate, accsteps, accCnt);
      frt += vdv;

      //updateStateStep();

      tickWas = false;

      if (flagZero) {
        if ((actstep[1] + 1) > lim[1]) {
          stopTimerLin();
          Serial.println(logM("AXIS AT LIMIT"));
          break;
        } else {
          actstep[1] += 1;
        }
      }
    }
  }
}

void stepElevate() {

  Serial.println("up active");

  stepperZ.setDirection(0);

  // if zeroed, check if at least 1 step is possible

  if (flagZero) {
    if ((actstep[2] + 1) > lim[2]) {
      Serial.println(logM("AXIS AT LIMIT"));
      return;
    }
  }

  mX = false;
  mY = false;
  mZ = true;

  //steps counter set to 0
  int accCnt = 0;

  // correction value generation, starting timer
  int vdv = initializeAccLin(feedrate);

  //setting initial speed value correctly
  int frt;
  if (vdv == 0) {
    frt = feedrate;
  } else {
    frt = limspeed;
  }

  ///////////////////////////// переделать управление шагами в случае если была калибровка
  while (true) {

    if (Serial.read() == '#') {
      Serial.println("Move ended");
      stopTimerLin();
      break;
    }
    if (tickWas) {
      //communicate();
      tickWas = false;

      // counting ticks
      accCnt++;
      // updating timeout if needed
      updateAccLin(vdv, frt, feedrate, accsteps, accCnt);
      frt += vdv;

      //updateStateStep();

      // step is counted and checked only if a zeroing was done
      if (flagZero) {
        if ((actstep[2] + 1) > lim[2]) {
          stopTimerLin();
          Serial.println(logM("AXIS AT LIMIT"));
          break;
        } else {
          actstep[2] += 1;
        }
      }
    }
  }
}

void stepLower() {
  Serial.println("low active");

  stepperZ.setDirection(1);

  // if zeroed, check if at least 1 step is possible

  if (flagZero) {
    if ((actstep[2] - 1) < 0) {
      Serial.println(logM("AXIS AT LIMIT"));
      return;
    }
  }



  mX = false;
  mY = false;
  mZ = true;

  //steps counter set to 0
  int accCnt = 0;

  // correction value generation, starting timer
  int vdv = initializeAccLin(feedrate);

  //setting initial speed value correctly
  int frt;
  if (vdv == 0) {
    frt = feedrate;
  } else {
    frt = limspeed;
  }

  ///////////////////////////// переделать управление шагами в случае если была калибровка
  while (true) {

    if (Serial.read() == '#') {
      Serial.println("Move ended");
      stopTimerLin();
      //set actual Z speed to 0
      break;
    }
    //service - tick count
    if (tickWas) {

      //communicate();
      // counting ticks
      accCnt++;
      // updating timeout if needed
      updateAccLin(vdv, frt, feedrate, accsteps, accCnt);
      frt += vdv;

      //updateStateStep();

      tickWas = false;

      if (flagZero) {
        if ((actstep[2] - 1) < 0) {
          stopTimerLin();
          Serial.println(logM("AXIS AT LIMIT"));
          break;
        } else {
          actstep[2] -= 1;
        }
      }
    }
  }
}

float brsAngle(long int x1, long int y1, long int x2, long int y2) {
  float thetaA = atan2(y1, x1);
  //Serial.print("=====tA");
  //Serial.println(thetaA);
  float thetaB = atan2(y2, x2);
  //Serial.print("=====tB");
  //Serial.println(thetaB);
  //thetaA = normalizeAngle(thetaA);
  //Serial.print("=====tAN");
  //Serial.println(thetaA);
  //thetaB = normalizeAngle(thetaB);
  //Serial.print("=====tBN");
  //Serial.println(thetaB);
  float centralAngle = abs(thetaA - thetaB);
  if (centralAngle >= 1.57) {
    centralAngle = abs(2 * PI - centralAngle);
  }
  //Serial.print("=====centralAngle");
  //Serial.println(centralAngle);
  //Serial.print("Angle of a step (rad) is ");
  //Serial.println(centralAngle);
  return centralAngle;
}

// when connected, the actual parameters are downloaded into the app
void connectUpdate() {

  delay(50);
  // update ecopen dosage, ml/L

  Serial.println("epen" + String(EcoPen.getValue()));

  delay(50);
  // update pump
  Serial.println("vpro" + String(PumpPro.getValue()));

  delay(50);

  // update mixer
  Serial.println("mix" + String(Mixer.getValue()));

  delay(50);

  Serial.println("frate" + String(feedrate / mtos));

  delay(50);
  // zeroing

  if (flagZero) {
    Serial.println("zzz");
  } else {
    Serial.println("nzz");
  }

  /// ASK THE APP ABOUT ACTUAL +- STEP in%
  // first - all three together

  delay(50);
  demand("stepA?", "Ecopen step, %", stepEco);
  delay(50);
  demand("stepB?", "Pump step, %", stepPump);
  delay(50);
  demand("stepC?", "Mixer step, %", stepMix);

  delay(200);

  serialInitialized = true;

  while (Serial.available()) {
    Serial.read();  // просто считываем и выбрасываем все байты
  }
  Serial.println(logM("Buffer clean"));
};

////////////////////////////////////////////////////////////////////////////////////////////////
//Timer circular

void stopTimer() {
  TIMSK3 &= ~(1 << OCIE3A);  // Отключаем прерывание для таймера 3
  TCCR3B = 0;                // Останавливаем таймер 3
}

void setupTimer(unsigned long intervalMicroseconds) {
  TCCR3A = 0;  // Сбросить регистр управления
  TCCR3B = 0;  // Сбросить регистр управления
  TCNT3 = 0;   // Сбросить счётчик таймера

  // Режим CTC
  TCCR3B |= (1 << WGM32);

  // Рассчитываем значение для OCR3A
  unsigned long compareMatchValue = (16000000 / (1024 * (1000000 / intervalMicroseconds))) - 1;
  OCR3A = compareMatchValue;

  // Установить предделитель 256
  TCCR3B |= (1 << CS32);

  // Разрешаем прерывание по совпадению с OCR3A
  TIMSK3 |= (1 << OCIE3A);
}




// Information timer

void setupTimerInfo(unsigned long intervalMicroseconds) {
  // Сбросить все регистры управления
  TCCR4A = 0;
  TCCR4B = 0;

  // Установить режим CTC (Clear Timer on Compare Match)
  TCCR4B |= (1 << WGM12);

  // Сбросить счётчик таймера
  TCNT4 = 0;

  // Проверяем, что интервал в пределах допустимых значений
  if (intervalMicroseconds > 0) {
    // Рассчитываем значение для OCR4A для заданного интервала
    unsigned long compareMatchValue = (16000000 / 1024 / (1000000 / intervalMicroseconds)) - 1;

    // Ограничиваем значение OCR4A максимальным значением (16-битный таймер)
    if (compareMatchValue > 65535) {
      compareMatchValue = 65535;
    }

    OCR4A = compareMatchValue;  // Устанавливаем значение для OCR4A
  }

  // Устанавливаем предделитель на 1024
  TCCR4B |= (1 << CS12) | (1 << CS10);  // 1024

  // Разрешаем прерывание по совпадению с OCR4A
  TIMSK4 |= (1 << OCIE4A);
}

void stopTimer4() {
  // Сбросить все биты управления таймером
  TCCR4B = 0;                // Останавливаем таймер
  TIMSK4 &= ~(1 << OCIE4A);  // Отключаем прерывание
}


void setupTimerCal(unsigned long intervalMicroseconds) {
  TCCR1A = 0;  // Сбросить регистр управления
  TCCR1B = 0;  // Сбросить регистр управления
  TCNT1 = 0;   // Сбросить счётчик таймера

  // Режим CTC
  TCCR1B |= (1 << WGM12);

  // Рассчитываем значение для OCR1A
  unsigned long compareMatchValue = (16000000 / (64 * (1000000 / intervalMicroseconds))) - 1;
  OCR1A = compareMatchValue;

  // Установить предделитель 64
  TCCR1B |= (1 << CS11) | (1 << CS10);

  // Разрешаем прерывание по совпадению с OCR1A
  TIMSK1 |= (1 << OCIE1A);
}


//updates linear timer
void OCRLin(unsigned long to) {
  unsigned long cmv = (16000000 / (1024 * (1000000 / to))) - 1;
  OCR2A = cmv;
}

//updates calibration timer
void OCRCal(unsigned long to) {
  unsigned long cmv = (16000000 / (1024 * (1000000 / to))) - 1;
  OCR1A = cmv;
}

void stopTimerCal() {
  TIMSK1 &= ~(1 << OCIE1A);  // Отключаем прерывание для таймера 1
  TCCR1B = 0;                // Останавливаем таймер 1
}

void stopTimerLin() {
  TIMSK2 &= ~(1 << OCIE2A);  // Отключаем прерывание для таймера 2
  TCCR2B = 0;                // Останавливаем таймер 2
}

void setupTimerLin(unsigned long intervalMicroseconds) {
  TCCR2A = 0;  // Сбросить регистр управления
  TCCR2B = 0;  // Сбросить регистр управления
  TCNT2 = 0;   // Сбросить счётчик таймера

  // Режим CTC
  TCCR2A |= (1 << WGM21);

  // Рассчитываем значение для OCR2A
  unsigned long compareMatchValue = (16000000 / (1024 * (1000000 / intervalMicroseconds))) - 1;
  OCR2A = compareMatchValue;

  // Установить предделитель 1024
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);

  // Разрешаем прерывание по совпадению с OCR2A
  TIMSK2 |= (1 << OCIE2A);
}


//asking ADAC for values

ISR(TIMER4_COMPA_vect) {
  flagI = true;
}

//Calibration ISR

ISR(TIMER1_COMPA_vect) {
  if (pX) {
    stepperX.step();
    if (digitalRead(ESPin11) == LOW) {
      pX = 0;
      Serial.println(logM("X triggered"));
    }
  }
  if (pY) {
    stepperY.step();
    if (digitalRead(ESPin12) == LOW) {
      pY = 0;
      Serial.println(logM("Y triggered"));
    }
  }

  if (pZ) {
    stepperZ.step();
    if (digitalRead(ESPin10) == LOW) {
      pZ = 0;
    }
  }
  if (pX == 0 && pY == 0 && pZ == 0) {
    stopTimerCal();
    timerRun = 0;
  }
  lastStep = micros();
}

// Linear ISR
ISR(TIMER2_COMPA_vect) {
  if (mX) {
    stepperX.step();
  }
  if (mY) {
    stepperY.step();
  }
  if (mZ) {
    stepperZ.step();
  }
  tickWas = true;
  lastStep = micros();
}

// Circular ISR
ISR(TIMER3_COMPA_vect) {
  if (mX) {
    stepperX.step();
  }
  if (mY) {
    stepperY.step();
  }
  if (mZ) {
    stepperZ.step();
  }
  tickWas = true;
  lastStep = micros();
}

//changes the output to be shown in app log
String logM(String s) {

  return "#" + s;
}

//Special function that is used to evacuate the table
void evacuate() {
  // it is a move that saves the printed piece by evacuating the table and leaving the concrete fall into the bin
  //keeping the old speed
  oldValue = feedrate;

  // setting rapid move speed (pre-defined for now)
  feedrate = G0Speed;

  // target coordinates:

  dpos[0] = 150;
  dpos[1] = actstep[1];
  dpos[2] = actstep[2];

  // standard move with high speed
  lineWrapper();
  flagH = 1;
  //restore previous speed
  feedrate = oldValue;
}

/////////////////////////////////////////

// it calculates a velocity step that should be added first N steps

int dv(int iniSpeed, int trgSpeed, int nstep) {
  int k;
  k = round(float(trgSpeed - iniSpeed) / nstep);
  //Serial.println(logM("To increase the feedrate from "+String(iniSpeed)+" to "+String(trgSpeed)+" a step is calculated : "+String(k)));
  return k;
}

// function launches a timer with initial TO for linear moves
//starts timer and returns acceleration value dv
int initializeAccLin(int V1) {

  // if speed stays in the limit, we just start the timer with the needed speed. dv=0
  // initial speed
  int V0;
  int vUpd = 0;
  Serial.println(logM("Target speed is " + String(V1) + " steps/s, limit speed is " + String(limspeed)));
  if (V1 >= limspeed) {
    // if speed is higher, we take limspeed as initial speed, and this
    vUpd = dv(limspeed, V1, accsteps);
    //starting value is limit speed
    V0 = limspeed;
    //the initial speed is set to limspeed
  } else {
    V0 = V1;
  }

  // Calculate timeout of speed
  unsigned long tmo = frTO(V0);
  Serial.println(logM(String(tmo) + " micros"));
  //starting the linear timer
  setupTimerLin(tmo);

  //the timer is started, then first accsteps it should be updated.
  return vUpd;
}


// using acceleration for calibration

int initializeAccCal(int V1) {

  // if speed stays in the limit, we just start the timer with the needed speed. dv=0
  // initial speed
  int V0;
  int vUpd = 0;

  if (V1 >= limspeed) {
    // if speed is higher, we take limspeed as initial speed, and this
    vUpd = dv(limspeed, V1, accsteps);
    //starting value is limit speed
    V0 = limspeed;
    //the initial speed is set to limspeed
  } else {
    V0 = V1;
  }

  // Calculate timeout of speed
  unsigned long tmo = frTO(V0);

  //starting the linear timer
  setupTimerCal(tmo);

  //the timer is started, then first accsteps it should be updated.
  return vUpd;
}

// update linear timer

void updateAccLin(int vl, int fdrt, int targsp, int mxstep, int crstep) {
  //Speed to assign
  int vc;
  if (mxstep >= crstep) {
    //Serial.println(logM("Transient step done"));
    if (fdrt >= targsp) {
      //final speed is assigned at last step
      vc = targsp;
      //Serial.println(logM("Target speed achieved : "+String(vc)));
    } else {
      //during the steps, the speed is increasing
      vc = fdrt + vl;
      //Serial.println(logM("Updated speed : "+String(vc)));
    }
    // new timeout
    unsigned long nTO = frTO(vc);
    //Serial.println(logM("Timeout : "+String(nTO)));
    // timer is updated
    OCRLin(nTO);
  }
}

// update calibration timer

void updateAccCal(int vl, int fdrt, int targsp, int mxstep, int crstep) {
  //Speed to assign
  int vc;
  if (mxstep >= crstep) {
    if (fdrt >= targsp) {
      //final speed is assigned at last step
      vc = targsp;
    } else {
      //during the steps, the speed is increasing
      vc = fdrt + vl;
    }
    // new timeout
    unsigned long nTO = frTO(vc);
    // timer is updated
    OCRCal(nTO);
  }
}


// function that transforms a feedrate steps/s into appropriate timeout
unsigned long frTO(int fedr) {
  unsigned long to = round(1000000 / fedr);
  return to;
}

//switch on the mixer
void startMotor() {
  // check if the servo is active, if yes
  if (digitalRead(SRDY) == HIGH) {
    Serial.println(logM("Mixer successfully started"));
    digitalWrite(SRVONPin, HIGH);
    Serial.println(">");
    flagMix = true;
  } else {
    Serial.println(logM("Mixer not responding - check connection"));
  }
}

//stop the mixer

void stopMotor() {
  if (flagMix) {
    Serial.println(logM("Mixer stopped"));
    digitalWrite(SRVONPin, LOW);
    Serial.println(">");
    flagMix = false;
  }
}

// measuring pressure if asked
float getPress() {
  float p = AD5593RR.read_ADC(3) * pressCoef;
  return p;
}

float getTorque() {
  float t = AD5593RR.read_ADC(4) * torqCoef;
  return t;
}

// if ecopen or pump dosage is updated, this function recalculates values in ml/Lmand
void recalcEco() {

  float ecomll = (EcoPen.getValue() * 0.6);
  Serial.println(logM("Desired ecopen is " + String(ecomll) + " ml/L"));
  float pumplm = (PumpPro.getValue() * 0.0168);
  Serial.println(logM("Current pump is " + String(pumplm) + " L/min"));
  float debit = (EcoPen.getValue() * 0.6) * (PumpPro.getValue() * 0.0168);
  Serial.println(logM("Debit is " + String(debit)));
  //recalculate now on a scale. Maximal ecopen rate is 60 ml/min = 100.
  float acs = 100 * debit / 60;
  //if too high, set to 100
  if (acs >= 100) {
    acs = 100;
  }
  AD5593RR.write_DAC(EcoPen.getId(), 5 * (acs / 100));
  Serial.println(logM("EcoPen dosage updated, new value ml/min : " + String(debit) + ", % : " + String(acs)));
}


// COMMUNICATION IMPROVED

void demand(String question, String name, float& param) {

  Serial.println(question);
  delay(50);
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Читаем строку до символа новой строки
    input.trim();                                 // Убираем пробелы и переносы строк
    if (input.length() == 0) {
      Serial.println(logM("Unable to read parameter " + name + " from the app. " + name + " set to default"));
    } else {
      float num = input.toFloat();
      param = num;
      Serial.println(logM(name + " is set to " + String(num) + "%"));
    }
  } else {
    Serial.println(logM("Unable to read parameter " + name + " from the app. " + name + " set to default"));
  }
}

//special mode to de-grip the shaft of the pump
void pumpActive() {
  // give a short pulse
  AD5593RR.write_DAC(PumpPro.getId(), 2.5);
  delay(100);
  AD5593RR.write_DAC(PumpPro.getId(), 0);
  //
  Serial.println(logM("Pulse!"));
}

// checks input and takes actions
void inputCommand() {
  //Check if a command is needed
  static String input = "";

  //prevent from not resetting
  unsigned long cs = millis();

  while (Serial.available()) {
    //Serial.println(logM("Available"));
    char c = Serial.read();

    //if stuck
    if (millis() - cs > 1) {
      Serial.println(logM("Exiting loop"));
      return;
    }
    if (c == '\r' || c == '\n') {
      //Serial.println(logM("End"));
      if (input.length() > 0) {
        Serial.println(logM("Command to execute: " + input));
        cm.handleCommand(input);
        input = "";
        return;
      }
    } else {
      input += c;
      //Serial.println(logM(input));
    }
  }
}

// The state reporting string is functioning as following - if doing steps after each step one char from a line is sent. Line is global and generated when last was

// service function to create a string including pressure, motor torque, coordinates and any other information

// ADD HERE ANY OTHER TRACKING DATA


// service function to know if timers are running
bool timersActive() {
  bool timer1 = (TCCR1B & ((1 << CS10) | (1 << CS11) | (1 << CS12))) != 0;
  bool timer2 = (TCCR2B & ((1 << CS20) | (1 << CS21) | (1 << CS22))) != 0;
  bool timer3 = (TCCR3B & ((1 << CS30) | (1 << CS31) | (1 << CS32))) != 0;
  return timer1 || timer2 || timer3;
}

void cleanBuffer() {
  if (!serialInitialized) {
    while (Serial.available()) {
      Serial.read();  // просто считываем и выбрасываем все байты
    }
  }
}


/// function to stop the pump in case when the tank is empty
void stopEmpty() {
  Serial.println(logM("Tank empty received. Stopping pump"));
  PumpPro.setValue(0);
  AD5593RR.write_DAC(EcoPen.getId(), 5 * (EcoPen.getValue() / 100));
  AD5593RR.write_DAC(PumpPro.getId(), 5 * (PumpPro.getValue()) / 100);
}

// function that gets values and sends them - to be included in loop fragments - after steps
void communicate() {

  // get ADC data

  if (flagI) {

    pressure = getPress();
    torque = getTorque();

    flagI = false;
  }
  // send info
  sendData(pressure, torque, round(actstep[0] / mtos), round(actstep[1] / mtos), round(actstep[2] / mtosZ));
}

//Sends data while in loop every ms millis

void reportLoop(int ms) {

  if (millis() - tmt >= ms) {
    // create a string

    communicate();

    tmt = millis();
  }
}

/////////////////////////// EXPERIMENTAL - BINARY COM

void sendData(float pressure, float force, int16_t x, int16_t y, int16_t z) {
  uint8_t packet[12];
  uint8_t index = 0;

  // Стартовый байт
  packet[index++] = 0xAA;

  // Масштабируем давление и силу (умножаем на 100 и преобразуем в uint16_t)
  uint16_t p_scaled = (uint16_t)(pressure * 100);
  uint16_t f_scaled = (uint16_t)(force * 100);

  // Записываем давление (2 байта, младший и старший)
  packet[index++] = lowByte(p_scaled);
  packet[index++] = highByte(p_scaled);

  // Записываем силу (2 байта)
  packet[index++] = lowByte(f_scaled);
  packet[index++] = highByte(f_scaled);

  // Записываем координаты (по 2 байта на каждую)
  packet[index++] = lowByte(x);
  packet[index++] = highByte(x);

  packet[index++] = lowByte(y);
  packet[index++] = highByte(y);

  packet[index++] = lowByte(z);
  packet[index++] = highByte(z);

  // Вычисляем контрольную сумму XOR по байтам, кроме стартового
  uint8_t checksum = 0;
  for (int i = 1; i < 11; i++) {
    checksum ^= packet[i];
  }
  packet[index++] = checksum;

  // Отправляем весь пакет сразу
  Serial.write(packet, index);
}
