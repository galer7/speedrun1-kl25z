// aici adaugam functiile de scanare care vor fi folosite in (aproape) toate celelalte programe

#include "common.h"
#include <cstring>

// ____________________ CONSTANTS __________________:

// -- CAMERA
#define NUM_LINE_SCAN 128 // # of samples
#define MAX_LINE_SCAN NUM_LINE_SCAN-1
#define MIN_LINE_WIDTH 2
#define MAX_LINE_WIDTH  5
#define MIN_START_WIDTH 16
#define MAX_START_WIDTH 25
#define FILTER_ENDS 5   // dead pixels to ignore
#define RANGE (NUM_LINE_SCAN - (2 * FILTER_ENDS))  // range of good pixels
#define EDGE_WIDTH_ERROR 3

// -- STEER
#define MAX_STEER_LEFT -0.34
#define MAX_STEER_RIGHT 0.34
#define MAX_MARGIN_RIGHT -48
#define MAX_MARGIN_LEFT  48

// -- SPEED
#define SUB_LIGHT_SPEED 0.5
#define LIGHT_SPEED 0.6
#define RIDICULOUS_SPEED 0.7
#define LUDICROUS_SPEED 0.9

// -- LOGGING
#define NUM_LOG_FRAMES 700

bool debugMode = false;
Timer timer;

#define UNKNOWN_COUNT_MAX  40                      // max value to allow for unknown track conditions before killing engine
#define STARTGATEFOUNDMAX  0                       // max value to allow for finding starting gate before killing engine
#define STARTGATEDELAY     50                      // Delay before searching for starting gate to kill engine

// ___________________VARIABLES__________________:


// -- CAMERA
uint16_t   GrabLineScanImage0[NUM_LINE_SCAN]; // snapshot of camera data for this 'frame'
float      DerivLineScanImage0[NUM_LINE_SCAN]; // derivative of line scan data
int      NegEdges[NUM_LINE_SCAN] = {}; // array that holds at which index the NegEdge is on the LineScan
int      PosEdges[NUM_LINE_SCAN] = {}; // array that holds at which index the PosEdge is on the LineScan

int nrDifferentNegEdges = 0;
int nrDifferentPosEdges = 0;

uint16_t   MaxLightIntensity = 0;
uint16_t   MinLightIntensity = (1 << 12);
float      maxDerVal = 0;                          // max deriv value
float      minDerVal = (float)(1 << 12);          // min deriv value
float      aveDerVal = 0;                          // average deriv value
float      averIntensity = 0.0;
float      DerivThreshold = (1 << 9);              // Derivative Threshold (default)
float      PosDerivThreshold = (1 << 9);           // Pos Edge Derivative Threshold (default)
float      NegDerivThreshold = (1 << 9);           // Neg Edge Derivative Threshold (default)
float      cameraCenter = (float)(127 / 2);       // center pixel
float offMarginDistance;


// Steering control variables
float      marginPosition;                    // Current position of track line (in pixels -- 0 to 127)
float      lastMarginPosition;                       // Last position of track line (in pixels -- 0 to 127)
float      AbsError;
float      lastOffMarginDistance = 0;                   // Last line position error (used for derivative calc)
float      SumLinePosError = 0;                    // Sum of line position error (used for integral calc)
float      DerivError = 0;                         // Derivative of error
float      CurrentSteerSetting;  // drive straight at first
float      CurrentLeftDriveSetting = 0;            // Drive setting (left wheel)
float      CurrentRightDriveSetting = 0;           // Drive setting (right wheel)

float kSteerLeft = (MAX_STEER_LEFT / cameraCenter); // the negative slope for the left quadrant
float kSteerRight = -(MAX_STEER_RIGHT / cameraCenter); // the negative slope for the right quadrant

// Speed control vars
float       MaxSpeed;                                    // maximum speed allowed
int         lastSteerSetting;
uint16_t    last10 = 0;
float       Pout;

// -- DEV
bool dev = false;


uint16_t   startRaceTicker;                        // ticker at start of race1

// Custom Data Types
enum TrackStatusType { Unknown, LineFound, StartGateFound, LeftLine, RightLine, FilteredLine, StraightRoad, SpeedUp, SlowDown };
typedef enum { eMainRace, eSpeedLimit, eEightRace, eObstacleAvoidance } RaceType;

TrackStatusType currentTrackStatus;                // current track status
TrackStatusType LastTrackStatus;                   // last track status
RaceType currentRaceType;

struct LogData
{
  float linepos;
  float steersetting;
  float leftdrivesetting;
  float rightdrivesetting;
};

LogData    frameLogs[NUM_LOG_FRAMES];              // array of log data to store
int        logDataIndex;                           // index for log data


int        StartGateFoundCount = 0;                // how many times start gate has been found
int        UnknownCount = 0;                       // how many times nothing has been found (to help with kill switch implementation)
bool       go = false;                             // Car can go!  Should be set to false to start.
int SlowDownZoneFound = false;
int SpeedUpZoneFoundCount = 0;

// EXTRA CONTROL PARAMETERS
bool debugFakeMode = false;         // if true, ignores real camera and uses fake camera input instead; used for data processing debug
int terminalOutput = 0;             // set debug level for terminal output
//    0 : no terminal output, race!
//    1 : output just to measure frame rate
//    2 : output for measuring time of operations
//    3 : output with delay
bool doLogData = false;             // whether to capture log data to output later on
bool killSwitch = false;             // whether to enable Kill Switch (allow engine to stop after not finding track)
bool startGateStop = true;         // whether to stop or not depending on starting gate reading
bool doRisky = false;               // race style-- whether conservative or risky



// timer stuff

int after_time, before_time, start_time, last_start_time;
bool run_once = false;




// ________________________FUNCTIONS_________________:

// -- DEFINITIONS:

void decideLineFromEdges();
void decideSteerSetting();
void decideSpeedSetting();
void drive();

// -- FETCH DATA:

uint16_t* acquireSamplesAndIntensity()
{
  // putem calcula avgIntensity de aici deja...
  averIntensity = 0;
  for (int i = 0; i < NUM_LINE_SCAN; i++)
  {
    GrabLineScanImage0[i] = TFC_LineScanImage0[i];

    if (i >= FILTER_ENDS && i <= NUM_LINE_SCAN - FILTER_ENDS)
    {
      averIntensity += GrabLineScanImage0[i];
    }

  }

  averIntensity = averIntensity / (NUM_LINE_SCAN - 2*FILTER_ENDS); // compute avg light intensity
  DerivThreshold = averIntensity / 8; // TODO: find perfect arguments...
  NegDerivThreshold = (float)-1 * (DerivThreshold);
  PosDerivThreshold = (float)(DerivThreshold);

  return GrabLineScanImage0;
}

void derivScanAndFindEdges(uint16_t* LineScanDataIn, float* DerivLineScanDataOut)
{
  // clean edge arrays
  //for (int k = 0; k < nrDifferentNegEdges - 1; k++)
  //  NegEdges[k] = 0;

  //for (int k = 0; k < nrDifferentPosEdges - 1; k++)
  //  PosEdges[k] = 0;

  std::memset(NegEdges, 0, sizeof(NegEdges));
  std::memset(PosEdges, 0, sizeof(PosEdges));

  int indPosEdges = 0;
  int indNegEdges = 0;

  nrDifferentPosEdges = 0;
  nrDifferentNegEdges = 0;

  float DerVal = 0;
  float upperDerVal, lowerDerVal;
  int minCnt = FILTER_ENDS;
  int maxCnt = NUM_LINE_SCAN - FILTER_ENDS;

  for (int i = minCnt; i < maxCnt; i++)
  {

    if (i == minCnt)                         // start point
    {
      upperDerVal = (float)(LineScanDataIn[i + 1]);
      lowerDerVal = (float)(LineScanDataIn[i]);
    }
    else if (i == maxCnt - 1)
    {
      upperDerVal = (float)(LineScanDataIn[i]);
      lowerDerVal = (float)(LineScanDataIn[i - 1]);
    }
    else
    {
      upperDerVal = (float)(LineScanDataIn[i + 1]);
      lowerDerVal = (float)(LineScanDataIn[i - 1]);
    }
    DerVal = (upperDerVal - lowerDerVal) / 2;
    DerivLineScanDataOut[i] = DerVal;

    if (DerVal >= PosDerivThreshold) {
      if (nrDifferentPosEdges == 0)
      {
        nrDifferentPosEdges++;
        PosEdges[indPosEdges] = i;

        continue;
      }

      if (nrDifferentPosEdges && i - PosEdges[indPosEdges] > MAX_LINE_WIDTH)
      {
        // daca nu sunt consecutive, salveaza si incrementeaza nr de edgeuri diferite
        indPosEdges++;
        PosEdges[indPosEdges] = i;
        nrDifferentPosEdges++;
      }
      else {
        /*  caz unic pos edge:
        * daca gasim inca un edge fix langa dar mai la dreapta, pune-l pe ala pt ca e mai aproape de centru => pericol mai mare. Pt ca mereu un pos edge va fi in stanga. Linie neagra si apoi alba => POS EDGE
        */
        PosEdges[indPosEdges] = i;
      }
    }

    if (DerVal <= NegDerivThreshold) {
      if (nrDifferentNegEdges == 0)
      {
        nrDifferentNegEdges++;
        NegEdges[indNegEdges] = i;

        continue;
      }

      if (nrDifferentNegEdges > 0 && i - NegEdges[indNegEdges] > MAX_LINE_WIDTH)
      {
        // daca nu sunt consecutive, salveaza-le
        indNegEdges++;
        NegEdges[indNegEdges] = i;
        nrDifferentNegEdges++;
      }
    }
  }
}

// -- PROCESS AND DECIDE:

void decideLineFromEdges()
{
  lastMarginPosition = marginPosition;


  // cazuri care se aplica la toate race type-urile:

  if (nrDifferentNegEdges == 0 && nrDifferentPosEdges == 0)
  {
    currentTrackStatus = StraightRoad;
  }

  else if (nrDifferentNegEdges == 1 && nrDifferentPosEdges == 1)
  {
    // we have a found a line...
    marginPosition = (PosEdges[0] + NegEdges[0]) / 2;
    currentTrackStatus = LineFound;
  }

  else if (nrDifferentPosEdges == 1 && nrDifferentNegEdges == 0 && PosEdges[0] < cameraCenter)
  {
    // pre detected left line
    marginPosition = PosEdges[0] - MAX_LINE_WIDTH / 2;
    currentTrackStatus = LeftLine;
  }

  else if (nrDifferentNegEdges == 1 && nrDifferentPosEdges == 0 && NegEdges[0] > cameraCenter)
  {

    // pre detected right line
    marginPosition = NegEdges[0] + MAX_LINE_WIDTH / 2;
    currentTrackStatus = RightLine;
  }

  // cazuri care se aplica doar la race type-uri particulare:
  if (currentRaceType == eMainRace) {

    if (nrDifferentNegEdges == 2 && nrDifferentPosEdges == 2 /* && (((NegEdges[0] - PosEdges[0]) <= MAX_LINE_WIDTH && (NegEdges[0] - PosEdges[0]) >= MIN_LINE_WIDTH) && ((NegEdges[1] - PosEdges[1]) <= MAX_LINE_WIDTH && (NegEdges[1] - PosEdges[1]) >= MIN_LINE_WIDTH))*/)
    {
      currentTrackStatus = StartGateFound;
      StartGateFoundCount++;
      if (startGateStop)
      {
        if (StartGateFoundCount > 0)
        {
          go = false;   // STOP!!
          return;
        }
      }
    }
  }
  
  else if (currentRaceType == eSpeedLimit) {
    if (nrDifferentNegEdges == 3 && nrDifferentPosEdges == 3 /* && (((NegEdges[0] - PosEdges[0]) <= MAX_LINE_WIDTH && (NegEdges[0] - PosEdges[0]) >= MIN_LINE_WIDTH) && ((NegEdges[1] - PosEdges[1]) <= MAX_LINE_WIDTH && (NegEdges[1] - PosEdges[1]) >= MIN_LINE_WIDTH))*/ )
    {
      currentTrackStatus = SlowDown;
    }

    if (nrDifferentNegEdges == 4 && nrDifferentPosEdges == 4 /* && (((NegEdges[0] - PosEdges[0]) <= MAX_LINE_WIDTH && (NegEdges[0] - PosEdges[0]) >= MIN_LINE_WIDTH) && ((NegEdges[1] - PosEdges[1]) <= MAX_LINE_WIDTH && (NegEdges[1] - PosEdges[1]) >= MIN_LINE_WIDTH)) */)
    {
      currentTrackStatus = SpeedUp;
    }
  }

  else if (currentRaceType == eEightRace) {
    //nimic special lol
    return;
  }

  else if (currentRaceType == eObstacleAvoidance)
  {
    return;

  }

  if (marginPosition > 127) marginPosition = 127;
  if (marginPosition < 0) marginPosition = 0;
}


// SPEED AND CONTROL

void decideSteerAndSpeed()
{
  // common decision
  
  if (currentRaceType == eMainRace && currentTrackStatus == StartGateFound)
  {
    // really quick slowdown.
    TFC_SetMotorPWM(-2 * MaxSpeed, -2 * MaxSpeed);
    //wait_ms(300);
    // just go to drive.
  }
  else
  {
    decideSteerSetting();
    decideSpeedSetting();
  }

  drive();

}


void decideSteerSetting()
{
  // daca >0 => vrem viraj DREAPTA
  // daca <0 => vrem viraj STANGA
  offMarginDistance = cameraCenter - marginPosition;
  lastSteerSetting = CurrentSteerSetting;

  // sa testezi aici kSteerLeft si Right. Normal, ar trebui ca numitorul lor sa fie cameraCenter/2 + shiftuite, deoarece daca te aflii la > -cameraCenter/2 sau < cameraCenter/2, masina este deja scoasa de pe  traseu...

  // corectie: remember anul trecut cand la viraj, el vedea linia pe mijloc chiar cand intra in curba, deci cred ca e ok sa lasam asa

  if (currentTrackStatus == StraightRoad) {
    CurrentSteerSetting = 0;
  }
  else {
    if (offMarginDistance > 0) {
      CurrentSteerSetting = kSteerRight * offMarginDistance + MAX_STEER_RIGHT; // valori pozitive
    }
    else if (offMarginDistance < 0) {
      CurrentSteerSetting = kSteerLeft * offMarginDistance + MAX_STEER_LEFT; // valori negative
    }
    else {
      CurrentSteerSetting = 0;
    }
  }
  PC.printf("\nCurrent steer setting: %3.10f\n", CurrentSteerSetting);


  if (CurrentSteerSetting >= MAX_STEER_RIGHT)
  {
    CurrentSteerSetting = MAX_STEER_RIGHT;
  }
  else if (CurrentSteerSetting <= MAX_STEER_LEFT)
  {
    CurrentSteerSetting = MAX_STEER_LEFT;
  }

  

  TFC_SetServo(0, CurrentSteerSetting);

}

void decideSpeedSetting()
{
  if (currentTrackStatus == StraightRoad)
  {
    CurrentLeftDriveSetting = MaxSpeed;
    CurrentRightDriveSetting = MaxSpeed;
    return;
  }

  if (currentRaceType == eSpeedLimit)
  {
    if (currentTrackStatus == SpeedUp)
    {
      CurrentLeftDriveSetting *= 2;
      CurrentRightDriveSetting *= 2;
      return;
    }

    if (currentTrackStatus == SlowDown)
    {
      CurrentLeftDriveSetting /= 2;
      CurrentRightDriveSetting /= 2;
      return;
    }
  }


  // set maximum speed allowed
  switch (2)
  {
  case 0:
    // read value off pot0
    MaxSpeed = TFC_ReadPot(0);
    break;
  case 1:
    MaxSpeed = 0.4;
    break;
  case 2:
    MaxSpeed = SUB_LIGHT_SPEED; // 0.5
    break;
  case 3:
    MaxSpeed = LIGHT_SPEED; // 0.6
    break;
  case 4:
    MaxSpeed = RIDICULOUS_SPEED; // 0.7
    break;
  case 5:
    MaxSpeed = LUDICROUS_SPEED; // 0.9
    break;
  default:
    MaxSpeed = 0.4;
    break;
  }

  // nothing special for drifting...
  CurrentLeftDriveSetting = (float)(MaxSpeed); // aici era cu * MaxSpeed
  CurrentRightDriveSetting = (float)(MaxSpeed);

  if (go)
    TFC_SetMotorPWM(-CurrentRightDriveSetting, CurrentLeftDriveSetting);
}

void drive()
{

  // START!
  // if not going, go when button A is pressed
  if (!go)
  {
    if (TFC_PUSH_BUTTON_0_PRESSED)
    {
      go = true;
      UnknownCount = 0;
      StartGateFoundCount = 0;
      startRaceTicker = TFC_Ticker[0];  // keep track of start of race
      TFC_HBRIDGE_ENABLE;
    }
    else {
      
      TFC_SetMotorPWM(0, 0); //Make sure motors are off
      TFC_HBRIDGE_DISABLE;
    }
  }

  // STOP!
  // if going, stop when button A is pressed
  if (go)
  {
    if (TFC_PUSH_BUTTON_1_PRESSED)
    {
      go = false;
      StartGateFoundCount = 0;
    }
  }
}

// FOR DEBUG ONLY:

void printLineScanData(uint16_t* LineScanData)
{
  uint32_t i = 0;
  float Val;

  TERMINAL_PRINTF("LINE SCAN DATA:,");

  for (i = 0; i < NUM_LINE_SCAN; i++) // print one line worth of data (128) from Camera 0
  {
    if (1 == 1)   // use float to print
    {
      Val = (float)LineScanData[i];
      TERMINAL_PRINTF("%9.3f", Val);
      if (i == MAX_LINE_SCAN)  // when last data reached put in line return
        TERMINAL_PRINTF("\r\n");
      else
        TERMINAL_PRINTF(",");
    }
    else
    {
      TERMINAL_PRINTF("0x%X", LineScanData[i]);
      if (i == MAX_LINE_SCAN)  // when last data reached put in line return
        TERMINAL_PRINTF("\r\n", LineScanData[i]);
      else
        TERMINAL_PRINTF(",", LineScanData[i]);
    }
  }

}

void printDerivLineScanData(float* derivLineScanData)
{
  uint32_t i, minCnt = 0, maxCnt = 0;

  minCnt = FILTER_ENDS;
  maxCnt = NUM_LINE_SCAN - FILTER_ENDS;

  TERMINAL_PRINTF("DERIVATIVE DATA:,");

  for (i = minCnt; i < maxCnt; i++) // print one line worth of data (128) from Camera 0
  {
    TERMINAL_PRINTF("%9.3f", derivLineScanData[i]);
    if (i == maxCnt - 1)          // when last data reached put in line return
      TERMINAL_PRINTF("\r\n", derivLineScanData[i]);
    else
      TERMINAL_PRINTF(", ", derivLineScanData[i]);
  }

}

void feedbackLights()
{
  switch (currentTrackStatus)
  {
  case LeftLine:
    TFC_BAT_LED0_ON;
    TFC_BAT_LED1_OFF;
    TFC_BAT_LED2_OFF;
    TFC_BAT_LED3_OFF;
    break;
  case RightLine:
    TFC_BAT_LED0_OFF;
    TFC_BAT_LED1_OFF;
    TFC_BAT_LED2_OFF;
    TFC_BAT_LED3_ON;
    break;
  case LineFound:
    TFC_BAT_LED0_ON;
    TFC_BAT_LED1_OFF;
    TFC_BAT_LED2_OFF;
    TFC_BAT_LED3_ON;
    break;
  case StartGateFound:
    TFC_BAT_LED0_OFF;
    TFC_BAT_LED1_ON;
    TFC_BAT_LED2_ON;
    TFC_BAT_LED3_OFF;
    break;
  case StraightRoad:
    TFC_BAT_LED0_ON;
    TFC_BAT_LED1_ON;
    TFC_BAT_LED2_ON;
    TFC_BAT_LED3_ON;
    break;
  case FilteredLine:
    TFC_BAT_LED0_ON;
    TFC_BAT_LED1_OFF;
    TFC_BAT_LED2_ON;
    TFC_BAT_LED3_OFF;
    break;
  default:
    TFC_BAT_LED0_OFF;
    TFC_BAT_LED1_OFF;
    TFC_BAT_LED2_OFF;
    TFC_BAT_LED3_OFF;
    break;
  }

}

void forceFeedbackLights(int var)
{
  switch (var) {
  case 8:
    TFC_BAT_LED0_ON;
    TFC_BAT_LED1_OFF;
    TFC_BAT_LED2_OFF;
    TFC_BAT_LED3_OFF;
    break;
  case 9:
    TFC_BAT_LED0_ON;
    TFC_BAT_LED1_OFF;
    TFC_BAT_LED2_OFF;
    TFC_BAT_LED3_ON;
    break;
  case 10:
    TFC_BAT_LED0_ON;
    TFC_BAT_LED1_OFF;
    TFC_BAT_LED2_ON;
    TFC_BAT_LED3_OFF;
    break;
  case 11:
    TFC_BAT_LED0_ON;
    TFC_BAT_LED1_OFF;
    TFC_BAT_LED2_ON;
    TFC_BAT_LED3_ON;
    break;
  default:
    TFC_BAT_LED0_OFF;
    TFC_BAT_LED1_OFF;
    TFC_BAT_LED2_OFF;
    TFC_BAT_LED3_OFF;
    break;
  }
}

const char* stringFromTrackStatus(TrackStatusType status)
{
  const char* strings[] = { "Unknown", "LineFound", "StartGateFound", "LeftLine", "RightLine", "FilteredLine", "StraightRoad", "SpeedUp", "SlowDown" };

  return strings[status];
}