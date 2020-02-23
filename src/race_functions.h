// aici adaugam functiile de scanare care vor fi folosite in (aproape) toate celelalte programe

#include "common.h"
#include TFC_PATH
#include MBED_PATH

// ____________________ CONSTANTS __________________:

// -- CAMERA
#define NUM_LINE_SCAN 128 // # of samples
#define MAX_LINE_SCAN NUM_LINE_SCAN-1
#define MIN_LINE_WIDTH 2
#define MAX_LINE_WIDTH  15
#define MIN_START_WIDTH 16
#define MAX_START_WIDTH 25
#define FILTER_ENDS 0   // dead pixels to ignore
#define RANGE (NUM_LINE_SCAN - (2 * FILTER_ENDS))  // range of good pixels

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
float      NegEdges[NUM_LINE_SCAN]; // array-- set of where in line scan data negative edges found
float      PosEdges[NUM_LINE_SCAN]; // array-- set of where in line scan data positive edges found
uint16_t   numNegEdges = 0, numPosEdges = 0; // max value of valid neg and positive indices (also serves as a count of # edges found)
uint16_t   MaxLightIntensity = 0;
uint16_t   MinLightIntensity = (1 << 12);
float      maxDerVal = 0;                          // max deriv value
float      minDerVal = (float)(1 << 12);          // min deriv value
float      aveDerVal = 0;                          // average deriv value
float      DerivThreshold = (1 << 9);              // Derivative Threshold (default)
float      PosDerivThreshold = (1 << 9);           // Pos Edge Derivative Threshold (default)
float      NegDerivThreshold = (1 << 9);           // Neg Edge Derivative Threshold (default)
float      cameraCenter = (float)(127 / 2);


// Steering control variables
float      marginPosition;                    // Current position of track line (in pixels -- 0 to 127)
float      lastMarginPosition;                       // Last position of track line (in pixels -- 0 to 127)
float      offMarginDistance = 0;                // Current line position error (used for derivative calc)
float      AbsError;
float      lastOffMarginDistance = 0;                   // Last line position error (used for derivative calc)
float      SumLinePosError = 0;                    // Sum of line position error (used for integral calc)
float      DerivError = 0;                         // Derivative of error
float      CurrentSteerSetting = (MAX_STEER_RIGHT + MAX_STEER_LEFT) / 2;  // drive straight at first
float      CurrentLeftDriveSetting = 0;            // Drive setting (left wheel)
float      CurrentRightDriveSetting = 0;           // Drive setting (right wheel)

// Speed control vars
float       MaxSpeed;                                    // maximum speed allowed
int         lastSteerSetting;
uint16_t    last10 = 0;
float       Pout;

uint16_t   startRaceTicker;                        // ticker at start of race1

// Custom Data Types
typedef enum { Unknown, LineFound, StartGateFound, LeftLine, RightLine, FilteredLine, StraightRoad } TrackStatusType;

TrackStatusType CurrentTrackStatus;                // current track status
TrackStatusType LastTrackStatus;                   // last track status

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

// EXTRA CONTROL PARAMETERS
bool debugFakeMode = false;         // if true, ignores real camera and uses fake camera input instead; used for data processing debug
int terminalOutput = 0;             // set debug level for terminal output
//    0 : no terminal output, race!
//    1 : output just to measure frame rate
//    2 : output for measuring time of operations
//    3 : output with delay
bool doLogData = false;             // whether to capture log data to output later on
bool killSwitch = false;             // whether to enable Kill Switch (allow engine to stop after not finding track)
bool startGateStop = false;         // whether to stop or not depending on starting gate reading
bool doRisky = false;               // race style-- whether conservative or risky



// timer stuff

int after_time, before_time, start_time, last_start_time;
bool run_once = false;


// ________________________FUNCTIONS_________________:

// -- FETCH DATA:

void grabCameraFrame()
{
    uint32_t i = 0;
    uint8_t fake_type = 4;                   // type of fake data if used

    for (i = 0; i < NUM_LINE_SCAN; i++) // print one line worth of data (128) from Camera 0
    {
        GrabLineScanImage0[i] = TFC_LineScanImage0[i];
    }


}

void derivativeLineScan(uint16_t* LineScanDataIn, float* DerivLineScanDataOut)
{

  uint32_t i, minCnt = 0, maxCnt = 0;
  float DerVal, upperDerVal, lowerDerVal = 0;

  maxDerVal = 0;
  minDerVal = (float)(1 << 12);
  aveDerVal = 0;

  minCnt = FILTER_ENDS;
  maxCnt = NUM_LINE_SCAN - FILTER_ENDS;

  // TERMINAL_PRINTF("i, upperDerVal, lowerDerVal, DerVal\r\n");

  for (i = minCnt; i < maxCnt; i++) // print one line worth of data from Camera 0
  {

    // store max light intensity value
    if (LineScanDataIn[i] > MaxLightIntensity)
      MaxLightIntensity = LineScanDataIn[i];

    // store min light intensity value
    if (LineScanDataIn[i] < MinLightIntensity)
      MinLightIntensity = LineScanDataIn[i];


    // Central Derivative
    if (i == minCnt)                         // start point
    {
      upperDerVal = (float)(LineScanDataIn[i + 1]);
      lowerDerVal = (float)(LineScanDataIn[i]);  // make same as start point
    }
    else if (i == maxCnt - 1)                // end point
    {
      upperDerVal = (float)(LineScanDataIn[i]);   // make same as end point
      lowerDerVal = (float)(LineScanDataIn[i - 1]);
    }
    else                                   // any other point
    {
      upperDerVal = (float)(LineScanDataIn[i + 1]);
      lowerDerVal = (float)(LineScanDataIn[i - 1]);
    }
    DerVal = (upperDerVal - lowerDerVal) / 2;
    DerivLineScanDataOut[i] = DerVal;
  }
}

// -- PROCESS AND DECIDE:

void adjustLights()
{

  DerivThreshold = (float)(MaxLightIntensity - MinLightIntensity) / 10;
  NegDerivThreshold = (float)-1 * (DerivThreshold);
  PosDerivThreshold = (float)(DerivThreshold);

}


void findEdges_v2(float* derivLineScanData)
{
  int i;

  int minCnt = FILTER_ENDS;
  int maxCnt = NUM_LINE_SCAN - FILTER_ENDS;

  numNegEdges = 0;                              // count of neg edges found thus far
  numPosEdges = 0;                              // count of pos edges found thus far

  float holderNeg[NUM_LINE_SCAN];
  float holderPos[NUM_LINE_SCAN];

  int divNeg = 0;
  int divPos = 0;

  int sumNeg = 0;
  int sumPos = 0;

  for (i = minCnt; i < maxCnt; i++) // print one line worth of data from Camera 0
  {

    if (derivLineScanData[i] <= NegDerivThreshold)          // NEGATIVE EDGE FOUND!
      holderNeg[i] = i;
    else
      holderNeg[i] = 300;

    if (derivLineScanData[i] > PosDerivThreshold)
      holderPos[i] = i;
    else
      holderPos[i] = 300;
  }

  for (i = minCnt;i < maxCnt;i++)
  {
    if (holderNeg[i] != 300)
    {
      divNeg++;
      sumNeg += holderNeg[i];
    }
    else
    {
      if (divNeg != 0)
      {
        NegEdges[numNegEdges] = sumNeg / divNeg;
        numNegEdges++;
        sumNeg = 0;
        divNeg = 0;
      }

    }

    if (holderPos[i] != 300)
    {
      divPos++;
      sumPos += holderPos[i];
    }
    else
    {
      if (divPos != 0)
      {
        PosEdges[numPosEdges] = sumPos / divPos;
        numPosEdges++;
        sumPos = 0;
        divPos = 0;
      }

    }

  }
}

void reviewEdges()
{
  int i;
  LastTrackStatus = CurrentTrackStatus;
  CurrentTrackStatus = Unknown;
  marginPosition = cameraCenter;

  if (numPosEdges == 0 && numNegEdges == 0)
  {
    // de adaugat functie de average pe fiecare jumatate daca nu intra in unknown (eu zic ca e f probabil sa nu intre in unknown)
    CurrentTrackStatus = StraightRoad;
    lastMarginPosition = marginPosition;
    marginPosition = lastMarginPosition;
    UnknownCount = 0;
  }
  else if (numPosEdges == 1 && numNegEdges == 1)  // margine gasita
  {
    if (
      ((PosEdges[0] - NegEdges[0]) >= MIN_LINE_WIDTH) &&
      ((PosEdges[0] - NegEdges[0]) <= MAX_LINE_WIDTH)
      )
    {
      CurrentTrackStatus = LineFound;                                   // report line found!
      UnknownCount = 0;                                          // reset unknown status count
      lastMarginPosition = marginPosition;
      marginPosition = (PosEdges[0] + NegEdges[0]) / 2;       // update line position
    }

    if (
      ((NegEdges[0] - PosEdges[0]) >= MIN_START_WIDTH) &&
      ((NegEdges[0] - PosEdges[0]) <= MAX_START_WIDTH)
      )
    {
      if (startRaceTicker > STARTGATEDELAY)                        // only start counting for starting gate until after delay
        StartGateFoundCount++;
      marginPosition = cameraCenter;
      CurrentTrackStatus = StartGateFound;
      UnknownCount = 0;
    }

    else
    {
      CurrentTrackStatus = Unknown;
      UnknownCount++;
      marginPosition = lastMarginPosition;
    }
  }
  else if (numPosEdges == 1 && numNegEdges == 0)          // margine la STANGA
  {
    CurrentTrackStatus = LeftLine;                                   // report line found!
    UnknownCount = 0;                                                 // reset unknown status count
    lastMarginPosition = marginPosition;
    marginPosition = PosEdges[0] - MAX_LINE_WIDTH / 2;
    if (marginPosition < 0)
      marginPosition = 0;
  }
  if (numNegEdges == 1 && numPosEdges == 0)          // margine la DREAPTA
  {
    CurrentTrackStatus = RightLine;
    UnknownCount = 0;
    lastMarginPosition = marginPosition;
    marginPosition = NegEdges[0] + MAX_LINE_WIDTH / 2;
    if (marginPosition > 127)
      marginPosition = 127;
  }
  else if (numPosEdges == 2 && numNegEdges == 2)
  {

    if (
      (((PosEdges[0] - NegEdges[0]) >= MIN_START_WIDTH) && ((PosEdges[0] - NegEdges[0]) <= 4.7 * MAX_LINE_WIDTH)) &&    // black left 'line'
      (((PosEdges[1] - NegEdges[1]) >= 4.7 * MIN_LINE_WIDTH) && ((PosEdges[1] - NegEdges[1]) <= 4.7 * MAX_LINE_WIDTH)) &&    // white right 'line'
      (((NegEdges[1] - PosEdges[0]) >= 3.7 * MIN_LINE_WIDTH) && ((NegEdges[1] - PosEdges[0]) <= 3.7 * MAX_LINE_WIDTH))       // actual track line
      )
    {
      if (startRaceTicker > STARTGATEDELAY)                        // only start counting for starting gate until after delay
        StartGateFoundCount++;
      marginPosition = cameraCenter;
      CurrentTrackStatus = StartGateFound;
      UnknownCount = 0;
    }
    else
    {
      CurrentTrackStatus = Unknown;
      UnknownCount++;
      marginPosition = lastMarginPosition;
    }
  }
  else if (numPosEdges == 1 && numNegEdges > 1)
  {
    for (i = 0;i < numNegEdges;i++)
    {
      if (((PosEdges[0] - NegEdges[i]) >= MIN_LINE_WIDTH) && (PosEdges[0] - NegEdges[i] <= MAX_LINE_WIDTH))
      {
        CurrentTrackStatus = FilteredLine;
        UnknownCount = 0;
        lastMarginPosition = marginPosition;
        marginPosition = (PosEdges[0] + NegEdges[i]) / 2;
        break;
      }
    }

    if (CurrentTrackStatus != FilteredLine)
    {
      CurrentTrackStatus = Unknown;
      UnknownCount++;
      marginPosition = lastMarginPosition;
    }
  }
  else if (numNegEdges == 1 && numPosEdges > 1)
  {
    for (i = 0;i < numPosEdges;i++)
    {
      if (((PosEdges[i] - NegEdges[0]) >= MIN_LINE_WIDTH) && (PosEdges[i] - NegEdges[0] <= MAX_LINE_WIDTH))
      {
        CurrentTrackStatus = FilteredLine;
        UnknownCount = 0;
        lastMarginPosition = marginPosition;
        marginPosition = (PosEdges[i] + NegEdges[0]) / 2;
        break;
      }
    }

    if (CurrentTrackStatus != FilteredLine)
    {
      CurrentTrackStatus = Unknown;
      UnknownCount++;
      marginPosition = lastMarginPosition;
    }
  }
}



// SPEED AND CONTROL

void SteeringControl()
{

  float offMarginDistance = cameraCenter - marginPosition;
  lastSteerSetting = CurrentSteerSetting;
  // proportional control term

  // if (offMarginDistance > -34.5 && offMarginDistance < 0)
  // {
  //     Pout =  -0.2*0.001 * offMarginDistance * offMarginDistance + MAX_STEER_RIGHT;
  // }
  // else if (offMarginDistance > 0 && offMarginDistance < 28)
  // {
  //     Pout =  0.17*0.001 * offMarginDistance * offMarginDistance - MAX_STEER_LEFT;
  // } else
  // {
  //     Pout = (2.6887) / (offMarginDistance);
  // }

  // - 40.25 extrama dreapta
  // 26 extrama stanga
  // 59.5 vede stanga prima data
  // - 62 vede dreapta prima data

  if (offMarginDistance < 0)
  {
    Pout = MAX_STEER_LEFT;
  }
  if (offMarginDistance > 0)
  {
    Pout = MAX_STEER_RIGHT;
  }
  if (offMarginDistance == 0)
  {
    if (CurrentTrackStatus == StraightRoad)
      Pout = 0;
  }

  CurrentSteerSetting = Pout;
}

void Steer()
{

  // make sure doesn't go beyond steering limits
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

void ActOnTrackStatus()
{

  if (CurrentTrackStatus == StartGateFound)       // STARTING GATE FOUND
  {
    if (startGateStop)
    {
      if (StartGateFoundCount > STARTGATEFOUNDMAX)
      {
        go = false;   // STOP!!
      }
    }

  }
  SteeringControl();
  Steer();


}

void SpeedControl()
{

  // Get max speed setting from reading pot0
  // then adjust

  float ErrLimit;
  float LeftDriveRatio, RightDriveRatio;

  // set maximum speed allowed
  switch (0)
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
    break;
  }

  if (offMarginDistance < 0)
  {
    if (offMarginDistance > MAX_MARGIN_RIGHT) // daca este aproape de marginea dreapta, viteza negativa pe stanga (interior), normal pe dreapta (exterior)
    {
      // atunci cand vede marginea perpendicular
      LeftDriveRatio = -200 * MaxSpeed;
      RightDriveRatio = 400 * MaxSpeed;
    }
    else
    {
      LeftDriveRatio = 0 * MaxSpeed;
      RightDriveRatio = 300 * MaxSpeed;
    }
  }
  if (offMarginDistance > 0)
  {
    if (offMarginDistance < MAX_MARGIN_LEFT)
    {
      // daca este aproape de marginea stanga, viteza negativa pe dreapta (interior), normal pe stanga (exterior)
      LeftDriveRatio = 400 * MaxSpeed;
      RightDriveRatio = -200 * MaxSpeed;
    }
    else
    {
      LeftDriveRatio = 300 * MaxSpeed;
      RightDriveRatio = 0 * MaxSpeed;
    }
  }

  if (offMarginDistance == 0)
  {
    LeftDriveRatio = 150 * MaxSpeed;
    RightDriveRatio = 150 * MaxSpeed;
  }


  // TBD-- add speed adjust based on Xaccel sensor!

  if (CurrentTrackStatus == Unknown && LastTrackStatus == Unknown)
  {
    LeftDriveRatio = -80 * MaxSpeed;
    RightDriveRatio = -80 * MaxSpeed;
  }

  // currently no control mechanism as don't have speed sensor
  CurrentLeftDriveSetting = -(float)(LeftDriveRatio / 100); // aici era cu * MaxSpeed
  CurrentRightDriveSetting = (float)(RightDriveRatio / 100);
}

void Drive()
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
      logDataIndex = 0;                 // reset log data index
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

  // EMERGENCY STOP!
  // 'kill switch' to prevent crashes off-track
  if (killSwitch)
  {
    if (UnknownCount > UNKNOWN_COUNT_MAX)    // if track not found after certain time
    {
      go = false;                            // kill engine
      StartGateFoundCount = 0;
    }
  }

  // ****************

  if (!go)   // stop!
  {
    TFC_SetMotorPWM(0, 0); //Make sure motors are off
    TFC_HBRIDGE_DISABLE;
  }

  if (go)    // go!
  {
    TFC_HBRIDGE_ENABLE;
    // motor A = right, motor B = left based on way it is mounted
    TFC_SetMotorPWM(CurrentRightDriveSetting, CurrentLeftDriveSetting);

    // if  (
    //         (CurrentTrackStatus == LeftLine && offMarginDistance < MAX_MARGIN_LEFT) ||
    //         (CurrentTrackStatus == RightLine && offMarginDistance > MAX_MARGIN_RIGHT)
    //     )
    // {
    //     wait(0.3);
    // }
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

void printEdgesFound()
{
  int i;

  // Check that neg edges captured ok
  TERMINAL_PRINTF("NEGATIVE EDGES FOUND:,");
  for (i = 0; i <= numNegEdges - 1; i++)
  {
    TERMINAL_PRINTF("%9.3f", NegEdges[i]);
    if (i == numNegEdges - 1)              // when last data reached put in line return
      TERMINAL_PRINTF("\r\n");
    else
      TERMINAL_PRINTF(", ");
  }


  // Check that pos edges captured ok
  TERMINAL_PRINTF("POSITIVE EDGES FOUND:,");
  for (i = 0; i <= numPosEdges - 1; i++)
  {
    TERMINAL_PRINTF("%9.3f", PosEdges[i]);
    if (i == numPosEdges - 1)              // when last data reached put in line return
      TERMINAL_PRINTF("\r\n");
    else
      TERMINAL_PRINTF(", ");
  }

}

void feedbackLights()
{
  switch (CurrentTrackStatus)
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