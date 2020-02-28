
#include "common.h"
#include "race_functions.h"
#include TFC_PATH
#include "serialCamera.h"


void printPosNegEdges();
void printEdgesNrAndStatus();
void MainRace()
{
  // main timed race
  currentRaceType = RaceType::eMainRace;

  acquireSamplesAndIntensity(); // gets samples and computes avg. light intensity
  derivScanAndFindEdges(&GrabLineScanImage0[0], &DerivLineScanImage0[0]); // computes the derivative and finds edges
  //findEdges_v2(&DerivLineScanImage0[0]);
  decideLineFromEdges();
  decideSteerAndSpeed();

  feedbackLights();
  
  printEdgesNrAndStatus();
  printPosNegEdges();
  PC.printf("\nCurrent steer setting: %3.10f\n", CurrentSteerSetting);
  PC.printf("kSteerRight: %.3f", kSteerRight);
  PC.printf("kSteerLeft: %.3f", kSteerLeft);
  //wait_ms(500);


}

void SpeedLimit()
{
  currentRaceType = RaceType::eSpeedLimit;

  // search for a pattern to lower speed
  acquireSamplesAndIntensity();
  derivScanAndFindEdges(&GrabLineScanImage0[0], &DerivLineScanImage0[0]); // computes the derivative and finds edges
  decideLineFromEdges();
  decideSteerAndSpeed();

  feedbackLights();



}

void ObstacleRace()
{
  currentRaceType = RaceType::eObstacleAvoidance;

  // search for a cube from camera. what will be the output of the camera?
}

void EightRace()
{
  currentRaceType = RaceType::eEightRace;

  // normal MainRace() program, maybe we can improve the steering a bit.
}



void chooseAndRunRace()
{

  switch (TFC_GetDIP_Switch())
  {
    case 0x08:
        serialCamera();
        forceFeedbackLights(8);
        break;
        
      

    case 0x09:
      SpeedLimit();
      forceFeedbackLights(9);
      break;

    case 0x0A:
      ObstacleRace();
      forceFeedbackLights(10);
      break;

    case 0x0B:
      EightRace();
      forceFeedbackLights(11);
      break;
      
    case 0x0F:
      MainRace();
      break;

    default:
      printf("Faied to select a program...");
      break;
  }
}


int main()
{
  PC.baud(115200); // works with Excel and TeraTerm 
   
  TFC_Init();
   
  dev = true;
  while (1) {
    chooseAndRunRace();
  }
}

void printPosNegEdges()
{
  PC.printf("\nNEG EDGES: ");
  for (int i = 0; i < nrDifferentNegEdges; i++)
  {
    PC.printf("%d ", NegEdges[i]);
  }
  PC.printf("\nPOS EDGES: ");
  for (int i = 0; i < nrDifferentPosEdges; i++)
  {
    PC.printf("%d ", PosEdges[i]);
  }
  PC.printf("\n");

}

void printEdgesNrAndStatus() {
  PC.printf("POS:%d NEG:%d STATUS:%s", nrDifferentNegEdges, nrDifferentPosEdges, stringFromTrackStatus(currentTrackStatus));
}