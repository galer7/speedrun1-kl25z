#include "race_functions.cpp"
#include "FRDM-TFC/TFC.h"

void main()
{
  while (1) {
    chooseAndRunRace()
  }
}

void chooseAndRunRace()
{
  switch (TFC_GetDIP_Switch())
  {
    case: 0x08
      MainRace();
      break;
    case: 0x09
      SpeedLimit();
      break;
    case: 0x10
      ObstacleRace();
      break;
    case: 0x11
      EightRace();
      break;
    default:
      printf("Faied to select a program...");
      break;
  }
}


void MainRace()
{
  // main timed race
  // TODO: implement this race without ticker just to see the new behaviour

  grabCameraFrame();
  derivativeLineScan(&GrabLineScanImage0[0], &DerivLineScanImage0[0]);
  adjustLights();
  findEdges_v2(&DerivLineScanImage0[0]);
  reviewEdges();
  ActOnTrackStatus();
  feedbackLights();
  SpeedControl();
  Drive();


}

void SpeedLimit()
{
  // search for a pattern to lower speed
}

void ObstacleRace()
{
  // search for a cube from camera. what will be the output of the camera?
}

void EightRace()
{
  // normal MainRace() program, maybe we can improve the steering a bit.
}