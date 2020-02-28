import serial
import matplotlib.pyplot as plt
import sys

cameraCenter = 128.0 / 2

margin_index = 0
maxSteerRight = 0.34
maxSteerLeft = -0.34
kSteerRight = -maxSteerRight / cameraCenter
kSteerLeft = maxSteerLeft / cameraCenter

xAxis = [x for x in range(128)]
cameraLine = [0 for x in range(128)]

plt.axis([-cameraCenter, 127, -2000, 5000])

MAX_LINE_WIDTH = 5

def steerSetting():
  global offmarginDistance
  if offmarginDistance > 0:
    # steer right
    steer = kSteerRight*offmarginDistance + maxSteerRight
  if offmarginDistance < 0:
    # steer left
    steer = kSteerLeft*offmarginDistance + maxSteerLeft
  return steer


def derivLineScanAndRetEdges(lineScan):
  nrDifferentNegEdges = 0
  nrDifferentPosEdges = 0

  thisFrameNegEdges = []
  thisFramePosEdges = []

  derivLine = [0 for x in range(128)]
  prev = 0
  next = 0
  
  for ind, x in enumerate(lineScan):
    if ind == 0:
      prev = x
      next = lineScan[ind+1]
    
    elif ind == 127:
      prev = lineScan[ind - 1]
      next = x
    
    else:
      prev = lineScan[ind - 1]
      next = lineScan[ind + 1]

    currentDer = (next - prev) / 2
    derivLine[ind] = currentDer

    if currentDer >= derivThPos:
      if thisFramePosEdges == []:
        nrDifferentPosEdges += 1
        thisFramePosEdges.append([ind, x])
        continue
        
      if (ind - thisFramePosEdges[-1][0]) >= MAX_LINE_WIDTH:
        thisFramePosEdges.append([ind, x])
        nrDifferentPosEdges += 1
      else:
        thisFramePosEdges[-1] = [ind, x]
    
    if currentDer <= derivThNeg:
      if thisFrameNegEdges == []:
        nrDifferentNegEdges += 1
        thisFrameNegEdges.append([ind, x])
        continue
        
      if (ind - thisFrameNegEdges[-1][0]) >= MAX_LINE_WIDTH:
        thisFrameNegEdges.append([ind, x])
        nrDifferentNegEdges += 1
        

  return derivLine, thisFrameNegEdges, thisFramePosEdges, nrDifferentNegEdges, nrDifferentPosEdges



def TrackStatus(negEdges, posEdges):
  global margin_index
  if len(negEdges) == 0 and len(posEdges) == 0:
    return "STRAIGHT ROAD"
  elif len(negEdges) == 1 and len(posEdges) == 1:
    margin_index = (negEdges[0][0] + posEdges[0][0])/2
    return "LINE DETECTED"
  elif len(negEdges) == 1 and len(posEdges) == 0:
    margin_index = negEdges[0][0]  - MAX_LINE_WIDTH/2
    return "LEFT LINE"
  elif len(negEdges) == 0 and len(posEdges) == 1:
    margin_index = posEdges[0][0]  + MAX_LINE_WIDTH/2
    return "RIGHT LINE"
  elif len(negEdges) == 2 and len(posEdges) == 2:
    return "START GATE"
  else:
    return "UNKNOWN"

with serial.Serial('COM4', 115200) as ser:
  frameNr = 0
  while True:
    margin_index = 0
    print(frameNr)
    if not frameNr >= 16:

      # ser.flushInput()
      # ser.flushOutput()
      line = ser.readline()
      # print([int(x) for x in line.decode('ascii').split(',')])
      frameNr += 1
      try:
        cameraLine = [int(x) for x in line.decode('utf-8').split(',')]
        avgIntensity = sum(cameraLine) / 128
        derivThPos = avgIntensity / 8
        derivThNeg = -derivThPos
        
        plt.clf()
        plt.axis([-cameraCenter, 127, -2000, 5000])

        plt.plot(xAxis, cameraLine)
        plt.plot([0, 127], [avgIntensity, avgIntensity])
        plt.plot([0, 127], [derivThPos, derivThPos], [derivThNeg, derivThNeg])
        
        try:
          # print(derivLineScanAndRetEdges(cameraLine))
          plotDerivLineScan, negEdges, posEdges, nrDifferentNegEdges, nrDifferentPosEdges = derivLineScanAndRetEdges(cameraLine)
          plt.plot(xAxis, plotDerivLineScan)
          
          if len(posEdges):
            plt.scatter(*zip(*posEdges), color='green')
        
          if len(negEdges):
            plt.scatter(*zip(*negEdges), color='red')
          
          currentTrackStatus = TrackStatus(negEdges, posEdges)
          offmarginDistance = cameraCenter - margin_index

          plt.plot([offmarginDistance, offmarginDistance], [0, 2000])
          plt.text(120, 2000, f'{steerSetting()}')

          print('N: ', nrDifferentNegEdges, 'P: ', nrDifferentPosEdges, 'offmarginDistance: ', offmarginDistance)
          print(currentTrackStatus)
        
        except Exception as e:
          exc_type, exc_obj, exc_tb = sys.exc_info()
          print(e)
          print(exc_type, exc_tb.tb_lineno)

        
        plt.pause(0.000001)

      except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        print(e)
        print(exc_type, exc_tb.tb_lineno)

        pass
    else: 
      ser.flushInput()
      ser.flushOutput()
      frameNr = 0
      continue
  plt.show()