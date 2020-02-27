import serial
import matplotlib.pyplot as plt

xAxis = [x for x in range(128)]
cameraLine = [0 for x in range(128)]
plt.axis([0, 127, -2000, 5000])

MAX_LINE_WIDTH = 5

def derivLineScan(lineScan):
  derivLine = [0 for x in range(128)]
  prev = next = 0
  for ind, x in enumerate(lineScan):
    try:
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
    except:
      return [0 for x in range(128)]

  return derivLine

def returnEdges(cameraLine, derivLine, thNeg, thPos):
  nrDifferentNegEdges = 0
  nrDifferentPosEdges = 0

  thisFrameNegEdges = []
  thisFramePosEdges = []

  for ind, (x, derivValue) in enumerate(zip(cameraLine, derivLine)):
    if derivValue >= thPos:
      if thisFramePosEdges == []:
        nrDifferentPosEdges += 1
        thisFramePosEdges.append([ind, x])
        continue
        
      if (ind - thisFramePosEdges[-1][0]) >= MAX_LINE_WIDTH:
        thisFramePosEdges.append([ind, x])
        nrDifferentPosEdges += 1
      else:
        thisFramePosEdges[-1] = [ind, x]
    
    if derivValue <= thNeg:
      if thisFrameNegEdges == []:
        nrDifferentNegEdges += 1
        thisFrameNegEdges.append([ind, x])
        continue
        
      if (ind - thisFrameNegEdges[-1][0]) >= MAX_LINE_WIDTH:
        thisFrameNegEdges.append([ind, x])
        nrDifferentNegEdges += 1

  return thisFrameNegEdges, thisFramePosEdges, nrDifferentNegEdges, nrDifferentPosEdges

def TrackStatus(negEdges, posEdges):
  if len(negEdges) == 0 and len(posEdges) == 0:
    return "STRAIGHT ROAD"
  elif len(negEdges) == 1 and len(posEdges) == 1:
    return "LINE DETECTED"
  elif len(negEdges) == 1 and len(posEdges) == 0:
    return "LEFT LINE"
  elif len(negEdges) == 0 and len(posEdges) == 1:
    return "RIGHT LINE"
  elif len(negEdges) == 2 and len(posEdges) == 2:
    return "START GATE"
  else:
    return "UNKNOWN"

with serial.Serial('COM3', 115200) as ser:
  frameNr = 0
  while True:
    if not frameNr >= 16:
      # ser.flushInput()
      # ser.flushOutput()
      line = ser.readline()
      # print([int(x) for x in line.decode('ascii').split(',')])
      frameNr += 1
      try:
        cameraLine = [int(x) for x in line.decode('utf-8').split(',')]
        avgLine = [sum(cameraLine) / 128 for x in range(128)]
        plotDerivLineScan = derivLineScan(cameraLine)

        plt.clf()
        plt.axis([0, 127, -2000, 5000])

        plt.plot(xAxis, cameraLine)
        plt.plot(xAxis, avgLine)
        plt.plot(xAxis, plotDerivLineScan)
        plt.plot(xAxis, [x/10 for x in avgLine], [-x/10 for x in avgLine])

        derivThPos = avgLine[0] / 8
        derivThNeg = -derivThPos
        try:

          negEdges, posEdges, nrDifferentNegEdges, nrDifferentPosEdges = returnEdges(cameraLine, plotDerivLineScan, derivThNeg, derivThPos)

          if len(posEdges):
            plt.scatter(*zip(*posEdges), color='green')
          
          if len(negEdges):
            plt.scatter(*zip(*negEdges), color='red')

          print('N: ', nrDifferentNegEdges, 'P: ', nrDifferentPosEdges)
          currentTrackStatus = TrackStatus(negEdges, posEdges)
          print(currentTrackStatus)
        except Exception as e:
          print(e)

        plt.pause(0.00001)
      except UnicodeDecodeError as e:
        # print(e)
        print(line)
        pass
      except ValueError  as e:
        print(e)
        pass
      except KeyboardInterrupt as e:
        print(e)
        pass
      except Exception as e:
        print(e)
        break
    else: 
      ser.flushInput()
      ser.flushOutput()
      frameNr = 0
      continue
  plt.show()