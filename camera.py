import serial
import matplotlib.pyplot as plt

xAxis = [x for x in range(128)]
cameraLine = [0 for x in range(128)]
plt.axis([0, 127, -2000, 5000])

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

def returnEdges(cameraLine, derivLine, th):
  thisFrameEdges = []
  for ind, x, derivValue in zip(enumerate(cameraLine), derivLine):
    if derivValue > abs(th):
      thisFrameEdges.append([ind, x])

  return thisFrameEdges

with serial.Serial('COM3', 115200) as ser:
  frameNr = 0
  while True:
    if not frameNr >= 32:
      # ser.flushInput()
      # ser.flushOutput()
      line = ser.readline()
      frameNr += 1
      try:

        cameraLine = [int(x) for x in line.decode('utf-8').split(',')]
        avgLine = [sum(cameraLine) / 128 for x in range(128)]
        plotDerivLineScan = derivLineScan(cameraLine)
        posEdges = returnEdges(cameraLine, plotDerivLineScan, avgLine[0]/10)
        negEdges = returnEdges(cameraLine, plotDerivLineScan, -avgLine[0]/10)


        print(cameraLine)
        plt.clf()
        plt.axis([0, 127, -2000, 5000])


        plt.plot(xAxis, cameraLine)
        plt.plot(xAxis, avgLine)
        plt.plot(xAxis, plotDerivLineScan)
        plt.plot(xAxis, [x/10 for x in avgLine], [-x/10 for x in avgLine])

        plt.plot(*zip(*posEdges))
        plt.plot(*zip(*negEdges))

        plt.pause(0.00001)
      except UnicodeDecodeError:
        pass
      except ValueError:
        pass
      except KeyboardInterrupt:
        pass
    else: 
      ser.flushInput()
      ser.flushOutput()
      frameNr = 0
      continue
  plt.show()