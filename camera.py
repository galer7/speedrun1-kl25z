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

def returnEdges(cameraLine, derivLine, thNeg, thPos):
  thisFrameNegEdges = []
  thisFramePosEdges = []

  for ind, (x, derivValue) in enumerate(zip(cameraLine, derivLine)):
    if derivValue >= thPos:
      thisFramePosEdges.append([ind, x])
    
    if derivValue <= thNeg:
      thisFrameNegEdges.append([ind, x])

  return thisFrameNegEdges, thisFramePosEdges

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
        negEdges, posEdges = returnEdges(cameraLine, plotDerivLineScan, -avgLine[0]/10, avgLine[0]/10)

        plt.clf()
        plt.axis([0, 127, -2000, 5000])

        plt.plot(xAxis, cameraLine)
        plt.plot(xAxis, avgLine)
        plt.plot(xAxis, plotDerivLineScan)
        plt.plot(xAxis, [x/10 for x in avgLine], [-x/10 for x in avgLine])
        try:

          x, y = zip(*posEdges)
          plt.scatter(x, y, color='green')
          x, y = zip(*negEdges)

          plt.scatter(x, y, color='red')
        except:
          pass

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