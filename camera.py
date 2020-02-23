import serial
import matplotlib.pyplot as plt

xAxis = [x for x in range(128)]
cameraLine = [0 for x in range(128)]
plt.axis([0, 127, 0, 4000])

with serial.Serial('COM3', 115200) as ser:
  while(True):

    ser.flushInput()
    # ser.flushOutput()
    line = ser.readline()
    try:
      # print(line.decode('utf-8'))
      cameraLine = [x for x in line.decode('utf-8').split(',')]

      cameraLine[0] = cameraLine[0][1:]
      cameraLine[-1] = cameraLine[-1][:-2]
      cameraLine = [int(x) for x in cameraLine]
      # print(cameraLine)
      plt.clf()
      plt.axis([0, 127, 0, 4000])

      plt.plot(xAxis, cameraLine)
      plt.pause(0.00000001)
    except UnicodeDecodeError:
      pass
    except ValueError:
      pass
    except KeyboardInterrupt:
      pass
  plt.show()  

  


