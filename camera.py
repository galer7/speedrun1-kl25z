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

      cameraLine = [int(x) for x in line.decode('utf-8').split(',')]
      print(cameraLine)
      avgLine = [sum(cameraLine) / 128 for x in range(128)]
      # print(cameraLine)
      plt.clf()
      plt.axis([0, 127, 0, 5000])

      plt.plot(xAxis, cameraLine, avgLine)
      plt.pause(0.00000001)
    except UnicodeDecodeError:
      pass
    except ValueError:
      pass
    except KeyboardInterrupt:
      pass
  plt.show()