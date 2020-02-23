import pyqtgraph as pg


xAxis = [x for x in range(128)]
cameraLine = [0 for x in range(128)]

with serial.Serial('COM3', 115200) as ser:
  while(True):

    # ser.flushInput()
    # ser.flushOutput()
    line = ser.readline()
    try:
      # print(line.decode('utf-8'))
      cameraLine = [x for x in line.decode('utf-8').split(',')]

      cameraLine[0] = cameraLine[0][1:]
      cameraLine[-1] = cameraLine[-1][:-2]
      cameraLine = [int(x) for x in cameraLine]
      # print(cameraLine)
      pg.plot(xAxis, cameraLine)   # data can be a list of values or a numpy array

    except UnicodeDecodeError:
      pass
    except ValueError:
      pass
    except KeyboardInterrupt:
      pass