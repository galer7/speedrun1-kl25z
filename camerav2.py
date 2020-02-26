from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from pyqtgraph.ptime import time
import serial

app = QtGui.QApplication([])

p = pg.plot()
p.setWindowTitle('live plot from serial')
curve = p.plot()

data = [0]

def update():
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

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)

if __name__ == '__main__':
  import sys
  if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
    QtGui.QApplication.instance().exec_()