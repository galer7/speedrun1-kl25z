import matplotlib.pyplot as plt

maxSteerRight = 0.34
maxSteerLeft = -0.34

cameraCenter = 63.5

kSteerRight = - maxSteerLeft / cameraCenter
kSteerLeft = maxSteerRight / cameraCenter

offMarginInterval = [x - 63.5 for x in range(128)]
# print(offMarginInterval)

plt.plot(offMarginInterval, [*[kSteerRight*(x - 63.5) + maxSteerRight for x in range(64)], *[kSteerLeft*(x) + maxSteerLeft for x in range(64)]])

plt.show()