import matplotlib.pyplot as plt

maxSteerRight = 0.34
maxSteerLeft = -0.34

cameraCenter = 63.5

kSteerRight = -maxSteerRight / cameraCenter
kSteerLeft = maxSteerLeft / cameraCenter

offMarginInterval = [x - 63.5 for x in range(128)]
steer = []
# print(offMarginInterval)
for x in offMarginInterval:
  if x <= 0:
    # steer left 
    steer.append(kSteerLeft*x + maxSteerLeft)

  else:
    steer.append(kSteerRight*x + maxSteerRight)

plt.plot(offMarginInterval, steer)
plt.xlabel('offmarginInterval (63.5 - marginPosition)')
plt.ylabel('steer')
plt.text(20, 0.2, 'steer > 0: viraj dreapta')
plt.text(-60, 0.2, 'steer < 0: viraj stanga')
plt.show()