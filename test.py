import numpy as np
from kalman_filter import KalmanFilter
import matplotlib.pyplot as plt
from sensors import PositonSensor

kf = KalmanFilter(nx=6, motion_model='ca')

n_simulation = 100

xk_records = []

# Etiket kontrolü için bayraklar
is_predicted_plotted = False
is_updated_plotted = False

sensor = PositonSensor(dimension=2) #x, y

for i in range(n_simulation):
    kf.predict()

    xkkm1 = kf.xk_km1

    zk = sensor.get()

    kf.update(zk=zk, sensor_std=1.0)

    xkk = kf.xk_k

    # Legend'de sadece bir kez etiket eklemek için kontrol mekanizması
    if not is_predicted_plotted:
        plt.scatter(xkkm1[0][0], xkkm1[1][0], c='blue', label='Predicted')
        plt.scatter(xkk[0][0], xkk[1][0], c='red', label='Updated')
        plt.scatter(zk[0][0], zk[1][0], c='green', label='Sensor')
        is_predicted_plotted = True

    else:
        plt.scatter(xkkm1[0][0], xkkm1[1][0], c='blue')
        plt.scatter(xkk[0][0], xkk[1][0], c='red')
        plt.scatter(zk[0][0], zk[1][0], c='green')


plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.legend()
plt.show()
