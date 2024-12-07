from copy import deepcopy

import numpy as np


class KalmanFilter:
    def __init__(self, xk_init = None,
                 nx = 2, #default: (x,y)
                 motion_model='cv' #default: constant velocity
                 ):

        if xk_init == None:
            self.xk_k = np.ones(shape=(nx, 1))
            self.xk_km1 = deepcopy(self.xk_k)

        self.Pk_k = np.ones(shape=(nx, nx))
        self.Pk_km1 = deepcopy(self.Pk_k)
        """
        nx
        2: x, y
        4: x, y, Vx, Vy
        6: x, y, Vx, Vy, ax, ay 
        """
        self.nx = nx # state dimension
        self.motion_model = motion_model


    def predict(self, dt = 1.0):

        nx = self.nx

        xk_k = self.xk_k

        Pk_k = self.Pk_k

        Fk = np.eye(nx) #dynamic transition matrix

        Qk = np.zeros(shape=(nx, nx))

        if self.motion_model.lower() == 'cv':
            if nx == 2: #x, y
                Qk[0][0] = Qk[1][1] = np.power(dt, 4)/4

            elif nx == 4: #x, y, Vx, Vy
                Qk[0][0] = Qk[1][1] = np.power(dt, 4) / 4
                Qk[0][2] = Qk[2][0] = np.power(dt, 3) / 2
                Qk[2][2] = Qk[3][3] = dt ** 2
                Fk[0][2] = Fk[1][3] = dt

            else:
                raise  ValueError('Undefined dimension.')

        elif self.motion_model == 'ca':
            if self.nx == 2: #x, y
                Qk[0][0] = Qk[1][1] = np.power(dt, 5) / 20
            elif self.nx == 4: # x, y, Vx, Vy
                Qk[0][0] = Qk[1][1] = np.power(dt, 5) / 20
                Qk[2][0] = Qk[0][2] = np.power(dt, 4) / 8
                Qk[2][2] = np.power(dt, 3) / 3
                Qk[3][1] = Qk[1][3] = np.power(dt, 4) / 8
                Qk[3][3] = np.power(dt, 3) / 3
                Qk[4][4] = Qk[5][5] = dt
                Qk[0][4] = Qk[4][0] = np.power(dt, 3) / 6
                Qk[2][4] = Qk[4][2] = np.power(dt, 2) / 2
                Fk[0][2] = Fk[1][3] = dt

            elif self.nx == 6: # x,y ,Vx, Vy, ax, ay
                Qk[0][0] = Qk[1][1] = np.power(dt, 5) / 20
                Qk[2][0] = Qk[0][2] = np.power(dt, 4) / 8
                Qk[2][2] = np.power(dt, 3) / 3
                Qk[3][1] = Qk[1][3] = np.power(dt, 4) / 8
                Qk[3][3] = np.power(dt, 3) / 3
                Qk[4][4] = Qk[5][5] = dt
                Qk[0][4] = Qk[4][0] = np.power(dt, 3) / 6
                Qk[2][4] = Qk[4][2] = np.power(dt, 2) / 2
                Qk[5][1] = Qk[5][1] = np.power(dt, 3) / 6
                Qk[5][3] = Qk[5][3] = np.power(dt, 2) / 2

                Fk[0][2] = Fk[1][3] = dt
                Fk[0][4] = Fk[1][5] = 0.5 * dt ** 2 #position - acceleration
                Fk[2][4] = Fk[3][5] = dt #velocity - acceleration

        else:
            raise ValueError("Undeclared motion model!")

        # State prediction
        self.xk_km1 = Fk @ xk_k

        # Uncertainity prediction
        self.Pk_km1 = Fk @ Pk_k @ Fk.T + Qk


    def update(self, zk, sensor_model='position', Rk=None, sensor_std = 0.1):
        """
        :param zk:
        :param sensor_model: position(gps), range(uwb), angle(doppler)
        :return:
        """

        Pk_km1 = self.Pk_km1 #predicted covariance matrix
        xk_km1 = self.xk_km1 #predicted state vector

        nx = self.nx
        nz = zk.shape[0] #measurement dimension
        # calculate measurement matrix
        Hk = np.zeros(shape=(nz, nx))

        if sensor_model == 'position':
            Hk[0][0] = Hk[1][1] = 1.0
            if Rk == None:
                Rk = np.eye(nz)
                Rk[0][0] = Rk[1][1] = sensor_std

        # calculate innovation covariance
        Sk = Hk @ Pk_km1 @ Hk.T + Rk

        #calculate Kalman gain
        Kk = Pk_km1 @ Hk.T @ np.linalg.pinv(Sk) #pseudo-inverse: to avoid numerical error

        # update state vector
        self.xk_k = xk_km1 + Kk @ (zk - Hk @ xk_km1)

        #update covariance matrix
        self.Pk_k = (np.eye(nx) - Kk @ Hk) @ Pk_km1

