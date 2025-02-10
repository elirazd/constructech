import numpy as np
from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt
from pymavlink import mavutil
import threading
import time
import os
import math
from start import DroneController
class DroneINSKalmanFilter:
    """
    Kalman Filter for INS (Inertial Navigation System) using Gyro, Magnetometer, Real Yaw, and Velocity IMU.
    This filter estimates position (X, Y), velocity (Vx, Vy), and yaw angle.
    """
    def __init__(self):
        self.kf = KalmanFilter(dim_x=8, dim_z=5)  # 8 משתנים (Yaw, X, Y, Vx, Vy)
        dt = 0.1  # קצב דגימה 10Hz

        # מטריצת מעבר (F) - כוללת אינטגרציה של זווית ומהירות
        self.kf.F = np.array([[1, dt, 0,  0,  0,  0,  0,  0],  
                              [0,  1,  0,  0,  0,  0,  0,  0],  
                              [0,  0,  1, dt,  0,  0,  0,  0],  
                              [0,  0,  0,  1,  0,  0, dt,  0],  
                              [0,  0,  0,  0,  1, dt,  0,  0],  
                              [0,  0,  0,  0,  0,  1,  0, dt],  
                              [0,  0,  0,  0,  0,  0,  1,  0],  
                              [0,  0,  0,  0,  0,  0,  0,  1]])

        # מטריצת מדידה (H) - משייכת Yaw, מיקום ומהירות
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0],  # Yaw אמיתי
                              [0, 0, 1, 0, 0, 0, 0, 0],  # X position
                              [0, 0, 0, 0, 1, 0, 0, 0],  # Y position
                              [0, 0, 0, 0, 0, 0, 1, 0],  # Vx (Velocity IMU)
                              [0, 0, 0, 0, 0, 0, 0, 1]]) # Vy (Velocity IMU)

        # רעש תהליך (Q)
        self.kf.Q = np.eye(8) * 0.001

        # רעש מדידה (R) - מאמינים יותר ל-Yaw האמיתי ולמהירות IMU
        self.kf.R = np.array([[0.01, 0, 0, 0, 0],  # Yaw real - נמוך
                              [0, 0.1, 0, 0, 0],  # X position
                              [0, 0, 0.1, 0, 0],  # Y position
                              [0, 0, 0, 0.05, 0],  # Velocity X
                              [0, 0, 0, 0, 0.05]]) # Velocity Y

        self.kf.P *= 1000  # חוסר ודאות ראשוני
        self.kf.x = np.zeros((8, 1))  # מצב ראשוני

    def predict(self):
        """ Predict the next state using the Kalman filter. """
        self.kf.predict()

    def update(self, measurement):
        """ Update the Kalman filter with new measurements. """
        self.kf.update(measurement)

    def get_estimated_position(self):
        """ Get estimated yaw, position, and velocity. """
        return self.kf.x[0, 0], self.kf.x[2, 0], self.kf.x[4, 0], self.kf.x[6, 0], self.kf.x[7, 0]  # Yaw, X, Y, Vx, Vy




def main():
    """
    Run the INS Kalman filter with live drone sensor data.
    """
    kalman = DroneINSKalmanFilter()
    drone = DroneController('COM11')  # עדכן לפורט הנכון
    thread = threading.Thread(target=drone.get, daemon=True)
    thread.start()
    time.sleep(3)
    drone.imu_offset()

    num_steps = 100  # מספר איטרציות
    dt = 0.1  # זמן מחזור (10Hz)

    print("Starting inertial navigation...\n")

    for t in range(num_steps):
        # נתוני Attitude (Yaw אמיתי)
        roll, pitch, yaw_real = drone.attitude()

        # נתוני IMU (Gyro, Magnetometer, Velocity)
        imu_acceleration, gyro, magnitude = drone.imu()

        yaw_gyro = gyro[2]  # קצב שינוי בזווית
        yaw_mag = magnitude[2]  # מדידת מגנטומטר

        # חישוב שינויי מיקום מבוסס Yaw וזמן
        yaw_rad = np.radians(yaw_real)  # משתמשים ב-Yaw האמיתי
        vx_imu = imu_acceleration[0] * dt  # אינטגרציה למהירות
        vy_imu = imu_acceleration[1] * dt

        dx = vx_imu * np.cos(yaw_rad) - vy_imu * np.sin(yaw_rad)
        dy = vx_imu * np.sin(yaw_rad) + vy_imu * np.cos(yaw_rad)

        # עדכון Kalman Filter
        kalman.predict()
        measurement = np.array([[float(yaw_real)],  # ✅ שימוש ב-Yaw האמיתי
                                [float(dx)],       # X position estimate
                                [float(dy)],       # Y position estimate
                                [float(vx_imu)],   # Velocity X
                                [float(vy_imu)]], dtype=np.float32)  # Velocity Y
        print(measurement)
        kalman.update(measurement)

        # קבלת מיקום מוערך
        yaw, x, y, vx, vy = kalman.get_estimated_position()
        print(f"Step {t}: X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f}, Vx={vx:.2f}, Vy={vy:.2f}")

        time.sleep(dt)

if __name__ == "__main__":
    main()