import numpy as np
from pymavlink import mavutil
import threading
import time
from pynput import keyboard
class DroneController():
    """
    Connect to the MAVLink device and print out:
      - Pitch, Roll, Yaw from ATTITUDE
      - GPS info (lat, lon, alt, # satellites) from GPS_RAW_INT
      - Local position (x, y, z, vx, vy, vz) in NED coordinates from LOCAL_POSITION_NED
    """
    def __init__(self, port, baud=115200):
        """
        Connect to autopilot/mavlink device
        Adjust for system port (e.g., '/dev/ttyUSB0' on Linux, 'udp:127.0.0.1:14550' for SITL, 'COM3' for Windows).
        Set the baudrate
        """
        self.master = mavutil.mavlink_connection(port, baud)
    
        # Wait for the first heartbeat to confirm connection
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(
            f"Heartbeat received (System: {self.master.target_system}, "
            f"Component: {self.master.target_component})"
        )
        self.database = {}
        self.offset_accel = 0 # Offset for IMU Accelometer
        self.offset_gyro = 0 # Offset for IMU Gyrometer
        self.offset_mag = 0 # Offset for IMU Magnitude
        self.rc_channels = [65535] * 8  # Default no override
        self.throttle = 1000  # Min throttle
        self.roll = 1500       # Neutral
        self.pitch = 1500      # Neutral
        self.yaw = 1500        # Neutral
        self.running = True

        # Request all data streams at 10 Hz.
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  
            10,  # Frequency (Hz)
            1
        )

    def get(self):
        """
        Get message frame from ArduPilot and set to database.
        """
        while True:
            msg = self.master.recv_match(blocking=True)
            if not msg:
                continue
            self.database[msg.get_type()] = msg.to_dict()
    
    def gps(self):
        try:
            gps=self.database['GPS_RAW_INT']
            # Extract GPS data
            # lat/lon = degrees * 1E7
            lat_deg = gps['lat'] / 1e7
            lon_deg = gps['lon'] / 1e7
            # altitude is in millimeters
            alt_m = gps['alt'] / 1000.0  

            # Delete previsous screen
            print("=== GPS ===")
            print(f"  Time (us): {gps['time_usec']}")
            print(f"  Fix type: {gps['fix_type']}")
            print(f"  Latitude:  {lat_deg:.6f} deg")
            print(f"  Longitude: {lon_deg:.6f} deg")
            print(f"  Altitude:  {alt_m:.2f} m")
            print(f"  Satellites visible: {gps['satellites_visible']}\n")
            
        except KeyError:
            print("GPS not found in data.")
    
    def imu_offset(self, num_samples=20, gyro_threshold=0.01):
            """
            Performs a reset of the sensors by averaging 20 samples where all Gyro values ​​are zero.
            :param drone: DroneMessage object used to collect sensor data
            :param num_samples: Number of samples to collect
            :param gyro_threshold: Threshold below which the gyro is considered to be "zero"
            """
            print("Search for 20 clean samples")
            
            sum_accel = np.zeros(3)
            sum_gyro = np.zeros(3)
            sum_mag = np.zeros(3)
            
            collected_samples = 0  # מונה דגימות תקינות

            while collected_samples < num_samples:
                accel, gyro, mag = self.imu()

                # בדיקה אם כל ערכי הג'יירו קרובים לאפס
                if all(abs(g) < gyro_threshold for g in gyro):
                    sum_accel += np.array(accel)
                    sum_gyro += np.array(gyro)
                    sum_mag += np.array(mag)
                    collected_samples += 1
                    print(f"Sample {collected_samples}/{num_samples} took")

                time.sleep(0.1)  # המתנה של 0.1 שניות בין הדגימות

            # חישוב ממוצע ערכי האיפוס
            self.offset_accel = sum_accel / num_samples
            self.offset_gyro = sum_gyro / num_samples
            self.offset_mag = sum_mag / num_samples

            print(f"IMU Sensors offset completed")
            print(f"  Accel Offset: {self.offset_accel}")
            print(f"  Gyro Offset:  {self.offset_gyro}")
            print(f"  Mag Offset:   {self.offset_mag}\n")

    def imu(self):
        """
        Reading IMU sensors and subtracting the calculated zero.
        """
        try:
            imu_data = self.database['RAW_IMU']
            raw_accel = np.array([imu_data['xacc'], imu_data['yacc'], imu_data['zacc']])
            raw_gyro = np.array([imu_data['xgyro'], imu_data['ygyro'], imu_data['zgyro']])
            raw_mag = np.array([imu_data['xmag'], imu_data['ymag'], imu_data['zmag']])

            # החסרת ערכי האיפוס שחושבו
            corrected_accel = raw_accel - self.offset_accel
            corrected_gyro = raw_gyro - self.offset_gyro
            corrected_mag = raw_mag - self.offset_mag

            return corrected_accel.tolist(), corrected_gyro.tolist(), corrected_mag.tolist()

        except KeyError:
            print("No IMU data.")
            return [0, 0, 0], [0, 0, 0], [0, 0, 0]

    
    def attitude(self):
        """
        Return Altitude of quadcopter - Roll, Pitch, Yaw
        """
        try:
            attitude=self.database['ATTITUDE']
            return attitude['roll'], attitude['pitch'], attitude['yaw']
        
        except KeyError:
            print("ATTITUDE not found in database.")
            return 0, 0, 0
        
    def set_mode(self, mode_name):
        """
        Sets the desired flight mode by name (e.g., 'GUIDED', 'AUTO') for ArduCopter.
        Uses the vehicle's mode mapping to find the custom_mode number.
        """
        # Check if mode is in the mode mapping
        if mode_name not in self.master.mode_mapping():
            print(f"Mode {mode_name} not found in mode mapping. Available modes: {list(self.master.mode_mapping().keys())}")
            return

        # Get mode ID from mode name
        mode_id = self.master.mode_mapping()[mode_name]
        
        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1 (bitmask)
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        print(f"Mode set to {mode_name} (id: {mode_id}).")

    def arm(self):
        """
        Arms the vehicle (motors). For ArduCopter, we can use built-in helper or
        send a MAV_CMD_COMPONENT_ARM_DISARM command.
        """
        # Send ARM to quadcopter
        self.master.mav.command_long_send(
                self.master.target_system,  # target_system
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
                0, # confirmation
                1, # param1 (1 to indicate arm)
                0, # param2 (all other params meaningless)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7
        print("Armed.")
    
    def disarm(self):
        '''disarm motors (arducopter only)'''
        self.master.mav.command_long_send(
            self.master.target_system,  # target_system
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
            0, # confirmation
            0, # param1 (0 to indicate disarm)
            0, # param2 (all other params meaningless)
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7
        
        # Wait and check if armed
        # Alternatively, you could read SYS_STATUS or HEARTBEAT to check arm status
        print("Disarmed.")
  
    def takeoff(self, altitude=10.0):
        """
        Commands the vehicle to take off to the specified altitude (meters).
        This typically requires the vehicle to be in GUIDED mode (and armed).
        """
        print(f"Initiating takeoff to {altitude} meters.")
        
        # MAV_CMD_NAV_TAKEOFF
        self.master.mav.command_long_send(
            self.master.target_system,                # target_system
            self.master.target_component,             # target_component
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
            0,                                   # confirmation
            0, 0, 0, 0, 0, 0,                    # unused parameters
            altitude                             # final param: altitude
        )
        print(f"Taking off to {altitude} meters.")

    def navController(self):
        try:
            self.t=+1
            nav=self.database['NAV_CONTROLLER_OUTPUT']
            axis = [nav['nav_roll'], nav['nav_pitch']]
            distance=[nav['nav_bearing'], nav['target_bearing'], nav['wp_dist']]
            error = [nav['alt_error'], nav['aspd_error'], nav['xtrack_error']]
                
            # Delete previsous screen 
            print(f"Step {self.t}: Roll={axis[0]:.2f}, Pitch={axis[1]:.2f}, "
                  f"Bearing={distance[0]:.2f}, target Bearing={distance[1]:.2f}, wp_dist={distance[2]:.2f} "
                  f"Errors={error[0]:.2f}, {error[1]:.2f}, {error[2]:.2f}")
                    
        except KeyError:
            print("Nav not found in database.")


    def set_servo(self, channel, pwm_value):
        """
        Sets a servo output to a specific PWM value using MAV_CMD_DO_SET_SERVO.
        
        :param master: MAVLink connection
        :param channel: Servo output channel (1-based index, e.g., 1 for SERVO1)
        :param pwm_value: PWM value (typically 1000-2000)
        """
        self.master.mav.command_long_send(
            self.master.target_system,  # Target system ID
            self.master.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Command ID
            0,  # Confirmation
            channel,  # Servo channel (1-based, e.g., 1 for SERVO1)
            pwm_value,  # PWM value (e.g., 1500 for neutral)
            0, 0, 0, 0, 0  # Unused parameters
        )
        print(f"Servo {channel} set to {pwm_value} PWM")

    def send_rc_override(self):
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            self.roll, self.pitch, self.throttle, self.yaw, 65535, 65535, 65535, 65535
        )
        print(f"Throttle: {self.throttle}, Roll: {self.roll}, Pitch: {self.pitch}, Yaw: {self.yaw}")
    
    def land(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0
        )
        print("Landing initiated.")

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.throttle = min(self.throttle + 50, 2000)
            elif key.char == 's':
                self.throttle = max(self.throttle - 50, 1000)
            elif key.char == 'a':
                self.yaw = max(self.yaw - 50, 1000)
            elif key.char == 'd':
                self.yaw = min(self.yaw + 50, 2000)
            elif key.char == 't':
                self.takeoff()
            elif key.char == 'l':
                self.land()
            elif key.char == 'f':
                self.arm()
            elif key.char == 'g':
                self.disarm()
        except AttributeError:
            if key == keyboard.Key.up:
                self.pitch = min(self.pitch + 50, 2000)
            elif key == keyboard.Key.down:
                self.pitch = max(self.pitch - 50, 1000)
            elif key == keyboard.Key.left:
                self.roll = max(self.roll - 50, 1000)
            elif key == keyboard.Key.right:
                self.roll = min(self.roll + 50, 2000)
            elif key == keyboard.Key.esc:
                print("Exiting...")
                self.running = False
                return False
        self.send_rc_override()

    def start_control(self):
        print("Keyboard Control Started. Use W/S (Throttle), A/D (Yaw), Arrow Keys (Pitch/Roll), T (Takeoff), L (Land), F (Arm), G (Disarm), Esc (Exit)")
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()


def main():
    drone = DroneController('COM11')  # עדכן לפורט הנכון
    thread = threading.Thread(target=drone.get, daemon=True)
    thread.start()
    time.sleep(3)
    
    # 1. Set flight mode to Stabilize
    drone.set_mode("STABILIZE")

    # 3. Flight
    control_thread = threading.Thread(target=drone.start_control)
    control_thread.start()
    control_thread.join()

    while True:
        try:
            print(drone.gps())
        except KeyboardInterrupt:
            print("\nExiting telemetry stream...")
            break


    
if __name__ == "__main__":
    main()
