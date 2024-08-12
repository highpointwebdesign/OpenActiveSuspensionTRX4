This code is designed for an ESP32-based active suspension system for an RC vehicle. It uses an MPU6050 sensor to measure accelerometer data and control four servos, representing the suspension on each corner of the vehicle (front-left, front-right, rear-left, rear-right). Here's a breakdown of its functionality:

**Key Components:**
MPU6050 Sensor: Measures acceleration along the X, Y, and Z axes.
ESP32 Wi-Fi Module: Creates an access point (AP) named 'ALVIN' with the password 'bluedaisy347'. It also hosts a web server to interact with the suspension settings.
Preferences Library: Stores and retrieves calibration data and suspension mode settings in non-volatile memory.
Servos: Control the suspension by adjusting their angles based on the vehicle's orientation.

**Main Features:**
1. Calibration and Preferences:

Calibration: The code reads initial accelerometer values for X, Y, and Z axes and uses these values to calibrate the sensor.
Preferences: Stores calibration offsets (X_CAL, Y_CAL, Z_CAL) and the suspension mode (suspensionMode) in non-volatile storage.

2. Suspension Modes:

  STABLE Mode: The suspension actively counteracts the vehicle's inertia to maintain stability. The servos adjust their angles more aggressively to compensate for tilt.
  REALISTIC Mode: Allows a more natural rocking motion, simulating a realistic suspension response with a less aggressive reaction.

3. Web Server API:

  API Endpoints:
    /api/load: Loads and returns the current settings as JSON.
    /api/save: Saves new settings received via POST requests.
    /api/failsafe: Resets the settings to their default/failsafe values.
    /api/reset_mpu: Resets the MPU6050 sensor.

4. Servo Control:

Reading Accelerometer Data: The code reads data from the MPU6050 sensor, subtracts the calibration offsets, and calculates the corrected acceleration values (acx, acy, acz).
Angle Calculation: Depending on the suspension mode, the servos' target angles are calculated based on the vehicle's tilt and acceleration data.
Angle Adjustment: The calculated angles are checked against thresholds and limits (SERVO_MIN, SERVO_MAX) before being sent to the servos to adjust their positions.

5. Interrupt Handling:

The code uses an interrupt to trigger reading accelerometer data whenever new data is available from the MPU6050 sensor.
