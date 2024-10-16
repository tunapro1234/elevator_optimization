import numpy as np
import pandas as pd
import time


# Load motor performance data
motor_test_data = pd.read_csv('Datas/motor_test_data.csv')

# Load motor other data
motor_other = pd.read_csv('Datas/motor_other.csv')

# Load elevator parameters
elevator_params = pd.read_csv('Datas/elevator_params.csv')

# Extract parameters from motor_other.csv
Pmotor = motor_other['Pmotor'].iloc[0]
Rs_pred = motor_other['Rs_pred'].iloc[0]
max_a = motor_other['max_a'].iloc[0]
min_a = motor_other['min_a'].iloc[0]
max_v = motor_other['max_v'].iloc[0]
pulley_dia = motor_other['pulley_dia'].iloc[0] / 1000  # Convert mm to meters

# Extract elevator parameters
cage_mass = float(elevator_params.loc[elevator_params['Parameter'] == 'cage_mass', 'Value'].iloc[0])
carrying_capacity = float(elevator_params.loc[elevator_params['Parameter'] == 'carrying_capacity', 'Value'].iloc[0])
counterweight_mass = float(elevator_params.loc[elevator_params['Parameter'] == 'counterweight_mass', 'Value'].iloc[0])

# Initial parameters
initial_cabin_height = 0.0  # meters
target_height = 30.0  # meters
passenger_load = 500.0  # kg
g = 9.81  # m/s^2

# Total cabin mass
cabin_mass = cage_mass + passenger_load  # kg

# Initial positions
cabin_position = initial_cabin_height
counterweight_position = 50 - cabin_position

# Initial velocities and accelerations
cabin_velocity = 0.0
cabin_acceleration = 0.0

# Time step
dt = 1.0  # seconds

# PID controller parameters
Kp = 2.0
Ki = 0.015
Kd = 5.0

# Initialize PID variables
integral = 0.0
previous_error = 0.0

# Pulley and motor inertia
I_motor = 10.0  # kg·m²
m_pulley = 100.0  # kg (assumed)
pulley_radius = pulley_dia / 2
I_pulley = 0.5 * m_pulley * pulley_radius ** 2
I_total = I_motor + I_pulley

# Air resistance coefficient
c_air = 0.5  # kg/s (assumed)

# Cable stiffness
k_cable = 1e6  # N/m (assumed)


# Simulation parameters
time = 0.0
max_time = 50  # seconds

# Initialize energy variables
total_energy_consumed = 0.0
total_power_consumed = 0.0
if total_energy_consumed > 0.0:
    total_power_consumed = total_energy_consumed/(time*1000)
total_energy_generated = 0.0
total_power_generated = 0.0
if total_energy_generated > 0.0:
    total_power_generated = total_energy_generated/(time*1000)

while time < max_time:
    start_time = time.time()

    # PID controller to compute desired acceleration
    error = target_height - cabin_position
    integral += error * dt
    derivative = (error - previous_error) / dt
    previous_error = error

    # PID output is desired acceleration
    a_desired = Kp * error + Ki * integral + Kd * derivative

    # Limit desired acceleration
    if a_desired > max_a:
        a_desired = max_a
    elif a_desired < -max_a:
        a_desired = -max_a

    # Gravitational force difference
    F_gravity = (counterweight_mass - cabin_mass) * g

    # Air resistance
    F_air = c_air * cabin_velocity

    # Required net force to achieve desired acceleration
    F_required = cabin_mass * a_desired

    # Motor force needed
    F_motor = F_required - F_gravity - F_air

    # Motor torque
    motor_torque = F_motor * pulley_radius

    # Motor angular acceleration
    alpha = motor_torque / I_total

    # Update motor speed
    motor_speed = cabin_velocity / pulley_radius

    # Update cabin acceleration
    cabin_acceleration = a_desired

    # Update velocities and positions
    cabin_velocity += cabin_acceleration * dt
    cabin_position += cabin_velocity * dt

    # Update counterweight position
    counterweight_position = 50 - cabin_position

    # Energy calculations
    power = motor_torque * motor_speed
    if power >= 0:
        motor_state = 'HIGH'
        total_energy_consumed += power * dt
    else:
        motor_state = 'LOW'
        total_energy_generated += -power * dt  # Power generated is negative of consumed

    # Mechanical brake
    brake_force = 0.0
    if abs(cabin_acceleration) > max_a:
        # Apply brake to limit acceleration
        brake_force = (abs(cabin_acceleration) - max_a) * cabin_mass
        cabin_acceleration = np.sign(cabin_acceleration) * max_a

    # Print the state every second
    if int(time % 1) == 0:
        print(f"Time: {time:.1f}s")
        print(f"Cabin position: {cabin_position:.2f} m")
        print(f"Cabin speed: {cabin_velocity:.2f} m/s")
        print(f"Cabin acceleration: {cabin_acceleration:.2f} m/s²")
        print(f"Counterweight position: {counterweight_position:.2f} m")
        print(f"Motor torque: {motor_torque:.2f} Nm")
        print(f"Motor speed: {motor_speed:.2f} rad/s")
        print(f"Brake status: {'Engaged' if brake_force > 0 else 'Disengaged'}")
        print(f"motor_state: {motor_state}")
        print(f"Total energy consumed: {total_energy_consumed:.2f} J")
        print(f"Total power consumed: {total_power_consumed:.2f} W")
        print(f"Total energy generated: {total_energy_generated:.2f} J")
        print(f"Total power generated: {total_power_generated:.2f} W")
        print("----------------------------")

    # Check if target reached
    if abs(cabin_position - target_height) < 0.01 and abs(cabin_velocity) < 0.01:
        print("Target height reached.")
        break

    time += dt

    elapsed = start_time - time.time()
    while elapsed < 1:
        elapsed = start_time - time.time()

