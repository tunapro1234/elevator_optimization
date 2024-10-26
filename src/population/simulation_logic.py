# simulation_logic.py

import pandas as pd
import random
import threading
import time
from datetime import datetime, timedelta
import json
import os
import re
import math

# Simulation parameters
SIMULATION_RUNNING = False
WAITING_AREAS_CAPACITY = 50  # Maximum number of people per floor
time_multiplier = 1  # Speed of the simulation
DOORS_OPEN = True  # Whether the elevator doors are open
settings_saved = False  # Indicates if settings have been saved

# Current simulation time and start time
current_time = datetime.now()
simulation_start_time = None

# Statistics
passenger_wait_times = []
passenger_journey_times = []
max_wait_time = 0
total_served_passengers = 0
wait_time_history = []
simulation_time_history = []

# Global structures
floors = {}
elevators = []

# Load settings from JSON file
def load_settings():
    if os.path.exists('settings.json'):
        with open('settings.json', 'r') as f:
            return json.load(f)
    else:
        # Default settings
        return {
            "FLOOR_COUNT": 10,
            "ELEVATOR_COUNT": 2,
            "ELEVATOR_CAPACITY_KG": 1000,
            "FLOOR_HEIGHTS": [3 for _ in range(9)],  # Default floor height is 3 meters
            "ELEVATOR_DOOR_WIDTH": 1.0,  # Default door width (meters)
            "ELEVATOR_WIDTH": 2.0,  # Default elevator width
            "ELEVATOR_DEPTH": 1.5   # Default elevator depth
        }

# Save settings to JSON file
def save_settings(settings):
    with open('settings.json', 'w') as f:
        json.dump(settings, f)

# Load data files (Ensure that the data files are in the 'datas' folder)
try:
    passenger_data = pd.read_csv('./data/avg_psg_trains.csv')
    commuting_data = pd.read_csv('./data/commuting_activity_by_age.csv')
    avg_weight_female = pd.read_csv('./data/avgWeightFemale.csv', delimiter=';')
    avg_weight_male = pd.read_csv('./data/avgWeightMale.csv', delimiter=';')
except FileNotFoundError:
    print("Error: Required data files not found in 'data' folder.")
    exit()

# Load initial settings
settings = load_settings()
FLOOR_COUNT = settings.get('FLOOR_COUNT', 10)
ELEVATOR_COUNT = settings.get('ELEVATOR_COUNT', 2)
ELEVATOR_CAPACITY_KG = settings.get('ELEVATOR_CAPACITY_KG', 1000)
FLOOR_HEIGHTS = settings.get('FLOOR_HEIGHTS', [3 for _ in range(FLOOR_COUNT - 1)])
ELEVATOR_DOOR_WIDTH = settings.get('ELEVATOR_DOOR_WIDTH', 1.0)
ELEVATOR_WIDTH = settings.get('ELEVATOR_WIDTH', 2.0)
ELEVATOR_DEPTH = settings.get('ELEVATOR_DEPTH', 1.5)

# Adjust FLOOR_HEIGHTS if necessary
if len(FLOOR_HEIGHTS) != FLOOR_COUNT - 1:
    FLOOR_HEIGHTS = [3 for _ in range(FLOOR_COUNT - 1)]

# Passenger class representing each passenger in the simulation
class Passenger:
    group_id_counter = 0  # Class variable to assign unique group IDs

    def __init__(self, arrival_time):
        self.gender = self.assign_gender()
        self.age_group = self.assign_age_group(arrival_time)
        self.weight = self.assign_weight()
        self.group_size = self.assign_group()
        self.arrival_time = arrival_time
        self.start_time = arrival_time
        self.current_floor = random.randint(1, FLOOR_COUNT)
        self.destination_floor = random.randint(1, FLOOR_COUNT)
        while self.destination_floor == self.current_floor:
            self.destination_floor = random.randint(1, FLOOR_COUNT)
        self.uses_elevator = self.decide_transport_mode()
        self.wait_time = 0
        self.journey_time = 0  # Initially zero
        self.passenger_type = self.assign_passenger_type()
        self.area = self.calculate_area()
        self.is_disabled = self.assign_disability()
        if self.is_disabled:
            self.area *= 1.5  # Disabled passengers occupy 150% more area
            self.weight += 20  # Additional weight for wheelchairs or aids
        if self.group_size > 1:
            self.group_id = Passenger.group_id_counter
            Passenger.group_id_counter += 1
        else:
            self.group_id = None

    # Assign group size (5% chance of being in a group)
    def assign_group(self):
        if random.random() <= 0.05:
            return random.randint(2, 5)
        else:
            return 1

    # Assign age group based on time of day
    def assign_age_group(self, arrival_time):
        hour = arrival_time.hour
        if 6 <= hour < 9:
            period = 'Morning (6 AM - 9 AM)'
        elif 11 <= hour < 14:
            period = 'Midday (11 AM - 2 PM)'
        elif 16 <= hour < 19:
            period = 'Evening (4 PM - 7 PM)'
        else:
            period = None

        if period:
            probabilities = commuting_data.get(period, [])
            age_groups = commuting_data.get('Age Group', [])
            if len(probabilities) != len(age_groups):
                return random.choice(['15-24', '25-54', '55+'])
            return random.choices(age_groups, weights=probabilities)[0]
        else:
            return random.choice(['15-24', '25-54', '55+'])

    # Assign passenger type based on age group
    def assign_passenger_type(self):
        age_group = self.age_group
        match = re.search(r'\d+', age_group)
        if match:
            age = int(match.group())
        else:
            age = 65  # Default age if not found

        if age < 25:
            return 'young'
        elif age < 55:
            return 'adult'
        else:
            return 'elderly'

    # Assign gender randomly
    def assign_gender(self):
        return random.choice(['Erkek', 'Kadın'])  # 50% chance for each

    # Assign weight based on gender and age group
    def assign_weight(self):
        if self.gender == 'Kadın':
            df = avg_weight_female
        else:
            df = avg_weight_male

        weight_row = df[df['Age Groups'] == self.age_group]
        if weight_row.empty:
            mean_weight = 70  # Default value
            std_dev = 10
        else:
            mean_weight = weight_row['Mean'].values[0]
            std_dev = weight_row['Standard Error of the Mean'].values[0] * 10

        weight = random.normalvariate(mean_weight, std_dev)
        return max(40, min(weight, 150))  # Keep weight within reasonable bounds

    # Decide whether the passenger uses the elevator or stairs
    def decide_transport_mode(self):
        floor_diff = abs(self.destination_floor - self.current_floor)
        if self.destination_floor > self.current_floor:
            max_walk_floors = 4.7
            stairs_pref = 0.842
        else:
            max_walk_floors = 6.7
            stairs_pref = 0.878

        uses_stairs = floor_diff <= max_walk_floors and random.random() < stairs_pref

        # Crowding factor
        waiting_passengers = floors.get(self.current_floor, {'waiting': []})['waiting']
        crowding_factor = len(waiting_passengers) / WAITING_AREAS_CAPACITY
        if crowding_factor >= 0.7:
            uses_stairs_chance = 0.5
            uses_stairs = uses_stairs or random.random() < uses_stairs_chance

        # Group behavior
        if self.group_size > 1:
            uses_stairs = False

        return not uses_stairs

    # Calculate area occupied by the passenger
    def calculate_area(self):
        return 0.2  # Average area in square meters

    # Assign disability status (1% chance)
    def assign_disability(self):
        return random.random() < 0.01

# Elevator class representing each elevator in the simulation
class Elevator:
    def __init__(self, elevator_id):
        self.elevator_id = elevator_id
        self.capacity = ELEVATOR_CAPACITY_KG
        self.current_load = 0
        self.current_floor = 1
        self.current_height = self.calculate_current_height()
        self.passengers = []
        self.direction = 'idle'  # 'up', 'down', or 'idle'
        self.status = 'idle'
        self.manual_targets = []  # User-defined target floors
        self.moving = False  # Elevator movement status
        self.speed = 1  # Elevator speed in m/s
        self.lock = threading.Lock()  # To manage concurrent access
        self.width = ELEVATOR_WIDTH
        self.depth = ELEVATOR_DEPTH
        self.total_area = self.width * self.depth  # Total area of the elevator
        self.current_area = 0  # Area occupied by current passengers

    # Calculate the current height of the elevator based on floor heights
    def calculate_current_height(self):
        return sum(FLOOR_HEIGHTS[:self.current_floor - 1])

    # Move the elevator to the next target floor
    def move(self):
        with self.lock:
            if not self.manual_targets:
                self.direction = 'idle'
                self.moving = False
                return

            if self.moving:
                # Elevator is already moving
                return

            # Prepare to move to the target floor
            target_floor = self.manual_targets.pop(0)
            self.moving = True
            self.direction = 'up' if target_floor > self.current_floor else 'down'

        # Simulate elevator movement
        distance = abs(target_floor - self.current_floor)
        move_duration = distance * 2  # For example, 2 seconds per floor
        time.sleep(move_duration / time_multiplier)

        with self.lock:
            self.current_floor = target_floor
            self.current_height = self.calculate_floor_height(target_floor)

        # Open doors and manage passengers
        if DOORS_OPEN:
            self.unload_passengers()
            self.load_passengers()

        with self.lock:
            self.direction = 'idle'
            self.moving = False

    # Calculate distance between two floors
    def get_distance(self, start_floor, end_floor):
        if start_floor == end_floor:
            return 0
        elif start_floor < end_floor:
            return sum(FLOOR_HEIGHTS[start_floor - 1:end_floor - 1])
        else:
            return sum(FLOOR_HEIGHTS[end_floor - 1:start_floor - 1])

    # Calculate the height of a given floor
    def calculate_floor_height(self, floor):
        return sum(FLOOR_HEIGHTS[:floor - 1])

    # Calculate boarding time for passengers
    def calculate_boarding_time(self, passengers, crowded=False):
        total_time = 0
        door_width_factor = 0.8 if ELEVATOR_DOOR_WIDTH >= 1.2 else 1.0

        # Crowding conditions
        crowding_factor = 1.0
        if crowded:
            crowding_factor += random.uniform(0.2, 0.5)

        for passenger in passengers:
            if passenger.passenger_type == 'young':
                base_time = 1.2
            elif passenger.passenger_type == 'adult':
                base_time = 1.5
            else:
                base_time = 2.0

            # Increase time for disabled passengers
            if passenger.is_disabled:
                base_time *= 1.5  # 50% more time for disabled passengers

            # Apply random variation
            base_time *= random.uniform(0.9, 1.1)
            # Apply door width and crowding factors
            boarding_time = base_time * door_width_factor * crowding_factor
            total_time += boarding_time * passenger.group_size

        return total_time

    # Load passengers into the elevator
    def load_passengers(self):
        if not DOORS_OPEN:
            return
        with self.lock:
            waiting_passengers = floors[self.current_floor]['waiting']
            boarding_passengers = []
            i = 0
            while i < len(waiting_passengers):
                passenger = waiting_passengers[i]
                if passenger.group_id is not None:
                    # Handle group boarding
                    group_members = [p for p in waiting_passengers if p.group_id == passenger.group_id]
                    total_group_weight = sum(p.weight * p.group_size for p in group_members)
                    total_group_area = sum(p.area * p.group_size for p in group_members)
                    total_weight = self.current_load + total_group_weight
                    total_area = self.current_area + total_group_area
                    if total_weight <= self.capacity and total_area <= self.total_area:
                        self.passengers.extend(group_members)
                        boarding_passengers.extend(group_members)
                        self.current_load = total_weight
                        self.current_area = total_area
                        for member in group_members:
                            waiting_passengers.remove(member)
                    else:
                        i += 1
                else:
                    # Handle individual passenger boarding
                    total_weight = self.current_load + (passenger.weight * passenger.group_size)
                    total_area = self.current_area + (passenger.area * passenger.group_size)
                    if total_weight <= self.capacity and total_area <= self.total_area:
                        self.passengers.append(passenger)
                        boarding_passengers.append(passenger)
                        self.current_load = total_weight
                        self.current_area = total_area
                        waiting_passengers.pop(i)
                    else:
                        i += 1
                if self.current_load >= self.capacity or self.current_area >= self.total_area:
                    break

        # Calculate boarding time and wait
        if boarding_passengers:
            crowded = len(self.passengers) > self.capacity * 0.8 or self.current_area > self.total_area * 0.8
            boarding_time = self.calculate_boarding_time(boarding_passengers, crowded=crowded)
            time.sleep(boarding_time / time_multiplier)

    # Unload passengers from the elevator
    def unload_passengers(self):
        global total_served_passengers, passenger_journey_times
        if not DOORS_OPEN:
            return
        with self.lock:
            disembarking = [p for p in self.passengers if p.destination_floor == self.current_floor]
            if not disembarking:
                return
            # Calculate alighting time and wait
            crowded = len(disembarking) > self.capacity * 0.8 or self.current_area > self.total_area * 0.8
            alighting_time = self.calculate_boarding_time(disembarking, crowded=crowded)

        time.sleep(alighting_time / time_multiplier)

        with self.lock:
            for passenger in disembarking:
                self.passengers.remove(passenger)
                self.current_load -= passenger.weight * passenger.group_size
                self.current_area -= passenger.area * passenger.group_size
                total_served_passengers += passenger.group_size
                # Record journey time
                passenger.journey_time = (current_time - passenger.start_time).total_seconds()
                passenger_journey_times.append(passenger.journey_time)

    # Set manual target floor (used by GUI controls)
    def set_manual_target(self, target_floor):
        with self.lock:
            if target_floor not in self.manual_targets:
                self.manual_targets.append(target_floor)
        # Start moving the elevator
        threading.Thread(target=self.move, daemon=True).start()

# Initialize simulation (called when settings are updated)
def initialize_simulation():
    global floors, elevators
    # Initialize floors
    floors = {i: {'waiting': [], 'stairs': []} for i in range(1, FLOOR_COUNT + 1)}
    # Initialize elevators
    elevators.clear()
    for i in range(1, ELEVATOR_COUNT + 1):
        elevators.append(Elevator(elevator_id=i))

# Generate passengers based on the current time
def generate_passengers(current_time):
    hour = current_time.hour
    passengers_to_add = get_passenger_count_for_hour(hour)
    for _ in range(passengers_to_add):
        passenger = Passenger(current_time)
        if passenger.uses_elevator:
            floor_queue = floors[passenger.current_floor]['waiting']
            floor_queue.append(passenger)
        else:
            floors[passenger.current_floor]['stairs'].append(passenger)

# Get the number of passengers for a given hour
def get_passenger_count_for_hour(hour):
    data_row = passenger_data.loc[passenger_data['transition_hour'] == hour]
    if not data_row.empty:
        passengers_per_hour = data_row.iloc[0]['number_of_passenger']
        return int(passengers_per_hour / 3600 * time_multiplier)
    else:
        return 0

# Run the simulation (to be called in a separate thread)
def run_simulation():
    global current_time, SIMULATION_RUNNING
    while SIMULATION_RUNNING:
        generate_passengers(current_time)
        # Update elevator movements
        for elevator in elevators:
            threading.Thread(target=elevator.move, daemon=True).start()
        time.sleep(1 / time_multiplier)
        current_time += timedelta(seconds=1 * time_multiplier)

# Update settings based on user input
def update_settings(new_settings):
    global FLOOR_COUNT, ELEVATOR_COUNT, ELEVATOR_CAPACITY_KG, SIMULATION_RUNNING, FLOOR_HEIGHTS
    global ELEVATOR_DOOR_WIDTH, ELEVATOR_WIDTH, ELEVATOR_DEPTH, settings_saved
    try:
        new_floor_count = int(new_settings['floor_count'])
        new_elevator_count = int(new_settings['elevator_count'])
        new_elevator_capacity = int(new_settings['elevator_capacity'])
        new_elevator_door_width = float(new_settings['elevator_door_width'])
        new_elevator_width = float(new_settings['elevator_width'])
        new_elevator_depth = float(new_settings['elevator_depth'])
        new_floor_heights = [float(h) for h in new_settings['floor_heights']]

        if new_floor_count <= 0 or new_elevator_count <= 0 or new_elevator_capacity <= 0 or \
           new_elevator_door_width <= 0 or new_elevator_width <= 0 or new_elevator_depth <= 0:
            raise ValueError

        # Adjust floor heights if floor count changed
        if len(new_floor_heights) != new_floor_count - 1:
            new_floor_heights = [3 for _ in range(new_floor_count - 1)]

        # Update settings
        settings = {
            "FLOOR_COUNT": new_floor_count,
            "ELEVATOR_COUNT": new_elevator_count,
            "ELEVATOR_CAPACITY_KG": new_elevator_capacity,
            "FLOOR_HEIGHTS": new_floor_heights,
            "ELEVATOR_DOOR_WIDTH": new_elevator_door_width,
            "ELEVATOR_WIDTH": new_elevator_width,
            "ELEVATOR_DEPTH": new_elevator_depth
        }
        save_settings(settings)

        # Update global variables
        FLOOR_COUNT = new_floor_count
        ELEVATOR_COUNT = new_elevator_count
        ELEVATOR_CAPACITY_KG = new_elevator_capacity
        FLOOR_HEIGHTS = new_floor_heights
        ELEVATOR_DOOR_WIDTH = new_elevator_door_width
        ELEVATOR_WIDTH = new_elevator_width
        ELEVATOR_DEPTH = new_elevator_depth

        # Restart simulation
        SIMULATION_RUNNING = False
        time.sleep(1)  # Wait for simulation to stop

        initialize_simulation()

        settings_saved = True
        return True, "Settings updated successfully."

    except ValueError:
        return False, "Invalid input. Please enter valid positive numbers."
