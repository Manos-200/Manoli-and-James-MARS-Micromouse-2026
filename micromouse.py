"""
Filename: micromouse.py
Author: Quinn Horton, UQ Mechatronics and Robotics Society
Date: 03/01/2025
Version: 0.5
Description: Provides a software abstraction for the Micromouse robot.
License: MIT License
"""
from machine import Pin, Timer, ADC
from motor import Motor
from ucollections import deque
import time
from encoders import Encoders
encoders = Encoders()


class Micromouse():
    """
    Represents the physical Micromouse device in code.
    Implemented as a singleton class, as the code should only know of the
    device it is running on.
    """
    def __new__(cls):
        """
        Creates instances of the class. Used here to ensure the object is a
        singleton.

        Parameters:
            cls (type): The class itself.

        Returns:
            Micromouse: The single instance of the class.
        """
        if not hasattr(cls, 'instance'):
            cls.instance = super(Micromouse, cls).__new__(cls)
        return cls.instance

    def __init__(self, size=9):
        """
        Initialises the member variables upon first creation.
        """
        if hasattr(self, 'exists'):
            return
        self.exists = True

        # Inputs
        self.button = Pin(11, Pin.IN)
        self.ir_1 = Pin(12, Pin.IN)
        self.ir_2 = Pin(13, Pin.IN)
        self.ir_3 = Pin(14, Pin.IN)
        self.ldr_green = ADC(Pin(26, Pin.IN)) # A0
        self.ldr_red = ADC(Pin(27, Pin.IN)) # A1


        # Outputs
        self.green_led = Pin(10, Pin.OUT)
        self.red_led = Pin(9, Pin.OUT)
        self.debug_led = Pin(25, Pin.OUT)
        self.motor_2 = Motor(17, 18, 15, 16)
        self.motor_1 = Motor(21, 20, 19, 22)
        self.motor_2.invert_motor()
        self.motor_1.invert_motor()

        # Other
        self.blink_timer = Timer()
        self.pos = [0,0]
        self.dir = [1,0]
        self.maze = [[0 for _ in range(size*2-1)] for _ in range(size*2-1)]

    def led_set(self, red_val, green_val):
        """
        Set both red and green LEDs to provided values.

        Parameters:
            red_val (bool): The desired state of the red led.
            green_val (bool): The desired state of the green led.
        """
        self.green_led.value(green_val)
        self.red_led.value(red_val)

    def led_green_set(self, value):
        """
        Set the green LED to the provided value.

        Parameters:
            value (bool): The desired state of the green led.
        """
        self.green_led.value(value)

    def led_red_set(self, value):
        """
        Set the red LED to the provided value.

        Parameters:
            value (bool): The desired state of the red led.
        """
        self.red_led.value(value)

    def led_debug_set(self, value):
        """
        Set the debug LED to the provided value.

        Parameters:
            value (bool): The desired state of the debug led.
        """
        self.debug_led.value(value)

    def led_toggle(self):
        """
        Toggle both red and green LEDs when called.
        """
        self.green_led.toggle()
        self.red_led.toggle()

    def led_toggle_start(self, frequency=1):
        """
        Initialise an LED blinking timer for the red and green LEDs.

        Parameters:
            frequency (int, optional): The frequency at which the LEDs should
                blink.
        """
        self.blink_timer.init(mode=Timer.PERIODIC, freq=frequency,
                              callback=lambda t: self.led_toggle())

    def led_toggle_stop(self):
        """
        Stop the blinking of the onboard red and green LEDs and turn them off.
        """
        self.blink_timer.deinit()
        self.red_led.off()
        self.green_led.off()

    def get_ir_values(self, index=0):
        """
        Gets the current values of the infrared object detector sensors.

        Parameters:
            index (int): The number of the IR sensor to read. 1 for IR1
            and 3 for IR3

        Returns:
            Union[(bool, bool, bool), bool]:
                The IR sensor readings in the order 1, 2, 3 if no
                index is provided, otherwise the result of the specified
                sensor. True indicates an object detected.
        """
        if index >= 4:
            raise IndexError("IR Sensor index should not exceed 2.")
        sensor_1 = self.ir_1.value() == 0
        sensor_2 = self.ir_2.value() == 0
        sensor_3 = self.ir_3.value() == 0
        if index == 1:
            return sensor_1
        elif index == 2:
            return sensor_2
        elif index == 3:
            return sensor_3
        elif index < 1:
            return (sensor_1, sensor_2, sensor_3)
        
    def get_ldr_values(self):
        """
        Gets the current value of the ldr sensors. Also adjusts the Pi's onboard red/green LEDs accordingly for visual feedback.
        
        Parameters:
            green_range (int, int): Values of the ldr that should be considered green.
            red_range (int, int): Values of the ldr that should be considered green.

        Returns:
            (int, int):
            Returns a tuple of integers from 0-65535, with higher values indicating more light detected.
            The green ldr is returned at index zero. The red ldr is returned at index one.
        """
        
        green_value = self.ldr_green.read_u16()
        red_value = self.ldr_red.read_u16()    

        return (green_value, red_value)
    
    def check_colour(self, green_range, red_range, red_cost=5, green_cost=-1):
        
        green_count = 0
        red_count = 0
        for _ in range(5):
            green_state = min(green_range) < self.get_ldr_values()[0] < max(green_range)
            red_state = min(red_range) < self.get_ldr_values()[1] < max(red_range)
            if green_state and red_state:
                green_state, red_state = False, False
            green_count += green_state
            red_count += red_state
        colour = (red_count > 3) * red_cost + (green_count > 3) * green_cost
        
        return colour
    
    def record_colour(self, colour):
        
        self.maze[2*self.pos[0]][2*self.pos[1]] = colour

    def drive_forward(self, power = 255):
        """
        Turn both motors on to drive forward at full speed.

        Parameters:
            power (int): Optional speed to run motors at
        """
        
        self.motor_1.spin_forward(power)
        self.motor_2.spin_forward(power)                    

    def drive_backward(self, power = 255):
        """
        Turn both motors on to drive backward at full speed.

        Parameters:
            power (int): Optional speed to run motors at
        """
        self.motor_2.spin_backward(power)
        self.motor_1.spin_backward(power)

    def drive_stop(self):
        """
        Turn off both motors.
        """
        self.motor_2.spin_stop()
        self.motor_1.spin_stop()

    def get_encoders(self):
        """
        Get the rotational frequency of both motor encoders, signed for
            direction.

        Returns:
            (int, int): Encoder 1, and encoder 2 reading respectively
        """
        encoder_2 = - encoders.get_counts()[1] // 4
        encoder_1 = - encoders.get_counts()[0] // 4
        return (encoder_1, encoder_2)
    
    def get_button(self):
        """
        Gets the value of the built-in button
        
        Returns:
            (bool): True if button is pressed
        """
        return self.button.value() < 1
    
    def invert_motor_1(self):
        """
        Toggles the invert direction of motor 1
        """
        self.motor_1.invert_motor()
    
    def invert_motor_2(self):
        """
        Toggles the invert direction of motor 2
        """
        self.motor_2.invert_motor()
    
    def convert_length(self, distance_cm):
        """
        Converts a distance (cm) to an encoder value.
        
        Parameters:
            distance_cm (float): The distance to convert, in centimeters.
        Returns:
            int: The converted encoder value.
        """
        
        return int(distance_cm * 1050 / 4.5 / 3.14159)
    
    def convert_angle(self, angle_deg):
        """
        Converts an angle (degrees) to an encoder value.
        
        Parameters:
            angle_deg (float): The angle to convert, in degrees.
        Returns:
            int: The converted encoder value.
        """
        
        return int(angle_deg * 21000/3460)
    
    def wall_found(self):
        self.maze[2*self.pos[0]+self.dir[0]][2*self.pos[1]+self.dir[0]] = 1
    
    def drive_straight(self, length = 20, max_power = 255, loop_delay = 0.01, k = 20, wallk = 50, detect_dist = 2):
        """
        Use the encoders to drive straight for a specified distance. Uses a PID controller
        (with just the P lol) to throttle the leading motor's power.
        
        Parameters:
            length (int): Distance to travel, as measured by the encoders.
            max_power (int): Maximum power to run the motors at.
            loop_delay (int): Loop delay, measured in seconds.
            k (float): Proportionality constant for the PID controller. Smaller values are more sensitive.
        """
        
        length = self.convert_length(length)
        detect_dist = self.convert_length(detect_dist)
        encoder_1, encoder_2 = self.get_encoders()
        start_1 = encoder_1
        start_2 = encoder_2
        count_1 = 0
        count_2 = 0
        buffer = 0
        self.pos = [self.pos[0]+self.dir[0],self.pos[1]+self.dir[1]]
        wall_1, wall_2 = False, False
        wall_correct = 0

        while count_1 < length or count_2 < length:
            if count_1 >= count_2:
                self.motor_1.spin_forward(int(max_power * (1-(count_1 - count_2)/k)))
                self.motor_2.spin_forward(int(max_power))
            if count_2 > count_1:
                self.motor_2.spin_forward(int(max_power * (1-(count_2 - count_1)/k)))
                self.motor_1.spin_forward(int(max_power))
            if count_1 >= length:
                self.motor_1.spin_stop()
            if count_2 >= length:
                self.motor_2.spin_stop()
            time.sleep(loop_delay)
            encoder_1, encoder_2 = self.get_encoders()
            
            wall_2, _, wall_1 = self.get_ir_values()
            wall_correct += wall_1-wall_2
            wall_correct = min(wallk, max(-wallk, wall_correct))
            
            if self.get_ir_values(2):
                buffer += 1
                if buffer == 5:
                    self.wall_found()
                    diff = (encoder_1 - start_1) - (encoder_2 - start_2)
                    length = detect_dist + abs(diff)
                    start_1 = encoder_1 + max(diff, 0)
                    start_2 = encoder_2 + max(-diff, 0)
            
            count_1 = encoder_1 - start_1 - wall_correct
            count_2 = encoder_2 - start_2 + wall_correct

            if self.get_button():
                break # kill switch DO NOT REMOVE
            
            
        self.motor_1.spin_stop()
        self.motor_2.spin_stop()
    
    def drive_turn(self, mode, wall_pass = False, max_power = 90, loop_delay = 0.01, k = 50):
        """
        Docstring
        mode = 1 for 90 anticlockwise, -1 for 90 clockwise, pass = true if turning further
        """
        
        goal = abs(self.convert_angle(90*mode))
        goal_1 = goal
        goal_2 = goal
        sign = int(mode / abs(mode))
        encoder_1, encoder_2 = self.get_encoders()
        start_1 = encoder_1
        start_2 = encoder_2
        count_1 = 0
        count_2 = 0
        buffer = 0
        init_dir = self.dir
        self.dir = [-mode*self.dir[1],mode*self.dir[0]]
        
        while count_1 < goal_1 or count_2 < goal_2:
            if count_1 >= abs(count_2):
                self.motor_1.spin_power(-sign * int(max_power * (1-(count_1 - count_2)/k)))
                self.motor_2.spin_power(sign * int(max_power))
            if count_2 > count_1:
                self.motor_2.spin_power(sign * int(max_power))
                self.motor_1.spin_power(-sign * int(max_power * (1-(count_1 - count_2)/k)))
            if count_1 >= goal_1:
                self.motor_1.spin_stop()
            if count_2 >= goal_2:
                self.motor_2.spin_stop()
            time.sleep(loop_delay)
            encoder_1, encoder_2 = self.get_encoders()
            print(encoder_1,encoder_2)
                        
            count_1 = abs(encoder_1 - start_1)
            count_2 = abs(encoder_2 - start_2)
            
            if count_1 > 0.6 * goal_1 and count_2 > 0.6 * goal_2 and self.get_ir_values(2) and sign == int(mode / abs(mode)):
                buffer += 1
                if buffer > 5:
                    self.wall_found()
                    if not wall_pass:
                        sign = -int(mode / abs(mode))
                        goal_1 = count_1
                        goal_2 = count_2   
                        start_1 = encoder_1
                        start_2 = encoder_2
                        self.dir = init_dir
        
            if self.get_button():
                break # kill switch DO NOT REMOVE
            
        self.motor_1.spin_stop()
        self.motor_2.spin_stop()
        
    def get_neighbours(self, xy_coord,maze):
        '''
        Docstring
        '''
        
        x, y = xy_coord
        maze_x, maze_y = 2*x, 2*y

        neighbours = []

        directions = [
            (1,0),
            (-1,0),
            (0,1),
            (0,-1)
        ] 

        for dx, dy in directions:
            if 0 <= maze_x+dx < len(maze) and 0 <= maze_y+dy < len(maze[0]):
                if maze[maze_x+dx][maze_y+dy] == 0:
                    neighbours.append((x + dx, y + dy))

        
        return neighbours

    def floodfill(self, maze, goal=[4,4], size=9):
        '''
        Takes the current map of the maze and updates the distance map based on detected walls or floor colours.
        
        Parameters:
            maze (list[list[int]]): The map of the maze. 0 represents an empty tile. 1 represents a wall.
        -1 represents a green tile and 5 represents a red tile.
            goal (tuple): The coordinates of the goal.
            size (int): The side lengths of the square.

        Returns:
            distances (list[list[int]]): The new maze floodfill.
        '''

        distances = [[float('inf') for _ in range(size)] for _ in range(size)]
        
        distances[goal[0]][goal[1]] = 0
        Q = deque([goal], 100) # Micropython requires a maxlength for deque

        while Q:
            current = Q.popleft()
            cx, cy = current

            for nx, ny in self.get_neighbours(current,maze):
                mod = maze[nx*2][ny*2]
                if distances[nx][ny] > distances[cx][cy] + 1 + mod:
                    distances[nx][ny] = distances[cx][cy] + 1 + mod
                    Q.append((nx,ny))

        return distances

    def pickroute(self, distances):
        '''
        Docstring
        '''

        x,y = self.pos
        dx,dy = self.dir
        neighbours = self.get_neighbours(self.pos, self.maze)

        n_dists = []
        s_neighbours = []
        for n_x, n_y in neighbours:
            n_dists.append(distances[n_x][n_y])

        for i, dist in enumerate(neighbours):
            if n_dists[i] == min(n_dists):
                s_neighbours.append(neighbours[i])
        print(s_neighbours)

        
        # this bit is one of the main things we could improve
        if (x+dx,y+dy) in s_neighbours:
            route = (dx,dy)
        elif (x-dy,y+dx) in s_neighbours:
            route = (-dy,dx)
        elif (x+dy,y-dx) in s_neighbours:
            route = (dy,-dx)
        elif (x-dx,y-dy) in s_neighbours:
            route = (-dy,-dx)
        print(route)
        return route


