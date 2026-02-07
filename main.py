"""
This file is provided as a sample of basic initialisation and working for
"plug-and-play" of the drivers, but is expected to be altered to implement
system control algorithms.
"""
from micromouse import Micromouse
from machine import Pin, ADC
import time
from encoders import Encoders

mm = Micromouse()

GREEN_RANGE = (2400,30000)
RED_RANGE = (1320,20000)


if __name__ == "__main__":
    
    mm.drive_straight(30, 200)
        
    
    
#     mm.motor_1.spin_power(100)
#     mm.motor_2.spin_power(-100)
#     while True:
#         print(mm.get_encoders())
#         if mm.get_button():
#             mm.drive_stop()
#             break
#         time.sleep(0.01)
    
#     for i in range(4):
#         mm.drive_straight(10, 255)
#         time.sleep(0.1)
#         mm.drive_turn(90, 255)
#         time.sleep(0.1)

# {mm.check_colour(GREEN_RANGE, RED_RANGE)[0]} \t Red LDR: {mm.check_colour(GREEN_RANGE, RED_RANGE)[1]}{mm.get_ldr_values()[0]} \t Red LDR: {mm.get_ldr_values()[1]}

#     mm.drive_straight(50,90)



### VERY IMPORTANT MAZE SOLVING STUFF
#     while not mm.get_button(): # Start button
#         pass
#         time.sleep(0.2)
#     time.sleep(1)
#     
#     buffer = 0
#     for _ in range(10):
#         if mm.get_ir_values(2):
#             buffer += 1
#         if buffer > 5:
#             mm.wall_found()
#         
#     goal = [4,4]
#     while True:
#         distances = mm.floodfill(mm.maze, goal)
# 
#         while mm.pos != goal:
#             distances = mm.floodfill(mm.maze, goal)
#             route = mm.pickroute(distances)
#             print(mm.maze)
#             if route == (-mm.dir[1],mm.dir[0]):
#                 mm.drive_turn(1)
#             elif route == (mm.dir[1],-mm.dir[0]):
#                 mm.drive_turn(-1)
#             elif route == (-mm.dir[0],-mm.dir[1]):
#                 mm.drive_turn(1,True)
#                 mm.drive_turn(1)
#             if route == (mm.dir[0], mm.dir[1]):
#                 mm.led_debug_set(False)
#                 mm.drive_straight(18)
#                 colour = mm.check_colour(GREEN_RANGE, RED_RANGE)
#                 mm.record_colour(colour)
#                 mm.led_debug_set(True)
#             if mm.get_button():
#                 mm.drive_stop()
#                 break
# #                 else:
# #                 mm.floodfill(mm.maze,goal)
# 
#         if mm.pos == [4,4]: goal = [0,0]
#         if mm.get_button():
#             mm.drive_stop()
#             break
## END OF VERY IMPORTANT STUFF


#     
#     while True:
#         print(f'Green LDR: {mm.check_colour(GREEN_RANGE, RED_RANGE)[0]} \t Red LDR: {mm.check_colour(GREEN_RANGE, RED_RANGE)[1]}')
#         time.sleep(0.2)
    
    # Code scaffolding
    

#     
#     map_maze()
    
    # Detect walls + update map
    # Determine next action (turn + move or just move or continue what I'm already doing)
    # Perform action.
        # Use encoders to drive straight -- restructure this so that I can run it in main loop without interrupting it
        # Use walls to ensure on track
        # Detect colour
    # Small delay
    # Repeat
        
