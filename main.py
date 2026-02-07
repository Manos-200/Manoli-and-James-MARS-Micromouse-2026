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

GREEN_RANGE = (8000,10000)
RED_RANGE = (1320,20000)



if __name__ == "__main__":
         
    
    




### VERY IMPORTANT MAZE SOLVING STUFF
    while not mm.get_button(): # Start button
        pass
        time.sleep(0.1)
    time.sleep(1)
        
    goal = [4,4]
    while True:
        distances = mm.floodfill(mm.maze, goal)
        
        while mm.pos != goal:
            distances = mm.floodfill(mm.maze, goal)
            route = mm.pickroute(distances)
            if route == (-mm.dir[1],mm.dir[0]):
                time.sleep(0.5)
                mm.drive_turn(1)
                time.sleep(0.5)
            elif route == (mm.dir[1],-mm.dir[0]):
                time.sleep(0.5)
                mm.drive_turn(-1)
                time.sleep(0.5)
            elif route == (-mm.dir[0],-mm.dir[1]):
                time.sleep(0.5)
                mm.drive_turn(1,True)
                time.sleep(0.5)
                mm.drive_turn(1)
                time.sleep(0.5)
            if route == (mm.dir[0], mm.dir[1]):
                mm.led_debug_set(False)
                buffer = 0
                for _ in range(10):
                    if mm.get_ir_values(2):
                        buffer += 1
                    if buffer > 5:
                        mm.wall_found()
                        break
                    time.sleep(0.01)
                mm.drive_straight()
#                 colour = mm.check_colour(GREEN_RANGE, RED_RANGE)
#                 mm.record_colour(colour)
#                 mm.display_colour(colour)
                mm.led_debug_set(True)
            if mm.get_button():
                mm.drive_stop()
                break

        if mm.pos == [4,4]: goal = [0,0]
        if mm.get_button():
            mm.drive_stop()
            break
        
    mm.save_persistent_text('maze.txt', str(mm.maze))
    mm.save_persistent_text('neighbours.txt', str(mm.get_neighbours(mm.pos, mm.maze)))
    mm.save_persistent_text('encoders.txt', str(mm.encoders))
    mm.save_persistent_text('route.txt', str(mm.route))
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
        
