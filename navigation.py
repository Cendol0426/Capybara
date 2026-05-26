import serial
import time
import math
import test_vision

# ser = serial.Serial('/dev/ttyUSB0', 115200)

cubes_present = {
    f"CUBE{i}": True for i in range(1, 17)
}

checkpoint = {
    "CP" : True
}
# Front layer CUBE 1 - 8
# Back layer CUBE 9 - 16
# Each enter shows which nodes are adjacent to each other
game_graph = {
    "START" :   ["LANE1", "LANE2", "LANE3", "LANE4", "LANE5", "LANE6", "LANE7", "LANE8"],
    "END"   :   ["LANE1", "LANE2", "LANE3", "LANE4", "LANE5", "LANE6", "LANE7", "LANE8"],
    "PRECP" :   ["LANE9", "CP"],
    "CP"    :   ["PRECP"],

    "LANE1" :   ["START", "END", "CUBE1", "LANE2", "LANE9"],
    "LANE2" :   ["START", "END", "CUBE2", "LANE1", "LANE2", "LANE9"],
    "LANE3" :   ["START", "END", "CUBE3", "LANE2", "LANE4", "LANE9"],
    "LANE4" :   ["START", "END", "CUBE4", "LANE3", "LANE5", "LANE9"],
    "LANE5" :   ["START", "END", "CUBE5", "LANE4", "LANE6", "LANE9"],
    "LANE6" :   ["START", "END", "CUBE6", "LANE5", "LANE7", "LANE9"],
    "LANE7" :   ["START", "END", "CUBE7", "LANE6", "LANE8", "LANE9"],
    "LANE8" :   ["START", "END", "CUBE8", "LANE7", "LANE9"],
    "LANE9" :   ["LANE1", "LANE2", "LANE3", "LANE4", "LANE5", "LANE6", "LANE7", "LANE8", "PRECP"],

    # Front layer
    "CUBE1" :   ["LANE1", "CUBE2", "CUBE9"],
    "CUBE2" :   ["LANE2", "CUBE1", "CUBE3", "CUBE10"],
    "CUBE3" :   ["LANE3", "CUBE2", "CUBE4", "CUBE11"],
    "CUBE4" :   ["LANE4", "CUBE3", "CUBE5", "CUBE12"],
    "CUBE5" :   ["LANE5", "CUBE4", "CUBE6", "CUBE13"],
    "CUBE6" :   ["LANE6", "CUBE5", "CUBE7", "CUBE14"],
    "CUBE7" :   ["LANE7", "CUBE6", "CUBE8", "CUBE15"],
    "CUBE8" :   ["LANE8", "CUBE7", "CUBE16"],

    # Back layer
    "CUBE9" :   ["CUBE1", "CUBE10"],
    "CUBE10":   ["CUBE2", "CUBE9", "CUBE11"],
    "CUBE11":   ["CUBE3", "CUBE10", "CUBE12"],
    "CUBE12":   ["CUBE4", "CUBE11", "CUBE13"],
    "CUBE13":   ["CUBE5", "CUBE12", "CUBE14"],
    "CUBE14":   ["CUBE6", "CUBE13", "CUBE15"],
    "CUBE15":   ["CUBE7", "CUBE14", "CUBE16"],
    "CUBE16":   ["CUBE8", "CUBE15"]
}

node_coordinates = {
    "START" :   (186.5, 25),
    "END"   :   (186.5, 25),
    "PRECP" :   (15, 262),
    "CP"    :   (15, 287),
    
    "LANE1" :   (186.5, 82),
    "LANE2" :   (186.5, 100),
    "LANE3" :   (186.5, 118),
    "LANE4" :   (186.5, 136),
    "LANE5" :   (186.5, 154),
    "LANE6" :   (186.5, 172),
    "LANE7" :   (186.5, 190),
    "LANE8" :   (186.5, 208),
    "LANE9" :   (186.5, 262),
    
    "CUBE1" :   (41, 82),
    "CUBE2" :   (41, 100),
    "CUBE3" :   (41, 118),
    "CUBE4" :   (41, 136),
    "CUBE5" :   (41, 154),
    "CUBE6" :   (41, 172),
    "CUBE7" :   (41, 190),
    "CUBE8" :   (41, 208),
    
    "CUBE9" :   (16, 82),
    "CUBE10":   (16, 100),
    "CUBE11":   (16, 118),
    "CUBE12":   (16, 136),
    "CUBE13":   (16, 154),
    "CUBE14":   (16, 172),
    "CUBE15":   (16, 190),
    "CUBE16":   (16, 208)
}

robot_state = {
    "current_node": "START",
    "heading": 90.0 # Assume 90 degrees is facing "North" (up the Y axis)
}

#PLAN:
# go next node
# get distance
# once reach, send activation to serial
# get vision
# got cube, take.
# no cube, go next node
# repeat until cube16

# Breath-first search
def get_path(start, target, graph, state):
    queue = [(start, [start])]
    visited = set()
    visited.add(start)

    while queue:
        current, path = queue.pop(0)
        if current == target:
            return path
        for neighbour in graph[current]:
            if neighbour not in visited:
                # is this move legal
                is_safe_zone = neighbour not in state
                # can move to space if cube is collected
                is_empty_space = (neighbour in state and state[neighbour] == False)
                
                is_target = (neighbour == target)

                if is_safe_zone or is_empty_space or is_target:
                    visited.add(neighbour)
                    queue.append((neighbour, path + [neighbour]))
    return None

def drive_to_next_node(next_node):
    current = robot_state["current_node"]

    curr_x, curr_y = node_coordinates[current]
    next_x, next_y = node_coordinates[next_node]
    
    # Calculate Euclidean distance
    dx = next_x - curr_x
    dy = next_y - curr_y
    distance = math.hypot(dx, dy)
    
    # Calculate the angle to the target
    target_heading = math.degrees(math.atan2(dy, dx))
    
    # Calculate how much the robot needs to turn
    turn_angle = target_heading - robot_state["heading"]
    # Normalize to -180 - 180
    turn_angle = (turn_angle + 180) % 360 - 180
    
    print(f"\nMoving {current} -> {next_node}")
    
    # Execute the Turn Instruction
    if turn_angle > 1.0:
        send_instruction(f"M:TL:{abs(turn_angle)}")
    elif turn_angle < -1.0:
        send_instruction(f"M:TR:{abs(turn_angle)}")
        
    # Execute the Forward Instruction
    if distance > 1.0:
        send_instruction(f"M:FWD:{distance}")
        
    # Update robot state
    robot_state["current_node"] = next_node
    robot_state["heading"] = target_heading

def get_vision():
    return

def send_instruction(cmd_string):
    print(f"Command: {cmd_string}")
    
    # full_cmd = f"{cmd_string}\n".encode('utf-8')
    # ser.write(full_cmd)
    
    # # Wait for the 'Movement executed' reply from CommandHandler
    # while True:
    #     if ser.in_waiting > 0:
    #         reply = ser.readline().decode('utf-8').strip()
    #         if "executed" in reply.lower():
    #             return
    #     time.sleep(0.05)

    time.sleep(1)

def main():    
    mission_targets = [f"CUBE{i}" for i in range(1, 17)]
    
    for target_cube in mission_targets:
        # Check if we already got it
        if cubes_present[target_cube] == False:
            continue
            
        print(f"\nNEW TARGET: {target_cube}")
        
        # Calculate path from current position
        current_loc = robot_state["current_node"]
        path = get_path(current_loc, target_cube, game_graph, cubes_present)
        
        if path is None:
            print(f"{target_cube} is blocked! Skipping for now.")
            continue
            
        print(f"Path Found: {path}")
        
        # Drive step-by-step
        for step in path[1:]:
            drive_to_next_node(step)
            
        print(f"Arrived at {target_cube}.")
        
        # VISION AND GRIPPER
        is_cube, is_correct = True, True
        if (is_cube and is_correct):
            send_instruction("M:PICKUP")
            print("PICKUP")
        
        # Mark as collected
        cubes_present[target_cube] = False
        print(f"{target_cube} collected. It is now empty space.")

        target = "CP"
        path = get_path(robot_state["current_node"], target, game_graph, checkpoint)
        if path is None:
            print(f"Cannot reach {target_cube}, skipping for now.")
            continue
            
        print(f"Path Found: {path}")
        
        for step in path[1:]:
            drive_to_next_node(step)
            
        print(f"Arrived at {target_cube}.")
    
    print("Routing back to END zone...")
    go_home_path = get_path(robot_state["current_node"], "END", game_graph, cubes_present)
    for step in go_home_path[1:]:
        drive_to_next_node(step)

if __name__ == "__main__":
    main()