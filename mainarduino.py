import time
import serial
import numpy as np
from rrt_star import RRTStar
from grid_map import create_grid_map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

def print_final_path(nodes):
    print("Final Path Coordinates:")
    for node in nodes:
        print(f"Coordinate: ({node[0]}, {node[1]})")
    print("Goal reached!")

def send_path_to_arduino(ser, nodes):
    """Send each coordinate in the final path to the Arduino over serial."""
    for node in nodes:
        # Format the node as "x,y" and send it over serial
        coord_str = f"{node[0]},{node[1]}\n"
        ser.write(coord_str.encode())
        print(f"Sent to Arduino: {coord_str.strip()}")  # Log the sent coordinate
        
        # Wait for Arduino's acknowledgment
        while True:
            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                print("Arduino:", response)  # Log Arduino's response
                if response == "reached":  # This message should be sent by Arduino
                    print("Waypoint reached:", coord_str.strip())
                    break
            time.sleep(0.1)

def main():
    start = (40, -140)
    goal = (20, -90)
    plyfile = '3dmap.ply'
    grid_resolution = 0.05

    # Create grid map from the PLY file
    occupancy_grid, boundaries = create_grid_map(plyfile, grid_resolution)

    # Plan the RRT* path
    rrt_star = RRTStar(start, goal, occupancy_grid, boundaries)
    nodes, edges = rrt_star.plan()

    # Print the coordinates of the final path
    print_final_path(nodes)

    # Save the occupancy grid and boundaries for further use
    np.save("occupancy_grid.npy", occupancy_grid)
    np.save("boundaries.npy", np.array(boundaries))  # Save boundaries as an array for easy loading

    # Initialize serial connection to Arduino
    try:
        ser = serial.Serial('/dev/ttyUSB0', 9600)  # Replace with the correct port
        time.sleep(2)  # Wait for Arduino to initialize
        
        # Send nodes (waypoints) to Arduino
        send_path_to_arduino(ser, nodes)
        
        ser.close()  # Close serial connection after sending all waypoints
    except serial.SerialException as e:
        print("Error connecting to Arduino:", e)

if __name__ == "__main__":
    main()

