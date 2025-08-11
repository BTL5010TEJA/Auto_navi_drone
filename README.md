

AI-driven approach. The simulation is developed using 
Autonomous UAV Navigation with Geo-Fencing and GPS
This MATLAB project simulates a UAV (drone) navigating autonomously from a start point to a destination while avoiding obstacles and staying within a predefined geo-fence.

Key Components
Geo-Fence Boundaries

Defined by two points:

geoFenceMin → minimum X, Y, Z coordinates.

geoFenceMax → maximum X, Y, Z coordinates.

Represented by a transparent 3D box with blue dashed edges.

Ensures the UAV remains inside the permitted airspace.

Waypoints and Path Planning

Start Point → Yellow sphere.

Mid Waypoints → Green spheres (intermediate navigation points).

End Point → Red sphere (final destination).

UAV path shown as a green line connecting these waypoints.

Navigation is designed to reach the goal without hitting obstacles.

Obstacles

Box Obstacles → Yellow-outlined cubes.

Cylinder Obstacles → Blue cylinders.

Positioned inside the geo-fence to test obstacle avoidance.

Navigation Logic

The UAV uses simulated GPS coordinates for position tracking.

The path is dynamically generated to bypass any obstacles inside the fence.

Goal message "Goal Reached!" is displayed upon successfully reaching the endpoint.

3D Visualization

Interactive 3D plot shows the UAV path, obstacles, and geo-fence.

X, Y, and Z axes represent spatial coordinates.

The live simulation visually demonstrates the UAV maneuvering around obstacles.
MATLAB.

Execution and Real-Time Uses
<img width="1915" height="688" alt="Screenshot 2025-08-11 230535" src="https://github.com/user-attachments/assets/434b7e80-62cf-43f4-8926-227ab4a19f6b" />

<img width="1918" height="992" alt="Screenshot 2025-08-11 225901" src="https://github.com/user-attachments/assets/12d31d2c-bcbc-4168-879f-c74fdbfc09c6" />

<img width="1915" height="688" alt="Screenshot 2025-08-11 230535" src="https://github.com/user-attachments/assets/2c059b18-b1c7-4648-b027-523aac8dc7ab" />


Workflow
Define geoFenceMin and geoFenceMax to set spatial limits.

Place start, waypoints, and goal within the geo-fence.

Insert random or predefined obstacles inside the area.

Run the navigation algorithm:

UAV moves from start → waypoints → goal.

Avoids obstacles using path adjustment logic.

Simulation runs until UAV reaches the goal, confirming safe navigation.

The simulation's execution involves a model-based or code-based approach in MATLAB to systematically test the drone's navigation capabilities. The AI-driven system handles the logic for obstacle avoidance and autonomous navigation.

In a real-time scenario, this system could have various practical applications:

Automated Inspection: Drones could autonomously inspect large indoor spaces like warehouses or factories for maintenance issues or inventory management.

Security and Surveillance: The system could patrol buildings autonomously, providing real-time video feeds without the need for constant human supervision.

Logistics and Delivery: Drones could be used for last-mile delivery within large facilities, moving items from one location to another efficiently.
