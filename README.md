# PacManBot_ROS2

PacManBot ROS is the ROS 2 codebase for a Pac-Man-inspired autonomous TurtleBot 4 project. The robot builds a 2D map of its environment, collects virtual pellets, and replans dynamically to avoid or pursue virtual ghosts depending on game state.

## Team Members
- Abdirahman Aden
- Gabriel Sandys
- Sean Vellequette

## Repository Structure
- `pacmanbot_msgs`: custom message definitions
- `pacmanbot_game_state`: ghost logic, pellet logic, maze generation
- `pacmanbot_planning`: risk-reward planning and threat-aware path planning
- `pacmanbot_bringup`: launch files and configuration
