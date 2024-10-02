# Tic-Tac-Joe

Overview

This MATLAB simulation demonstrates two robotic arms, UR3E and CRX10IA, collaborating to play a game of Tic-Tac-Toe. The robots are programmed to alternate turns, marking X's and O's on a virtual 8x8 board. The simulation demonstrates robotic arm motion, pick-and-place operations, and basic game logic for Tic-Tac-Toe. The robot arms can support each other to reach the play squares that are not within one's 
reach.

Features
    UR3E Robot: Responsible for placing X's on the board (controlled by player).
    CRX10IA Robot: Responsible for placing O's on the board (controlled by AI).
    Game Logic: Implements Tic-Tac-Toe rules (4-in-a-row), checking for valid moves and determining the winner.
    Motion: Each robot arm follows smooth trajectories generated using quintic polynomial and RMRC to pick and place          
    game pieces at designated board locations. 

How to Run the Simulation

    Prerequisites:
        MATLAB (with Robotics Toolbox by Peter Corke and Parallel Processing Toolbox).

    Running the Simulation:
        Clone or download the project files into your working directory.
        Open and run the startup.rvc script to initialize the robotic toolbox.
        Run the main.m script to start the program.

Files Included

    main.m: Main script to run the Tic-Tac-Toe game simulation.
    UR3E.m: UR3E robot class and related functions.
    CRX10IA.m: CRX10IA class model and related functions.
    board.m: Contains the Tic-Tac-Toe game AI.
    GUI.m: Draw the game board on another figure and get user input about where to place X.
    helper.m: General functions that could be used by both arms.

Future Improvements

    Implement Threat-Space-Search for the AI.

<img width="1440" alt="Screen Shot 2024-10-01 at 2 33 33 am" src="https://github.com/user-attachments/assets/1d66f412-b00d-4de5-9c2b-a15232b98b91">
<img width="1440" alt="Screen Shot 2024-10-01 at 2 35 23 am" src="https://github.com/user-attachments/assets/712568e7-ed89-4fff-803e-9b5a98d002e0">
<img width="1440" alt="Screen Shot 2024-10-01 at 2 36 06 am" src="https://github.com/user-attachments/assets/dee8cfab-7377-4ae1-857e-35796d8326b5">
