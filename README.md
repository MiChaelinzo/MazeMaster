# MazeMaster
🤖"Imagine a tiny, intelligent robot navigating complex mazes with lightning speed and precision. That's MazeMaster, our cutting-edge micromouse engineered for the IEEE Micromouse Competition.
![Gemini_Generated_Image_k4ogbzk4ogbzk4og](https://github.com/user-attachments/assets/e75333f2-2b41-4965-bfc8-4612f51b6e8e)


**Key Features and Code Elements:**

*   **Arrowhead Shape:** This is implemented in the physical design of the robot.
*   **Three-Wheel Configuration:** This is also implemented in the physical design. The code assumes differential drive control (two motors controlling two wheels).
*   **Infrared Sensors:** The code reads distance measurements from the front and side Sharp IR sensors.
*   **PID Control:** The `executeMovement` function implements a PID controller to adjust motor speeds based on the distance from the left wall, ensuring smooth wall-following behavior.
*   **Wall-Following Algorithm:** The `determineNextMove` function implements a simple wall-following algorithm based on the IR sensor readings.
*   **Dead Reckoning:**  Not explicitly implemented here, but you could add code to track wheel movements and estimate the robot's position and orientation within the maze.

![Gemini_Generated_Image_k4ogbyk4ogbyk4og](https://github.com/user-attachments/assets/73555af8-29b4-4fec-863e-a694d5286970)
