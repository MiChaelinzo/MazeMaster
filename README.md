# MazeMaster
🤖"Imagine a tiny, intelligent robot navigating complex mazes with lightning speed and precision. That's MazeMaster, our cutting-edge micromouse engineered for the IEEE Micromouse Competition.


**Key Features and Code Elements:**

*   **Arrowhead Shape:** This is implemented in the physical design of the robot.
*   **Three-Wheel Configuration:** This is also implemented in the physical design. The code assumes differential drive control (two motors controlling two wheels).
*   **Infrared Sensors:** The code reads distance measurements from the front and side Sharp IR sensors.
*   **PID Control:** The `executeMovement` function implements a PID controller to adjust motor speeds based on the distance from the left wall, ensuring smooth wall-following behavior.
*   **Wall-Following Algorithm:** The `determineNextMove` function implements a simple wall-following algorithm based on the IR sensor readings.
*   **Dead Reckoning:**  Not explicitly implemented here, but you could add code to track wheel movements and estimate the robot's position and orientation within the maze.

**Spider MazeMaster** 

![Gemini_Generated_Image_5m73m65m73m65m73](https://github.com/user-attachments/assets/d33139b1-9443-414e-8c4e-e46c9f62e8c1)

**Spider 2 MazeMaster**

![Gemini_Generated_Image_5m73m55m73m55m73](https://github.com/user-attachments/assets/c4ab6027-cb4c-4f80-9941-080bccefe4dc)

**Arrowhead MazeMaster**

![Gemini_Generated_Image_k4ogbzk4ogbzk4og](https://github.com/user-attachments/assets/442a6a0a-4bfe-441f-87b2-0ed524ab8bee)

**Arrowhead 2 MazeMaster**

![Gemini_Generated_Image_k4ogbyk4ogbyk4og](https://github.com/user-attachments/assets/a9c49e5d-4b49-42b7-bb93-2a4b54cdd692)


