# 1716-2024-robot
This repo contains the code for FRC team 1716's 2024 robots.
# Robot Controlls
![Main Driver Controlls](https://github.com/Brown-County-FIRST-Robotics/1716-2024-robot/blob/CF_Custom_Trajectory_Follower/docs/FRC Button Layout-1.png)
![Second Driver Controlls](https://github.com/Brown-County-FIRST-Robotics/1716-2024-robot/blob/CF_Custom_Trajectory_Follower/docs/FRC Button Layout-2.png)
# IMPORTANT: Building code
Run the `compileJava` task to build the code. If you run the `build` task, it will fail if the formatting is incorrect.
# Notes
## Git Data on Deploy
On deploy, a commit is made and tagged, then the head resets. To push these commits, run `git push --tags`
## Spotless
Spotless is our code formatter. To run spotless, run the `spotlessApply` task
