# 1716-2024-robot
This repo contains the code for FRC team 1716's 2024 robots.
# IMPORTANT: Building code
Run the `compileJava` task to build the code. If you run the `build` task, it will fail if the formatting is incorrect.
# Notes
## Git Data on Deploy
On deploy, a commit is made and tagged, then the head resets. To push these commits, run `git push --tags`
## Spotless
Spotless is our code formatter. To run spotless, run the `spotlessApply` task
