# 1716-2024-robot
This repo contains the code for FRC team 1716's 2024 robots. 
# Superstructure Proposal
- Swervebase and Mecanum test bot will be fully interchangable
- We will use feature branches, and squash-merge often
  - You own your branches, and are allowed to force push whenever you want
- Projects that will run on a robot with additional hardware should have their own (protected) branch
- `main` only contains drivebase code. After we get a consistent 2024 bot, we can change the default branch
- `main` and other non-feature branches will be protected. All PRs must pass CI before merging.
  - Github has a feature where you can require a specific person's approval when certian files are changed. We should consider this.
- Tests are encouraged, but not required. There are many situations where creating tests isn't possible, or would just end up testing WPILib.
