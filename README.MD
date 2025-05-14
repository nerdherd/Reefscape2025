# Reefscape2025

FRC Team 687's code for its Reefscape 2025 competition robot, *Squidward*.

## How to Use

This program requires WPILib VSCode 2025 and FRC Game Tools 2025 to be installed.
Clone this repository from <https://github.com/nerdherd/Reefscape2025> 
or download and unzip the [latest release](https://github.com/nerdherd/Reefscape2025/releases/latest),
open it in VSCode, and press shift + F5 to deploy it to the robot. 
Then use Driver Station to enable the robot.

The robot is controlled using two PS4 Controllers, with alternatives for operator input. For compatibility purposes, a wrapper class `Controller.java` is used to translate button inputs to generic inputs.

## Structure

src/main/java/frc/robot/ — Main robot code

subsystems/ — Contains subsystem classes like Drivetrain, Shooter, Arm, etc.

commands/ — Autonomous and teleop commands

Constants.java — Robot-wide constants

RobotContainer.java — Central place for binding subsystems and commands

Controller.java — Wrapper for gamepad input

## Contributing

Please branch off from the `V1` branch for any changes, and then submit a PR when finished. 
Changes will be merged into the main branch and tagged as a release every two weeks.

## 3rd Party Libraries

This repository uses:
- [Pathplanner 2025.2.3](https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json)
- [CTRE Phoenix v5](https://store.ctr-electronics.com/software/)
- [CTRE Phoenix v6](https://store.ctr-electronics.com/software/)
- [WPIlib New Commands Library](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html)
