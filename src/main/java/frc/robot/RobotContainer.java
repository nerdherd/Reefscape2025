// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.util.NerdyMath;

public class RobotContainer {
  public Gyro imu = new PigeonV2(2);
  
  public SwerveDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);
  
  private final CommandPS4Controller commandDriverController = new CommandPS4Controller(
    ControllerConstants.kDriverControllerPort);
  private final PS4Controller driverController = commandDriverController.getHID();
  private final CommandPS4Controller commandOperatorController = new CommandPS4Controller(
    ControllerConstants.kOperatorControllerPort);
  private final PS4Controller operatorController = commandOperatorController.getHID();

  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  
  private SwerveJoystickCommand swerveJoystickCommand;
  
  /**
   * The container for the robot. Contain
   * s subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      swerveDrive = new SwerveDrivetrain(imu);

    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }
    
    initAutoChoosers();
    initShuffleboard();
    initDefaultCommands_teleop();

    // Configure the trigger bindings
    // Moved to teleop init
    
    DriverStation.reportWarning("Initalization complete", false);

  }

  static boolean isRedSide = false;

  public static void refreshAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isRedSide = (alliance.get() == DriverStation.Alliance.Red);
    }
  }

  public static boolean IsRedSide() {
    return isRedSide;
  }

  public void initDefaultCommands_teleop() {
      swerveJoystickCommand = 
      new SwerveJoystickCommand(
        swerveDrive,
        () -> -commandDriverController.getLeftY(), // Horizontal translation
        commandDriverController::getLeftX, // Vertical Translation
        // () -> 0.0, // debug
        () -> {
          // if (driverController.getL2Button()) {
          //   SmartDashboard.putBoolean("Turn to angle 2", true);
          //   double turnPower = apriltagCamera.getTurnToTagPower(swerveDrive, angleError, IsRedSide() ? 4 : 7, adjustmentCamera); 
          //   SmartDashboard.putNumber("Turn Power", turnPower);
          //   return turnPower;
          // }
          // SmartDashboard.putBoolean("Turn to angle 2", false);
          if (swerveDrive.getTurnToAngleMode()) {
            return 0.0;
          }
          return commandDriverController.getRightX(); // Rotation
        },

        // driverController::getSquareButton, // Field oriented
        () -> false, // should be robot oriented now on true
        () -> false,
        // driverController::getCrossButton, // Towing
        // driverController::getR2Button, // Precision/"Sniper Button"
        () -> driverController.getR2Button(), // Precision mode (disabled)
        () -> {
          if (swerveDrive.getTurnToAngleMode()) {
            return (
            Math.abs(driverController.getRightX()) > 0.05 
            || Math.abs(driverController.getRightY()) > 0.05
            || driverController.getCircleButton()
            );
          } 
          else if (driverController.getCircleButton()) {
            return(true);  
          }
          else {
            return(false);
          }
        }, 
        // () -> false, // Turn to angle (disabled)
        () -> { // Turn To angle Direction
          if (driverController.getCircleButton()) { //turn to amp
            if (!IsRedSide()){
              return 270.0;
            }
            return 90.0;
          } 
          else {
            double xValue = commandDriverController.getRightX();
            double yValue = commandDriverController.getRightY();
            double magnitude = Math.sqrt((xValue*xValue) + (yValue*yValue));
            if (magnitude > 0.49) {
              double angle = (90 + NerdyMath.radiansToDegrees(Math.atan2(commandDriverController.getRightY(), commandDriverController.getRightX())));
              angle = (((-1 * angle) % 360) + 360) % 360;
              SmartDashboard.putNumber("desired angle", angle);
              return angle;
            }
            return 1000.0;
          }
        });

      swerveDrive.setDefaultCommand(swerveJoystickCommand);
  }

  public void initDefaultCommands_test() {}

  public void configureBindings_teleop() {
    // Driver bindings

    commandDriverController.share().whileTrue(
      Commands.runOnce(() -> swerveDrive.zeroGyroAndPoseAngle())
    );
  }

  public void configureBindings_test() {}

  private void initAutoChoosers() {
  	List<String> paths = AutoBuilder.getAllAutoNames();
    autoChooser.addOption("Do Nothing", Commands.none());

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(LOG_LEVEL.MINIMAL);    
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command currentAuto = autoChooser.getSelected();

    swerveDrive.setDriveMode(DRIVE_MODE.AUTONOMOUS);
    return currentAuto;
  }
}