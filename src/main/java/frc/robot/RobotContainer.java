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
import frc.robot.commands.autos.PreloadTaxi;

public class RobotContainer {
  public Gyro imu = new PigeonV2(2);
  
  public SwerveDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(1, ModuleType.kCTRE);
  
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
      () -> {
        return commandDriverController.getRightX(); // Rotation
      },
      () -> false, // robot oriented vairable
      () -> false, // tow supplier
      driverController::getR2Button, // Precision/"Sniper Button"
      () -> {
        if (driverController.getCircleButton() || driverController.getCrossButton() || driverController.getTriangleButton()) {
          return true;
        }
        return false;
      },
      () -> { // Turn To angle Direction | TODO WIP
        if (driverController.getCircleButton()) { // turn to amp
          if (!IsRedSide()){
            return 90.0;
          }
          return 270.0;
        }
        if (driverController.getCrossButton()) {
           return 180.0;
        }
        if(driverController.getTriangleButton()) {
          return 0.0;
        }
        return swerveDrive.getImu().getHeading();
      }
    );

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
    try { // TODO fix for vendordeps not importing
    PathPlannerPath S4R3 = PathPlannerPath.fromPathFile("S4R3");

  	List<String> paths = AutoBuilder.getAllAutoNames();
    autoChooser.addOption("Do Nothing", Commands.none());

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
    if (paths.contains("S4R3")) {
      autoChooser.addOption("PreloadTaxi", AutoBuilder.buildAuto("PreloadTaxi"));
      autoChooser.addOption("PreloadTaxi", new PreloadTaxi(swerveDrive, List.of(S4R3)));
    }
    } catch (Exception e) {SmartDashboard.putBoolean("Auto Error", true);}
  
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