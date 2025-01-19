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
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.commands.autos.PreloadTaxi;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.util.GuliKit;

public class RobotContainer {
  public Gyro imu = new PigeonV2(2);

  public SwerveDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(1, ModuleType.kCTRE);
  
  public AlgaeRoller algaeRoller;
  public Elevator elevator;

  // private final CommandPS4Controller commandDriverController = new CommandPS4Controller(
  //   ControllerConstants.kDriverControllerPort);
  // private final PS4Controller driverController = commandDriverController.getHID();
  // private final CommandPS4Controller commandOperatorController = new CommandPS4Controller(
  //   ControllerConstants.kOperatorControllerPort);
  // private final PS4Controller operatorController = commandOperatorController.getHID();

  private final GuliKit driverController = new GuliKit(ControllerConstants.kDriverControllerPort, true, true);
  private final GuliKit operatorController = new GuliKit(ControllerConstants.kOperatorControllerPort, true, true);
  
  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;
  
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  
  static boolean isRedSide = false;
  
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

    elevator = new Elevator();
    algaeRoller = new AlgaeRoller();
    
    // initAutoChoosers();
    initShuffleboard();
    initDefaultCommands_teleop();
    initAutoChoosers();

    // Configure the trigger bindings
    // Moved to teleop init
    
    
    SmartDashboard.putData("Swerve Drive", swerveDrive);

    DriverStation.reportWarning("Initalization complete", false);

  }


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
      () -> -driverController.getLeftY(), // Horizontal translation
      driverController::getLeftX, // Vertical Translation
      () -> {
        return driverController.getRightX(); // Rotation
      },
      () -> false, // robot oriented vairable
      () -> false, // tow supplier
      driverController::getRTdigital, // Precision/"Sniper Button"
      () -> {
        if (driverController.getA() || driverController.getB() || driverController.getX()) {
          return true;
        }
        return false;
      },
      () -> { // Turn To angle Direction | TODO WIP
        if (driverController.getA()) { // turn to amp
          if (!IsRedSide()){
            return 90.0;
          }
          return 270.0;
        }
        if (driverController.getB()) {
           return 180.0;
        }
        if(driverController.getX()) {
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

    driverController.buttonA().onTrue(
      Commands.runOnce(() -> swerveDrive.zeroGyroAndPoseAngle())
    );

    driverController.buttonA().onTrue(elevator.goToPosition(ElevatorConstants.kElevatorL1Position))
      .onFalse(elevator.goToPosition(ElevatorConstants.kElevatorStowPosition)); 
    driverController.buttonX().onTrue(elevator.goToPosition(ElevatorConstants.kElevatorL2Position))
      .onFalse(elevator.goToPosition(ElevatorConstants.kElevatorStowPosition)); 
    driverController.buttonY().onTrue(elevator.goToPosition(ElevatorConstants.kElevatorL3Position))
      .onFalse(elevator.goToPosition(ElevatorConstants.kElevatorStowPosition)); 
    driverController.buttonB().onTrue(elevator.goToPosition(ElevatorConstants.kElevatorL4Position))
      .onFalse(elevator.goToPosition(ElevatorConstants.kElevatorStowPosition)); 
    
    driverController.triggerLT().onTrue(algaeRoller.intake()) // hold it :)
      .onFalse(algaeRoller.stop());
    driverController.triggerRT().onTrue(algaeRoller.shootBarge()) // hold it :)
      .onFalse(algaeRoller.stop());

  }

  public void configureBindings_test() {}
  
  private void initAutoChoosers() {
    try { // TODO fix for vendordeps not importing
    PathPlannerPath S4R3 = PathPlannerPath.fromPathFile("S4R3");

  	List<String> paths = AutoBuilder.getAllAutoNames();
    

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
    autoChooser.addOption("Do Nothing", Commands.none());
    autoChooser.addOption("Taxi", AutoBuilder.buildAuto("Taxi"));
    autoChooser.addOption("Squarto", AutoBuilder.buildAuto("Squarto"));
    autoChooser.addOption("Test", AutoBuilder.buildAuto("Test"));
    // if (paths.contains("S4R3")) {
      autoChooser.addOption("PreloadTaxi", AutoBuilder.buildAuto("PreloadTaxi"));
      autoChooser.addOption("PreloadTaxi2", new PreloadTaxi(swerveDrive, List.of(S4R3)));
    // }
    } catch (Exception e) {SmartDashboard.putBoolean("Auto Error", true);}
  
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(LOG_LEVEL.MINIMAL);   
    algaeRoller.initShuffleboard(loggingLevel); 
    elevator.initShuffleboard(loggingLevel);
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