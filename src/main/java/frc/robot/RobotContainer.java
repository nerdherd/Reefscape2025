// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

// import frc.robot.commands.autos.PreloadTaxi;
// import frc.robot.commands.autSquare;
import frc.robot.commands.SwerveJoystickCommand;
// import frc.robot.commands.autos.AutoDriving;
import frc.robot.commands.autos.twoPiece;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.ElevatorPivot;

import frc.robot.util.Controller;

public class RobotContainer {
  public Gyro imu = new PigeonV2(1);

  public SwerveDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  public IntakeRoller intakeRoller;
  public Elevator elevator;
  public ElevatorPivot elevatorPivot;
  public IntakeWrist coralWrist;

  public twoPiece bottom2Piece;
  public twoPiece mid2Piece;
  public twoPiece top2Piece;
  public twoPiece driveAuto;

  private final Controller driverController = new Controller(ControllerConstants.kDriverControllerPort);
  private final Controller operatorController = new Controller(ControllerConstants.kOperatorControllerPort, true, true);
  
  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;
  
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  
  static boolean isRedSide = false;
  
  private SwerveJoystickCommand swerveJoystickCommand;
  
  private static boolean USE_ELEV = true;
   
     public RobotContainer() {
    try {
      swerveDrive = new SwerveDrivetrain(imu);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    } 

    if (USE_ELEV) {
      intakeRoller = new IntakeRoller();
      coralWrist = new IntakeWrist();
      elevator = new Elevator();
      elevatorPivot = new ElevatorPivot();
    }

  
    try { // ide displayed error fix
      bottom2Piece = new twoPiece(swerveDrive, intakeRoller, elevator, "Bottom2Piece");
      mid2Piece = new twoPiece(swerveDrive, intakeRoller, elevator, "Mid2Piece");
      top2Piece = new twoPiece(swerveDrive, intakeRoller, elevator, "Top2Piece");
      driveAuto = new twoPiece(swerveDrive, intakeRoller, elevator, "DriveAuto");
    } catch (IOException e) {
      DriverStation.reportError("IOException for twoPiece", e.getStackTrace());
    } catch (ParseException e) {
      DriverStation.reportError("ParseException for twoPiece", e.getStackTrace());
    }

    initShuffleboard();
    // initDefaultCommands_test();
    configureBindings_test();
    initDefaultCommands_teleop();
    configureBindings_teleop();
    initAutoChoosers();
    
    SmartDashboard.putData("Swerve Drive", swerveDrive);
    DriverStation.reportWarning("Initalization complete", false);
  }


  public static void refreshAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent())
      isRedSide = (alliance.get() == DriverStation.Alliance.Red);
  }

  public static boolean IsRedSide() {
    return isRedSide;
  }

  public void initDefaultCommands_teleop() {
    swerveJoystickCommand = 
    new SwerveJoystickCommand(
      swerveDrive,
      () -> -driverController.getLeftY(), // Horizontal translation
      () -> driverController.getLeftX(), // Vertical Translation
      () -> driverController.getRightX(), // Rotation
      () -> true, // robot oriented variable (false = field oriented)
      () -> false, // tow supplier
      () -> driverController.getTriggerRight(), // Precision/"Sniper Button"
      () -> { return driverController.getButtonRight() || driverController.getButtonDown() || driverController.getButtonUp(); },
      () -> { // Turn To angle Direction | TODO WIP
        if (driverController.getButtonRight())
          if (!IsRedSide())
            return 90.0; 
          else return 270.0;
        if (driverController.getButtonDown())
           return 180.0;
        if (driverController.getButtonUp())
          return 0.0;
        return swerveDrive.getImu().getHeading();
      }
    );

    swerveDrive.setDefaultCommand(swerveJoystickCommand);

}

  public void initDefaultCommands_test() {}

  public void configureBindings_teleop() {
    // Driver bindings
    driverController.controllerLeft().onTrue(
      Commands.runOnce(() -> swerveDrive.zeroGyroAndPoseAngle()) // TODO: When camera pose is implemented, this won't be necessary anymore
    );
    

    driverController.triggerRight().onTrue(intakeRoller.intake())
                                    .onFalse(intakeRoller.stop());
    driverController.buttonUp().onTrue(elevator.moveToReefL3())
                                    .onFalse(elevator.stow());
    driverController.buttonLeft().onTrue(elevatorPivot.moveToPickup())
                                    .onFalse(elevatorPivot.moveToStow());
    driverController.buttonRight().onTrue(coralWrist.moveToReefL14())
                                    .onFalse(coralWrist.moveToStow());
    // if(USE_ELEV) {
    //   // driverController.triggerRight()
    //   // .onTrue(elevatorPivot.moveToPickup()) // hold it :)
    //   // .onFalse(elevatorPivot.moveToStow());
    // }
  }

  public void configureBindings_test() {
    driverController.triggerLeft()
      .whileTrue(bottom2Piece.runAuto())
      .onFalse(bottom2Piece.stopAuto());
    
    driverController.triggerRight()
      .whileTrue(mid2Piece.runAuto())
      .onFalse(mid2Piece.stopAuto());

    driverController.bumperLeft()
      .whileTrue(top2Piece.runAuto())
      .onFalse(top2Piece.stopAuto());

    driverController.bumperRight()
      .whileTrue(driveAuto.driveAuto());
  }
  
  private void initAutoChoosers() {
    try { // fix for vendordeps not importing
    PathPlannerPath S4R3 = PathPlannerPath.fromPathFile("S4R3");

  	List<String> paths = AutoBuilder.getAllAutoNames();
    
    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
    autoChooser.setDefaultOption("Bottom 2 Piece", new twoPiece(swerveDrive, intakeRoller, elevator, "Bottom2Piece"));
    autoChooser.addOption(" Mid 2 Piece", new twoPiece(swerveDrive, intakeRoller, elevator, "Mid2Piece"));
    autoChooser.addOption("Top 2 Piece", new twoPiece(swerveDrive, intakeRoller, elevator, "Top2Piece"));
    autoChooser.addOption("Drive auto", AutoBuilder.buildAuto("DriveAuto"));
    autoChooser.addOption("Square just drive", AutoBuilder.buildAuto("Square"));
    autoChooser.addOption("Taxi", AutoBuilder.buildAuto("Taxi"));
    // if (paths.contains("S4R3")) {
      // autoChooser.addOption("PreloadTaxi", AutoBuilder.buildAuto("PreloadTaxi"));
      // autoChooser.addOption("PreloadTaxi2", new PreloadTaxi(swerveDrive, List.of(S4R3)));
    // }
    } catch (Exception e) { SmartDashboard.putBoolean("Auto Error", true); }
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(LOG_LEVEL.MINIMAL);  
    if (USE_ELEV) { 
      intakeRoller.initShuffleboard(loggingLevel); 
      elevator.initShuffleboard(loggingLevel);
      coralWrist.initShuffleboard(loggingLevel);
      elevatorPivot.initShuffleboard(loggingLevel);
    }
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