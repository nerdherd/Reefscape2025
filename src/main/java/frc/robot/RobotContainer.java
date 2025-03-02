// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.controls.VoltageOut;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.StationCommand;
import frc.robot.commands.StowCommand;
import frc.robot.commands.SuperSystemCommand;
import frc.robot.Constants.V1ElevatorConstants;
import frc.robot.Constants.WristConstants;
// import frc.robot.commands.autos.PreloadTaxi;
// import frc.robot.commands.autSquare;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.autos.Generic2Piece;
import frc.robot.commands.autos.Generic3Piece;
import frc.robot.commands.autos.Generic4Piece;
import frc.robot.commands.autos.Bottom2Piece;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeV2;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.ElevatorPivot;

import frc.robot.util.Controller;
import frc.robot.util.filters.OldDriverFilter2;

public class RobotContainer {
  public Gyro imu = new PigeonV2(1, ModuleConstants.kCANivoreName);

  public SwerveDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  public IntakeV2 intakeV2;
  // public IntakeRoller intakeRoller;
  public Elevator elevator;
  public ElevatorPivot elevatorPivot;
  public IntakeWrist intakeWrist;
  public SuperSystem superSystem;

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  public Generic2Piece bottom2Piece;
  public Generic3Piece bottom3Piece;
  public Generic4Piece bottom4Piece;
  public Bottom2Piece bottom2Piece;


  private final Controller driverController = new Controller(ControllerConstants.kDriverControllerPort);
  private final Controller operatorController = new Controller(ControllerConstants.kOperatorControllerPort);
  
  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;
  
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  
  static boolean isRedSide = false;
  
  private SwerveJoystickCommand swerveJoystickCommand;
  
  private static boolean USE_SUBSYSTEMS = true;
  private static boolean USE_ELEV = false;
  private static boolean USE_WRIST = false;
  private static boolean USE_PIVOT = false;
  private static boolean USE_INTAKE = false;
  private static boolean V1 = true;
  
  
  // For logging wrist
  public final VoltageOut voltageRequest = new VoltageOut(0);
  public double voltage = 0;
  public double desiredAngle = 0.0; //164, 99.8

  public double desiredRotation = 0.0;//ElevatorConstants.kElevatorPivotStowPosition; -1.6

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

    if (USE_SUBSYSTEMS) {
      // intake = new IntakeV2();
      intakeWrist = new IntakeWrist(V1);
      elevator = new Elevator();
      elevatorPivot = new ElevatorPivot(V1);
      intakeV2 = new IntakeV2();
      superSystem = new SuperSystem(elevator, elevatorPivot, intakeWrist, intakeV2);
    }

    // intakeWrist.setEnabledCommand(USE_WRIST);
    // elevator.setEnabledCommand(USE_ELEV);
    // elevatorPivot.setEnabledCommand(USE_PIVOT);
    // intakeV2.setEnabledCommand(USE_INTAKE);

/*  TODO: Fix Bottom2Piece to take in IntakeV2
    try { // ide displayed error fix
        bottom2Piece = new Generic2Piece(swerveDrive, intakeRoller, elevator, "Bottom2Piece");
        bottom3Piece = new Generic3Piece(swerveDrive, intakeRoller, elevator, "Bottom3Piece");
        bottom4Piece = new Generic4Piece(swerveDrive, intakeRoller, elevator, "Bottom4Piece");
    } catch (IOException e) {
      DriverStation.reportError("IOException for Bottom2Piece", e.getStackTrace());
    } catch (ParseException e) {
      DriverStation.reportError("ParseException for Bottom2Piece", e.getStackTrace());
    }
*/
    initShuffleboard();
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
  public Command getAutonomousCommand() {
    Command currentAuto = autoChooser.getSelected();
    
    swerveDrive.setDriveMode(DRIVE_MODE.FIELD_ORIENTED);//AUTONOMOUS);
    return currentAuto;
  }

  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(LOG_LEVEL.MINIMAL);  
    if (USE_ELEV) { 
      intakeRoller.initShuffleboard(loggingLevel); 
      elevator.initShuffleboard(loggingLevel);
      wrist.initShuffleboard(loggingLevel);
      elevatorPivot.initShuffleboard(loggingLevel);
    }
  }
  
  private void initAutoChoosers() {
    try { // fix for vendordeps not importing
    // PathPlannerPath S4R3 = PathPlannerPath.fromPathFile("S4R3");
  	// List<String> paths = AutoBuilder.getAllAutoNames();
    
    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");
    autosTab.add("Selected Auto", autoChooser);

    autoChooser.setDefaultOption("Bottom 2 Piece", bottom2Piece);
    autoChooser.addOption("Bottom 3 Piece", bottom3Piece);
    autoChooser.addOption("Bottom 4 Piece", bottom4Piece);

    autoChooser.addOption("Square just drive", AutoBuilder.buildAuto("Square"));
    autoChooser.addOption("Taxi", AutoBuilder.buildAuto("Taxi"));
    // if (paths.contains("S4R3")) {
      // autoChooser.addOption("PreloadTaxi", AutoBuilder.buildAuto("PreloadTaxi"));
      // autoChooser.addOption("PreloadTaxi2", new PreloadTaxi(swerveDrive, List.of(S4R3)));
    // }
    } catch (Exception e) { SmartDashboard.putBoolean("Auto Error", true); }
  }

  /**
   * Teleop commands configuration 
   * used in teleop mode.
   */
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

    operatorController.controllerLeft()
      .onTrue(superSystem.moveToCage());

      operatorController.controllerRight()
      .onTrue(superSystem.moveToNet());
    
      operatorController.buttonUp()
      .whileTrue(superSystem.moveToStation());
      
      operatorController.buttonLeft()
      .onTrue(superSystem.moveToSemiStow());
  
      operatorController.buttonRight()
      .onTrue(superSystem.moveToProcs());

      operatorController.buttonDown()
      .onTrue(superSystem.moveToStow());

      operatorController.triggerLeft()
    .whileTrue(Commands.run(() -> {
      desiredRotation += 0.001;
      intakeV2.setJawPosition(desiredRotation);
    }))
    .onFalse(intakeV2.stopJawCommand());

    operatorController.triggerRight()
    .whileTrue(Commands.run(() -> {
      desiredRotation -= 0.001;
      intakeV2.setJawPosition(desiredRotation);
    }))
    .onFalse(intakeV2.stopJawCommand());

    //TODO for roller operatorController.bumperLeftRight();
    //TODO for roller operatorController.bumperLeftRight();


  }

  boolean keylock_crossup = false;
  boolean keylock_crossleft = false;

  boolean keylock_crossupleft = false;

  boolean keylock_crossdown = false;
  boolean keylock_crossright = false;

  boolean keylock_crossdownright = false;


  public void operatorController_teleop() {
    OldDriverFilter2 xFilter = new OldDriverFilter2(
                0.05,//ControllerConstants.kDeadband,
                0.05,//kMinimumMotorOutput,
                5,//kTeleDriveMaxSpeedMetersPerSecond,
                0.11765,//kDriveAlpha,
                5,//kTeleMaxAcceleration,
                -5);//kTeleMaxDeceleration);
    
    OldDriverFilter2 yFilter = new OldDriverFilter2(
                0.05,//ControllerConstants.kDeadband,
                0.05,//kMinimumMotorOutput,
                5,//kTeleDriveMaxSpeedMetersPerSecond,
                0.11765,//kDriveAlpha,
                5,//kTeleMaxAcceleration,
                -5);//kTeleMaxDeceleration);

    // TODO pivot ctrl operatorController.getLeftX();
    // TODO wrist ctrl operatorController.getLeftY();
    // TODO elevator ctrl operatorController.getLeftX();
    // TODO intake ctrl operatorController.getLeftY();
    double pivotPosition = yFilter.calculate(operatorController.getLeftY());
    double elevatorPosition = xFilter.calculate(operatorController.getLeftX());

    SmartDashboard.putNumber("Operator L-Y", pivotPosition);
    SmartDashboard.putNumber("Operator L-X", elevatorPosition);


    boolean upPressed = operatorController.getDpadUp();
    boolean leftPressed = operatorController.getDpadLeft();

    if( leftPressed && upPressed)
    {
      if (!keylock_crossupleft)
      {
        keylock_crossupleft = true;
        Commands.run(()->superSystem.moveToL1()); // TODO: change to mid high
      }      
    }
    else
    {
      keylock_crossupleft = false;

      if (upPressed)
      {
          if (!keylock_crossup)
          {
              keylock_crossup = true;
              Commands.run(()->superSystem.moveToL4());
          }
      }
      else
      {
          keylock_crossup = false;
      }

      if (leftPressed)
      {
          if (!keylock_crossleft)
          {
              keylock_crossleft = true;
              Commands.run(()->superSystem.moveToL2());
          }
      }
      else
      {
          keylock_crossleft = false;
      }
    }


    boolean downPressed = operatorController.getDpadDown();
    boolean rightPressed = operatorController.getDpadRight();

    if( rightPressed && downPressed)
    {
      if (!keylock_crossdownright)
      {
        keylock_crossdownright = true;
        Commands.run(()->superSystem.moveToL4()); // TODO: change to mid low
      }      
    }
    else
    {
      keylock_crossdownright = false;

      if (downPressed)
      {
          if (!keylock_crossdown)
          {
              keylock_crossdown = true;
              Commands.run(()->superSystem.moveToL1());
          }
      }
      else
      {
          keylock_crossdown = false;
      }

      if (rightPressed)
      {
          if (!keylock_crossright)
          {
              keylock_crossright = true;
              Commands.run(()->superSystem.moveToL3());
          }
      }
      else
      {
          keylock_crossright = false;
      }
    }


  }

  public void configureBindings_test() {
    operatorController.controllerLeft()
    .onTrue(superSystem.zeroEncoders());
    // driverController.bumperRight()
    //   .whileTrue(Commands.run(() -> {
    //     desiredAngle -= 1 / 50; // 1 degree per second ish
    //     // voltage = -1.928;
    //   }));
    // driverController.bumperLeft()
    //   .whileTrue(Commands.run(() -> {
    //     intakeWrist.setPositionDegrees(desiredAngle);
    //   }));
    // driverController.buttonDown()
    //   .onTrue(Commands.runOnce(() -> {
    //     desiredAngle = 90.0; //Top: -85.4
    //   }));

    // driverController.triggerLeft()
    // .whileTrue(elevator.setEnabledCommand(true))
    // .onFalse(elevator.setEnabledCommand(false));

    // driverController.buttonUp()
    // .whileTrue(elevator.moveToReefL1())
    // .onFalse(elevator.stow());

    // driverController.buttonLeft()
    // .whileTrue(elevator.moveToReefL2())
    // .onFalse(elevator.stow());

    // driverController.buttonRight()
    // .whileTrue(elevator.moveToReefL3())
    // .onFalse(elevator.stow());

    // driverController.buttonDown()
    // .whileTrue(elevator.moveToReefL4())
    // .onFalse(elevator.stow());
      
    // driverController.bumperLeft();
    

    operatorController.buttonUp()
    .whileTrue(superSystem.moveToStation());
    operatorController.buttonRight()
    .whileTrue(superSystem.moveToL2());
    operatorController.buttonLeft()
    .onTrue(superSystem.stop());

    // operatorController.buttonRight()
    // .onTrue(intakeV2.intakeCoral());

    operatorController.dpadUp()
    .onTrue(intakeV2.outtakeAlgae());

    operatorController.dpadDown()
    .onTrue(intakeV2.outtakeCoral());

    operatorController.dpadRight()
    .onTrue(intakeV2.setEnabledCommand(false));

    

    // operatorController.buttonDown()
    // .whileTrue(superSystem.moveToL2());

    // operatorController.triggerLeft()
    // .whileTrue(Commands.run(() -> {
    //   desiredRotation += 0.001;
    //   intakeV2.setEnabled(true);
    //   intakeV2.setPosition(desiredRotation);
    // }))
    // .onFalse(elevatorPivot.setEnabledCommand(false));
    // operatorController.buttonLeft().onTrue(superSystem.moveTogroundIntake());

  }
  
  private void initAutoChoosers() {
    try { // fix for vendordeps not importing
    PathPlannerPath S4R3 = PathPlannerPath.fromPathFile("S4R3");

  	List<String> paths = AutoBuilder.getAllAutoNames();
    
    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
    autoChooser.setDefaultOption("Bottom 2 Piece", bottom2Piece);
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
    // swerveDrive.initShuffleboard(loggingLevel);
    // swerveDrive.initModuleShuffleboard(LOG_LEVEL.MINIMAL);  
    if (USE_SUBSYSTEMS) { 
      // intakeRoller.initShuffleboard(loggingLevel); 
      elevator.initShuffleboard(loggingLevel);
      intakeWrist.initShuffleboard(loggingLevel);
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