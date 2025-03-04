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
import frc.robot.Constants.V1ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.SuperSystemConstants.NamedPositions;
// import frc.robot.commands.autos.PreloadTaxi;
// import frc.robot.commands.autSquare;
import frc.robot.commands.SwerveJoystickCommand;
// import frc.robot.commands.autos.AutoDriving;
import frc.robot.commands.autos.Bottom2Piece;
import frc.robot.commands.autos.PathOnlyBottom2Piece;
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

  private Bottom2Piece bottom2Piece;
  private PathOnlyBottom2Piece pathOnlyBottom2Piece;


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
      elevatorPivot = new ElevatorPivot();
      intakeV2 = new IntakeV2();
      superSystem = new SuperSystem(elevator, elevatorPivot, intakeWrist, intakeV2);
    }

    // intakeWrist.setEnabledCommand(USE_WRIST);
    // elevator.setEnabledCommand(USE_ELEV);
    // elevatorPivot.setEnabledCommand(USE_PIVOT);
    // intakeV2.setEnabledCommand(USE_INTAKE);

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

  public void initDefaultCommands_teleop() {
    swerveJoystickCommand = 
    new SwerveJoystickCommand(
      swerveDrive,
      () -> -driverController.getLeftY(), // Horizontal translation
      () -> driverController.getLeftX(), // Vertical Translation
      () -> driverController.getRightX(), // Rotation
      () -> false, // robot oriented variable (false = field oriented)
      () -> false, // tow supplier
      () -> driverController.getBumperLeft(),
      () -> driverController.getBumperRight(),
      () -> driverController.getTriggerLeft(), // Precision/"Sniper Button"
      () -> swerveDrive.getCurrentZoneByPose(),
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

    double PIVOT_SPEED = 10;// Degrees per second
    elevatorPivot.setDefaultCommand(Commands.run(() -> {
      double leftY = -operatorController.getLeftY(); // Left Y (inverted for up = positive)
      if (Math.abs(leftY) > 0.05) {
          double currentAngle = elevatorPivot.getPosition();
          elevatorPivot.setTargetPosition(currentAngle + (leftY * PIVOT_SPEED * 0.02)); // 20ms loop
      }
  }, elevatorPivot));

  double Wrist_SPEED = 20;// Degree  per second
    intakeWrist.setDefaultCommand(Commands.run(() -> {
      double leftX = -operatorController.getLeftX(); // leftX (inverted for up = positive)
      if (Math.abs(leftX) > 0.05) {
          double currentRot = intakeWrist.getPosition();
          intakeWrist.setTargetPosition(currentRot + (leftX * Wrist_SPEED * 0.02)); // 20ms loop
      }
  }, intakeWrist));

  double Elevator_SPEED = 0.3;// Meters per second
    elevator.setDefaultCommand(Commands.run(() -> {
      double rightY = -operatorController.getRightY(); // rightY Y (inverted for up = positive)
      if (Math.abs(rightY) > 0.05) {
          double currentAngle = elevator.getPosition();
          elevator.setTargetPosition(currentAngle + (rightY * Elevator_SPEED * 0.02)); // 20ms loop
      }
  }, elevator));  

}

  public void initDefaultCommands_test() {}

  public void configureBindings_teleop() {
    ///////////////////////
    // Driver bindings
    //////////////////////
    driverController.controllerLeft().onTrue(
      Commands.runOnce(() -> swerveDrive.zeroGyroAndPoseAngle()) // TODO: When camera pose is implemented, this won't be necessary anymore
    );

    driverController.controllerRight()
    .onTrue(superSystem.moveToStow()); // todo remove it

    driverController.dpadUp()
    .onTrue(superSystem.moveToL4());

    driverController.dpadRight()
    .onTrue(superSystem.moveToL3());

    driverController.dpadLeft()
    .onTrue(superSystem.moveToL2());

    driverController.dpadDown()
    .onTrue(superSystem.moveToL1());

    driverController.triggerLeft()
    .onTrue(superSystem.outtakeCoral());
    //.onFalse(intakeV2.stopRollerCommand());// TODO

    driverController.bumperLeft()
    .onTrue(superSystem.outtakeAlgae());
    //.onFalse(intakeV2.stopRollerCommand());// TODO

    //////////////////////
    // operator bindings
    //////////////////////

    operatorController.dpadUp()
      .onTrue(superSystem.moveToNet());

    operatorController.dpadRight()
    .onTrue(superSystem.moveToL3L4());

    operatorController.dpadLeft()
    .onTrue(superSystem.moveToL2L3());

    operatorController.dpadDown()
    .onTrue(superSystem.moveToProcessor());

    operatorController.controllerRight()
      .onTrue(superSystem.moveToCage()); 
    
      operatorController.buttonUp()
      .whileTrue(superSystem.moveToStation());
      
      operatorController.buttonRight()
      .onTrue(superSystem.moveToSemiStow());

      operatorController.buttonDown()
      .onTrue(superSystem.moveToStow());

      
      operatorController.buttonLeft()
      .onTrue(superSystem.moveToGroundIntake());


      ////////
      /// Many things need to be done!!!
      operatorController.triggerRight()
      .onTrue(superSystem.intakeCoral());
      // .onFalse(intakeV2.stopClawCommand()); // TODO

    // todo: use a trigger to do all motors' position and intake algae actions
    //(intake from reef, and from ground)
    operatorController.bumperRight()
    .onTrue(superSystem.intakeAlgae());
    // .onFalse(intakeV2.stopClawCommand()); // TODO
  }


  public void configureBindings_test() {
    //////////////////////////
    /// DO NOT REMOVE IT
    operatorController.controllerLeft()
    .onTrue(superSystem.zeroEncoders());

    
    ////////////////////////
    
    operatorController.controllerRight()
    .onTrue(superSystem.stop());    

    operatorController.dpadDown()
    .onTrue(superSystem.moveToL1());
    operatorController.dpadLeft()
    .onTrue(superSystem.moveToL2());
    operatorController.dpadUp()
    
    .onTrue(superSystem.moveToL3());
    operatorController.dpadRight()
    .onTrue(superSystem.moveToGroundIntake());
    operatorController.triggerRight().onTrue(superSystem.intakeCoral());
    operatorController.triggerLeft().onTrue(superSystem.outtakeCoral());
    operatorController.bumperRight().onTrue(superSystem.stopRoller());
    operatorController.bumperLeft().onTrue(superSystem.intakeStation());

    operatorController.buttonUp()
    .onTrue(superSystem.moveToStation());
    operatorController.buttonDown()
    .onTrue(superSystem.moveToStow());
    // operatorController.buttonRight()
    // .onTrue(superSystem.moveToSemiStow());
    
    // operatorController.bumperLeft()
    // .onTrue(superSystem.intakeCoral());
    // operatorController.triggerLeft()
    // .onTrue(superSystem.outtakeCoral());
    // operatorController.bumperRight()
    // .onTrue(superSystem.intakeAlgae());
    // operatorController.triggerRight()
    // .onTrue(superSystem.outtakeAlgae());

    // operatorController.buttonLeft()
    
    // .onTrue(superSystem.stopRoller());
    // operatorController.buttonRight()
    // .onTrue(superSystem.closeClaw());

    driverController.buttonDown()
    .onTrue(superSystem.moveToGroundIntake());
  //   operatorController.triggerRight()
  //   .onTrue(superSystem.moveToSemiStow());
   }
  
  private void initAutoChoosers() {

    try { // ide displayed error fix
      bottom2Piece = new Bottom2Piece(swerveDrive, intakeV2, elevator, "Bottom2Piece");
      // pathOnlyBottom2Piece = new PathOnlyBottom2Piece(swerveDrive, "PathOnlyBottom2Piece");
    } catch (Exception e) {
      DriverStation.reportError("IOException for Bottom2Piece", e.getStackTrace());
      return;
    } 

    try { // fix for vendordeps not importing
    PathPlannerPath S4R3 = PathPlannerPath.fromPathFile("S4R3");

  	List<String> paths = AutoBuilder.getAllAutoNames();
    
    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
    autoChooser.addOption("Square just drive", AutoBuilder.buildAuto("Square"));
    autoChooser.addOption("Taxi", AutoBuilder.buildAuto("Taxi"));
    // autoChooser.addOption("Path Only Bottom 2 Piece", pathOnlyBottom2Piece);
    autoChooser.addOption("test", AutoBuilder.buildAuto("Taxi") );
    autoChooser.addOption("twopieceauto", AutoBuilder.buildAuto("Bottom2Piece"));
    // if (paths.contains("S4R3")) {, 
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
      intakeV2.initShuffleboard(loggingLevel);
    }
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command currentAuto = autoChooser.getSelected();
    
    swerveDrive.setDriveMode(DRIVE_MODE.FIELD_ORIENTED);
    return currentAuto;
  }


  public void refreshSupersystem() {
    // TODO add for Wrist and possibly pivot
    elevator.setPivotAngle(0);


    elevatorPivot.setTargetPosition(0);
    elevator.setTargetPosition(0);
    intakeWrist.setTargetPosition(0);

    elevatorPivot.setEnabled(true);
    elevator.setEnabled(true);
    intakeWrist.setEnabled(true);
  }

  
}