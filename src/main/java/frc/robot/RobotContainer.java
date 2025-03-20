// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SuperSystemConstants.CoralPositions;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.Constants.SuperSystemConstants.AlgaePositions;
import frc.robot.commands.autos.PreloadTaxi;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.autos.TwoPieceOffset;
import frc.robot.commands.autos.Generic2Piece;
import frc.robot.commands.autos.Generic3Piece;
import frc.robot.commands.autos.Generic4Piece;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.SuperSystem.PositionMode;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.BannerSensor;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Pivot;
import frc.robot.util.Controller;

public class RobotContainer {
  public Gyro imu = new PigeonV2(1, ModuleConstants.kCANivoreName);

  public SwerveDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  

  public IntakeRoller intakeRoller;
  public BannerSensor intakeSensor;
  public Elevator elevator;
  public Pivot pivot;
  public Wrist wrist;
  public BannerSensor floorSensor;
  public SuperSystem superSystem;
  public Climb climbMotor;
  public CANdi candi;
  public PositionMode positionMode;

  private final Controller driverController = new Controller(ControllerConstants.kDriverControllerPort,false);
  private final Controller operatorController = new Controller(ControllerConstants.kOperatorControllerPort,false);
  
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  // private Bottom2Piece bottom2Piece;
  public Generic2Piece bottom2Piece;
  public Generic3Piece bottom3Piece;
  public Generic4Piece bottom4Piece;

  private final LOG_LEVEL loggingLevel = LOG_LEVEL.MEDIUM;
  
  static boolean isRedSide = false;
  
  private SwerveJoystickCommand swerveJoystickCommand;
  
  public static boolean USE_SUBSYSTEMS = true;
  
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
    // try {
    //   swerveDrive = new SwerveDrivetrain(imu);
    // } catch (IllegalArgumentException e) {
    //   DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    // }

    if (USE_SUBSYSTEMS) {
      wrist = new Wrist();
      // elevator = new Elevator();
      pivot = new Pivot();
      // intakeRoller = new IntakeRoller();
      // candi = new CANdi(6);
      // climbMotor = new Climb();
      // superSystem = new SuperSystem(elevator, pivot, wrist, intakeRoller, candi, climbMotor);
      try { // ide displayed error fix
        bottom2Piece = new Generic2Piece(swerveDrive, superSystem, "Bottom2Piece", 2, 2);
        bottom3Piece = new Generic3Piece(swerveDrive, superSystem, "Bottom3Piece", 2, 2, 2);
        bottom4Piece = new Generic4Piece(swerveDrive, superSystem, "Bottom4Piece", 2, 2, 2, 2);
      } catch (IOException e) {
        DriverStation.reportError("IOException for Bottom2Piece", e.getStackTrace());
      } catch (ParseException e) {
        DriverStation.reportError("ParseException for Bottom2Piece", e.getStackTrace());
      }
    }

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
      // () -> driverController.getControllerRight(), // robot oriented variable (false = field oriented)
      () -> false, // robot oriented variable (false = field oriented)
      () -> false, // tow supplier
      () -> driverController.getBumperLeft(), //move left of 
      () -> driverController.getBumperRight(), // move right of
      () -> driverController.getTriggerRight(), // Precision/"Sniper Button"
      () -> swerveDrive.getCurrentZoneByPose(),
      () -> false,
      // () -> { return driverController.getButtonRight() || driverController.getButtonDown() || driverController.getButtonUp(); },
      () -> { // Turn To angle Direction | TODO WIP
        // if (driverController.getButtonRight())
        //   if (!IsRedSide())
        //     return 90.0; 
        //   else return 270.0;
        // if (driverController.getButtonDown())
        //    return 180.0;
        // if (driverController.getButtonUp())
        //   return 0.0;
        return swerveDrive.getImu().getHeading();
      }, 
      () -> driverController.getDpadDown() || driverController.getDpadUp() || driverController.getDpadLeft() || driverController.getDpadRight(),
      () -> {

        if (driverController.getDpadDown()) {
          return 180.0;
       } else if (driverController.getDpadLeft()) {
          return 270.0;
        }else if (driverController.getDpadRight()) {
          return 90.0;
        } else if (driverController.getDpadUp()){
          return 0.0;
        } else {
          return -1.0;
        } 



      }
    );

      swerveDrive.setDefaultCommand(swerveJoystickCommand);

      double PIVOT_SPEED = 1;// Degrees per second
        pivot.setDefaultCommand(Commands.run(() -> {
          double rightY = -operatorController.getRightY(); // Left Y (inverted for up = positive)
          if (Math.abs(rightY) > 0.05) {
              double currentAngle = pivot.getPosition();
              pivot.setTargetPosition(currentAngle + (rightY * PIVOT_SPEED * 0.02)); // 20ms loop
          }
      }, pivot));

      double Wrist_SPEED = 2;// Degree  per second
        wrist.setDefaultCommand(Commands.run(() -> {
          double leftX = operatorController.getLeftX(); // leftX (inverted for up = positive)
          if (Math.abs(leftX) > 0.3) {
              double currentRot = wrist.getPosition();
              wrist.setTargetPosition(currentRot + (leftX * Wrist_SPEED * 0.02)); // 20ms loop
          }
      }, wrist));

      

  }

  public void initDefaultCommands_test() {
    initDefaultCommands_teleop();
  }

  public void configureBindings_teleop() {
    ///////////////////////
    // Driver bindings
    //////////////////////
    driverController.controllerLeft().onTrue(
      Commands.runOnce(() -> swerveDrive.zeroGyroAndPoseAngle()) // TODO: When camera pose is implemented, this won't be necessary anymore
    );
    
    // driverController.dpadDown().onTrue(
    //   superSystem.moveTo(NamedPositions.AlgaeL2)
    // );
    // driverController.dpadUp().onTrue(
    //   superSystem.moveTo(NamedPositions.AlgaeL3)
    // );

    //////////////////////
    // Operator bindings
    //////////////////////

    operatorController.dpadUp()
    .onTrue(Commands.runOnce(() -> pivot.setTargetPosition(0.2)));
    operatorController.dpadLeft()
    .onTrue(Commands.runOnce(() -> pivot.setTargetPosition(0.1)));
    operatorController.dpadDown()
    .onTrue(Commands.runOnce(() -> pivot.setTargetPosition(0.05)));

    operatorController.buttonUp()
      .onTrue(Commands.runOnce(() -> wrist.setTargetPosition(-0.2)));
    operatorController.buttonLeft()
    .onTrue(Commands.runOnce(() -> wrist.setTargetPosition(-0.35)));
    operatorController.buttonDown()
      .onTrue(Commands.runOnce(() -> wrist.setTargetPosition(-0.5)));
    

    // operatorController.dpadDown()
    // .onTrue(superSystem.moveTo(NamedPositions.L1));
    // operatorController.dpadLeft()
    // .onTrue(superSystem.moveTo(NamedPositions.L2));
    // operatorController.dpadUp()
    // .onTrue(superSystem.moveTo(NamedPositions.L3));
    // operatorController.dpadRight()
    // .onTrue(superSystem.moveTo(NamedPositions.L4));
    
    // operatorController.triggerRight()
    //   .onTrue(superSystem.intake())
    //   .onFalse(superSystem.holdPiece());
    // operatorController.triggerLeft()
    //   .onTrue(superSystem.moveTo(NamedPositions.GroundIntake));
    
    // operatorController.controllerRight()
    // .onTrue(superSystem.moveTo(NamedPositions.Processor));

    // operatorController.buttonUp()
    //   .onTrue(superSystem.moveTo(NamedPositions.Station));
    // operatorController.buttonRight()
    //   .onTrue(superSystem.moveTo(NamedPositions.SemiStow));
    // operatorController.buttonDown()
    //   .onTrue(superSystem.moveTo(NamedPositions.Stow));


  }


  public void configureBindings_test() {
    // driverController.buttonDown()
    // .onTrue(superSystem.moveTo(NamedPositions.GroundIntake));

    // //////////////////////////
    // /// DO NOT REMOVE IT
    operatorController.controllerLeft()
      .onTrue(superSystem.zeroEncoders());
    // ////////////////////////
    
    // operatorController.controllerRight()
    // .onTrue(superSystem.moveTo(NamedPositions.Processor));    

    // operatorController.dpadDown()
    // .onTrue(superSystem.moveTo(NamedPositions.L1));
    // operatorController.dpadLeft()
    // .onTrue(superSystem.moveTo(NamedPositions.L2));
    // operatorController.dpadUp()
    // .onTrue(superSystem.moveTo(NamedPositions.L3));
    // operatorController.dpadRight()
    // .onTrue(superSystem.moveTo(NamedPositions.L4));
    

    // operatorController.triggerRight()
    //   .onTrue(superSystem.intake())
    //   .onFalse(superSystem.holdPiece());
    // operatorController.triggerLeft()
    //   .onTrue(superSystem.outtake())
    //   .onFalse(superSystem.stopRoller());

    // operatorController.buttonUp()
    //   .onTrue(superSystem.moveTo(NamedPositions.Station));
    // operatorController.buttonRight()
    //   .onTrue(superSystem.moveTo(NamedPositions.GroundIntake));
    // operatorController.buttonDown()
    //   .onTrue(superSystem.moveTo(NamedPositions.Stow));
    // operatorController.buttonLeft()
    //   .onTrue(superSystem.moveTo(NamedPositions.SemiStow));
    
    // driverController.buttonUp()
    // .onTrue(superSystem.climbCommandUp());

    // driverController.buttonDown()
    // .onTrue(superSystem.climbCommandDown());
    

    // operatorController.triggerRight()
    // .onTrue(superSystem.moveToSemiStow());
   }
  
  private void initAutoChoosers() {

    // try { // ide displayed error fix
      // bottom2Piece = new Bottom2Piece(swerveDrive, intakeRoller, elevator, "Bottom2Piece");
      // pathOnlyBottom2Piece = new PathOnlyBottom2Piece(swerveDrive, "PathOnlyBottom2Piece");
    // } catch (Exception e) {
    //   DriverStation.reportError("IOException for Bottom2Piece", e.getStackTrace());
    //   return;
    // } 

    try { // fix for vendordeps not importing
    // PathPlannerPath S4R3 = PathPlannerPath.fromPathFile("S4R3");

  	List<String> paths = AutoBuilder.getAllAutoNames();
    
    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");
    autosTab.add("Selected Auto", autoChooser);
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    
    autoChooser.addOption("PreloadTaxi", new PreloadTaxi(swerveDrive, "TaxiPreload", superSystem));
    // autoChooser.addOption("PreloadTaxi", new PreloadTaxiAutoMove(swerveDrive, "TaxiPreload", superSystem));
    autoChooser.addOption("TaxiMid", AutoBuilder.buildAuto("TaxiPreload"));
    autoChooser.addOption("TaxiLeft", AutoBuilder.buildAuto("S1Taxi"));
    autoChooser.addOption("TaxiRight", AutoBuilder.buildAuto("S7Taxi"));
    
    autoChooser.addOption("2PieceLeftOffset", new TwoPieceOffset(swerveDrive, "TopTwoPieceOffset", superSystem));
    // autoChooser.addOption("2PieceLeft", new TwoPiece(swerveDrive, "TopTwoPiece", superSystem));
    autoChooser.addOption("2PieceRightOffset", new TwoPieceOffset(swerveDrive, "BottomTwoPieceOffset", superSystem));
    // autoChooser.addOption("2PieceRight", new TwoPiece(swerveDrive, "BottomTwoPiece", superSystem));
    
    // autoChooser.addOption("2PiecePathOnly", new TwoPiecePath(swerveDrive, "TopTwoPiece", superSystem));
    
    // autoChooser.addOption("Bottom 3 Piece", bottom3Piece);
    // autoChooser.addOption("Bottom 4 Piece", bottom4Piece);
    

    } catch (Exception e) { SmartDashboard.putBoolean("Auto Error", true); }
  }
  
  public void initShuffleboard() {
    // imu.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(LOG_LEVEL.MINIMAL);  
    if (USE_SUBSYSTEMS) { 
      intakeRoller.initShuffleboard(loggingLevel); 
      elevator.initShuffleboard(loggingLevel);
      wrist.initShuffleboard(loggingLevel);
      pivot.initShuffleboard(loggingLevel);
      climbMotor.initShuffleboard(loggingLevel);
      superSystem.initShuffleboard(loggingLevel);
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

  public void DisableAllMotors_Test()
  {
    elevator.stopMotion();
    pivot.stopMotion();
    wrist.stopMotion();

    pivot.setEnabled(false);
    elevator.setEnabled(false);
    wrist.setEnabled(false);
    intakeRoller.setEnabled(false);
    climbMotor.setEnabled(false);
    swerveDrive.setBreak(true);
  }
  

  
}