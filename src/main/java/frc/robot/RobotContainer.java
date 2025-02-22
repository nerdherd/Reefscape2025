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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.SetArmPosition;
// import frc.robot.commands.autos.PreloadTaxi;
// import frc.robot.commands.autSquare;
import frc.robot.commands.SwerveJoystickCommand;
// import frc.robot.commands.autos.AutoDriving;
import frc.robot.commands.autos.Bottom2Piece;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeWristCopy;
import frc.robot.subsystems.ElevatorPivot;

import frc.robot.util.Controller;

public class RobotContainer {
  public Gyro imu = new PigeonV2(1, ModuleConstants.kCANivoreName);

  public SwerveDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  public IntakeRoller intakeRoller;
  public Elevator elevator;
  public ElevatorPivot elevatorPivot;
  public IntakeWristCopy intakeWristCopy;

  public Bottom2Piece bottom2Piece;

  private final Controller driverController = new Controller(ControllerConstants.kDriverControllerPort);
  private final Controller operatorController = new Controller(ControllerConstants.kOperatorControllerPort);
  
  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;
  
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  
  static boolean isRedSide = false;
  
  private SwerveJoystickCommand swerveJoystickCommand;
  
  private static boolean USE_ELEV = true;
  private static boolean V1 = true;


  // For logging wrist
  public final VoltageOut voltageRequest = new VoltageOut(0);
  public double voltage = 0;
  public double desiredAngle = 90.0; //164, 99.8
  public double desiredRotation = -0.19;

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

    if (USE_ELEV) {
      intakeRoller = new IntakeRoller();
      intakeWristCopy = new IntakeWristCopy();
      elevator = new Elevator();
      elevatorPivot = new ElevatorPivot(V1);
    }

    try { // ide displayed error fix
      bottom2Piece = new Bottom2Piece(swerveDrive, intakeRoller, elevator, "Bottom2Piece");
    } catch (IOException e) {
      DriverStation.reportError("IOException for Bottom2Piece", e.getStackTrace());
    } catch (ParseException e) {
      DriverStation.reportError("ParseException for Bottom2Piece", e.getStackTrace());
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

  private double manualVoltage = 0.0; // Start at 0V

  public void configureBindings_teleop() {
    // Driver bindings
    driverController.controllerLeft().onTrue(
      Commands.runOnce(() -> swerveDrive.zeroGyroAndPoseAngle()) // TODO: When camera pose is implemented, this won't be necessary anymore
    );

    // TODO someone tell me how useful this is
    // try {
    //   driverController.triggerLeft()
    //     .whileTrue(new AutoDriving(swerveDrive, "Bottom2Piece"))
    //     .onFalse(AutoDriving.stopDriving(algaeRoller, coralWrist, elevator, elevatorPivot));
    // } catch (IOException e) { DriverStation.reportError("Auto Driving IOException for Left Trigger", e.getStackTrace());
    // } catch (ParseException e) { DriverStation.reportError("Auto Driving ParseException for Left Trigger", e.getStackTrace()); }
    
    // driverController.triggerRight()
    //   .whileTrue(bottom2Piece.runAuto())
    //   .onFalse(bottom2Piece.stopAuto());
    
    // driverController.buttonUp()
    //   .onTrue(IntakeWristCopy.moveToStation())
    //   .onFalse(IntakeWristCopy.moveToStow());
    // driverController.buttonLeft()
    //   .onTrue(IntakeWristCopy.setEnabledCommand());
    // driverController.buttonRight()
    //   .onTrue(IntakeWristCopy.setDisabledCommand());
    // driverController.buttonUp()
    //   .onTrue(intakeRoller.intakeLeft())
    //   .onFalse(intakeRoller.stop());
    // driverController.buttonDown()
    //   .onTrue(intakeRoller.outtake())
    //   .onFalse(intakeRoller.stop());
    // driverController.buttonLeft()
    //   .onTrue(intakeRoller.intake())
    //   .onFalse(intakeRoller.stop());


    // driverController.bumperRight()
    //   .whileTrue(Commands.run(() -> {
    //     desiredRotation -= 1.0 / 50.0; // 2 degrees per second ish
    //     // voltage = -1.928;
    // }));

    // driverController.bumperLeft()
    //   .whileTrue(Commands.run(() -> {
    //     desiredRotation += 1.0 / 50.0; // 2 degrees per second ish
    //     // voltage = -1.928;
    // }));

    
    // driverController.bumperRight()
    //   .whileTrue(IntakeWristCopy.moveToStow()
    // );
    // driverController.triggerRight()
    //   .whileTrue(IntakeWristCopy.moveToStation()
    // );
    // driverController.bumperLeft()
    //   .whileTrue(IntakeWristCopy.moveToReefL14()
    // );
    // driverController.triggerLeft()
    //   .whileTrue(IntakeWristCopy.moveToReefL23()
    // );

    // driverController.buttonUp()
    //   .whileTrue(IntakeWristCopy.setEnabledCommand()
    // );
      operatorController.dpadDown().onTrue(new SetArmPosition(intakeWristCopy, 0.0));
      operatorController.triggerRight().onTrue(new SetArmPosition(intakeWristCopy, 45.0));
      operatorController.bumperRight().onTrue(new SetArmPosition(intakeWristCopy, 90.0));
      operatorController.triggerLeft().onTrue(new SetArmPosition(intakeWristCopy, 135.0));
      operatorController.bumperLeft().onTrue(new SetArmPosition(intakeWristCopy, 180.0));
      operatorController.dpadUp().onTrue(new SetArmPosition(intakeWristCopy, 210.0));

      operatorController.controllerLeft().onTrue(new InstantCommand(() -> intakeWristCopy.resetEncoder(), intakeWristCopy));

      operatorController.buttonUp().onTrue(new InstantCommand(() -> {
        manualVoltage -= 0.1;
        if (manualVoltage < -2) {
            manualVoltage = -1.8; // Reset to a V if exceeds max
        }
        intakeWristCopy.setArmVoltage(manualVoltage);
        SmartDashboard.putNumber("Manual Voltage", manualVoltage); // Display voltage
      }, intakeWristCopy));

      operatorController.buttonDown().onTrue(new InstantCommand(() -> {
        manualVoltage += 0.05;
        if (manualVoltage > 0) {
            manualVoltage = -0.1; // Reset to b V if exceeds min
        }
        intakeWristCopy.setArmVoltage(manualVoltage);
        SmartDashboard.putNumber("Manual Voltage", manualVoltage); // Display voltage
      }, intakeWristCopy));


    // driverController.buttonUp()
    //   .whileTrue(
    //     Commands.parallel(
    //       // Commands.run(() -> IntakeWristCopy.setPositionDegrees(desiredAngle)),
    //       Commands.run(() -> IntakeWristCopy.setPosition(desiredRotation)),
    //       IntakeWristCopy.setEnabledCommand()
    //   ))
    //   .onFalse(IntakeWristCopy.setDisabledCommand()
    //   );

    driverController.buttonDown()
      .onTrue(Commands.runOnce(() -> {
        desiredRotation = -0.19;
        // desiredAngle = 90.0; //Top: -85.4
      }));
  
    // driverController.buttonUp()
    //   .onTrue(IntakeWristCopy.setEnabledCommand())
    //   .onFalse(IntakeWristCopy.setDisabledCommand());
    
    // driverController.buttonLeft()
    //   .onTrue(IntakeWristCopy.moveToReefL14())
    //   .onFalse(IntakeWristCopy.moveToStow());


    
    // driverController.buttonRight()
    //   .onTrue(IntakeWristCopy.moveToReefL23())
    //   .onFalse(IntakeWristCopy.setDisabledCommand()); 
    
    if(USE_ELEV) {
      // driverController.triggerRight()
      // .onTrue(elevatorPivot.moveToPickup()) // hold it :)
      // .onFalse(elevatorPivot.moveToStow());
    }
  }

  public void configureBindings_test() {
    // driverController.buttonRight()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button A Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button A Test", "bye")));
    // driverController.buttonDown()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button B Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button B Test", "bye")));
    // driverController.buttonUp()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button X Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button X Test", "bye")));
    // driverController.buttonLeft()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Y Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Y Test", "bye")));

    // driverController.bumperLeft()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Bumper L Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Bumper L Test", "bye")));
    // driverController.bumperRight()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Bumper R Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Bumper R Test", "bye")));
    // driverController.triggerLeft()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Trigger ZL Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Trigger ZL Test", "bye")));
    // driverController.triggerRight()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Trigger ZR Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Trigger ZR Test", "bye")));

    // driverController.dpadUp()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Up Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Up Test", "bye")));
    // driverController.dpadRight()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Right Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Right Test", "bye")));
    // driverController.dpadDown()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Down Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Down Test", "bye")));
    // driverController.dpadLeft()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Left Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Left Test", "bye")));

    // driverController.controllerLeft()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Minus Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Minus Test", "bye")));
    // driverController.controllerRight()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Plus Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Plus Test", "bye")));
    // driverController.joystickLeft()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Left Joy Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Left Joy Test", "bye")));
    // driverController.joystickRight()
    //   .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Right Joy Test", "hi")))
    //   .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Right Joy Test", "bye")));


    // driverController.bumperRight()
    //   .whileTrue(Commands.run(() -> {
    //     desiredAngle -= 1 / 50; // 1 degree per second ish
    //     // voltage = -1.928;
    //   }));
    // driverController.bumperLeft()
    //   .whileTrue(Commands.run(() -> {
    //     intakeWristCopy.setPositionDegrees(desiredAngle);
    //   }));
    // driverController.buttonDown()
    //   .onTrue(Commands.runOnce(() -> {
    //     desiredAngle = 90.0; //Top: -85.4
    //   }));


      // driverController.bumperRight()
      // .whileTrue(Commands.run(() -> {
      //   voltage += 0.2 / 50.0;
      //   // voltage = -1.928;
      // }));
      // driverController.bumperLeft()
      //   .whileTrue(Commands.run(() -> {
      //     IntakeWristCopy.getMotor().setControl(voltageRequest.withOutput(voltage));
      // }));
      // driverController.buttonDown()
      //   .onTrue(Commands.runOnce(() -> {
      //     voltage = 0;
      // }));
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
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(LOG_LEVEL.MINIMAL);  
    if (USE_ELEV) { 
      intakeRoller.initShuffleboard(loggingLevel); 
      elevator.initShuffleboard(loggingLevel);
      //intakeWristCopy.initShuffleboard(loggingLevel);
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