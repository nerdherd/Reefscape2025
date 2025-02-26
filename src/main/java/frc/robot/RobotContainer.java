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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ROBOT_ID;
import frc.robot.Constants.V1ElevatorConstants;
// import frc.robot.commands.autos.PreloadTaxi;
// import frc.robot.commands.autSquare;
import frc.robot.commands.SwerveJoystickCommand;
// import frc.robot.commands.autos.AutoDriving;
import frc.robot.commands.autos.Bottom2Piece;
import frc.robot.commands.autos.isMeBottom2Piece;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeV2;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.ElevatorPivot;

import frc.robot.util.Controller;

public class RobotContainer {
  public Gyro imu = new PigeonV2(1, ModuleConstants.kCANivoreName);

  public SwerveDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  public IntakeV2 intakeV2;
  public IntakeRoller intakeRoller;
  public Elevator elevator;
  public ElevatorPivot elevatorPivot;
  public IntakeWrist intakeWrist;

  public SuperSystem superSystem;

  public Bottom2Piece bottom2Piece;

  public isMeBottom2Piece isMeBottom2Piece;

  private final Controller driverController = new Controller(ControllerConstants.kDriverControllerPort);
  private final Controller operatorController = new Controller(ControllerConstants.kOperatorControllerPort, true, true);
  
  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;
  
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  
  static boolean isRedSide = false;
  
  private SwerveJoystickCommand swerveJoystickCommand;
  
//////////////////////////////////////////////////////
  // PLEASE ONLY ENABLE THE SUBSYSTEM YOU ARE TESTING
  private static boolean USE_ELEV = false; // keep thie value is false before you check in your code
  private static boolean USE_PIVOT = false;// keep thie value is false before you check in your code
/////////////////////////////////////////////////////

  private static boolean V1 = true;


  // For logging wrist
  public final VoltageOut voltageRequest = new VoltageOut(0);
  public double voltage = 0;
  public double desiredAngle = 0.0; //164, 99.8
  public double desiredRotation = -0.01;

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

    if(Constants.ROBOT_NAME == ROBOT_ID.ISME)
    {
      // add your code only for isme 
    }

   
    intakeRoller = new IntakeRoller();
    intakeWrist = new IntakeWrist(V1);
    elevator = new Elevator();
    elevatorPivot = new ElevatorPivot(V1);
	intakeV2 = new IntakeV2();

    Elevator.enabled = USE_ELEV;
    ElevatorPivot.enabled = USE_PIVOT;


    try { // ide displayed error fix

        if(Constants.ROBOT_NAME == ROBOT_ID.ISME)
        {
          isMeBottom2Piece = new isMeBottom2Piece(swerveDrive, intakeRoller, elevator, "isMeBottom2Piece");
        }
        else
        {
          bottom2Piece = new Bottom2Piece(swerveDrive, intakeRoller, elevator, "Bottom2Piece");
        }

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
  
  public Command getAutonomousCommand() {
    Command currentAuto = autoChooser.getSelected();
    
    swerveDrive.setDriveMode(DRIVE_MODE.FIELD_ORIENTED);//AUTONOMOUS);
    return currentAuto;
  }

  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(LOG_LEVEL.MINIMAL);  

      intakeRoller.initShuffleboard(loggingLevel); 
      elevator.initShuffleboard(loggingLevel);
      intakeWrist.initShuffleboard(loggingLevel);
      elevatorPivot.initShuffleboard(loggingLevel);

  }
  
  private void initAutoChoosers() {
    try { // fix for vendordeps not importing
    PathPlannerPath S4R3 = PathPlannerPath.fromPathFile("S4R3");

  	List<String> paths = AutoBuilder.getAllAutoNames();
    
    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);

    if(Constants.ROBOT_NAME == ROBOT_ID.ISME)
    {
      autoChooser.addOption("isMe Bottom 2 Piece", isMeBottom2Piece);
    }
    else 
    {
      autoChooser.setDefaultOption("Bottom 2 Piece", bottom2Piece);
    }

    autoChooser.addOption("Square just drive", AutoBuilder.buildAuto("Square"));
    autoChooser.addOption("Taxi", AutoBuilder.buildAuto("Taxi"));
    // if (paths.contains("S4R3")) {
      // autoChooser.addOption("PreloadTaxi", AutoBuilder.buildAuto("PreloadTaxi"));
      // autoChooser.addOption("PreloadTaxi2", new PreloadTaxi(swerveDrive, List.of(S4R3)));
    // }
    } catch (Exception e) { SmartDashboard.putBoolean("Auto Error", true); }
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

  public void configureBindings_teleop() {
    // Driver bindings
    driverController.controllerLeft().onTrue(
      Commands.runOnce(() -> swerveDrive.zeroGyroAndPoseAngle()) // TODO: When camera pose is implemented, this won't be necessary anymore
    );
    
    // driverController.triggerRight()
    //   .whileTrue(bottom2Piece.runAuto())
    //   .onFalse(bottom2Piece.stopAuto());
    
    // driverController.buttonUp()
    //   .onTrue(intakeWrist.moveToStation())
    //   .onFalse(intakeWrist.moveToStow());
    // driverController.buttonLeft()
    //   .onTrue(intakeWrist.setEnabledCommand());
    // driverController.buttonRight()
    //   .onTrue(intakeWrist.setDisabledCommand());
    // driverController.buttonUp()
    //   .onTrue(intakeRoller.intakeLeft())
    //   .onFalse(intakeRoller.stop());
    // driverController.buttonDown()
    //   .onTrue(intakeRoller.outtake())
    //   .onFalse(intakeRoller.stop());
    // driverController.buttonLeft()
    //   .onTrue(intakeRoller.intake())
    //   .onFalse(intakeRoller.stop());

    // driverController.bumperLeft()
    //   .whileTrue(Commands.run(() -> {
    //     desiredRotation += 0.005; // 1/60 for GR 1/50 for 20 times per second
    //     intakeWrist.setPosition(desiredRotation);
    // }));
    
    // Position Ramp Wrist
    // driverController.bumperLeft()
    //   .whileTrue(Commands.run(() -> {
    //     desiredRotation -= 0.0004; // one degree / 360 per second times 13.889 for GR 1/50 for 20 times per second
    // }));
    // driverController.bumperRight()
    //   .whileTrue(Commands.run(() -> {
    //     desiredRotation += 0.0004; // one degree / 360 per second times 13.889 for GR 1/50 for 20 times per second
    //     intakeWrist.setPosition(desiredRotation);
    // }));

    // Position Ramp Elevator
    // driverController.bumperLeft()
    //   .whileTrue(Commands.run(() -> {
    //     desiredRotation += 0; // 1/ for GR 1/50 for 20 times per second
    //     elevator.setPosition(desiredRotation);
    // }));

    driverController.bumperRight()
      .whileTrue(intakeWrist.moveToStow()
    );
    driverController.triggerRight()
      .whileTrue(intakeWrist.moveToStation()
    );
    driverController.bumperLeft()
      .whileTrue(intakeWrist.moveToReefL14()
    );
    driverController.triggerLeft()
      .whileTrue(intakeWrist.moveToReefL23()
    );

    driverController.buttonUp()
      .onTrue(intakeWrist.setEnabledCommand()
    ).onFalse(intakeWrist.setDisabledCommand());

    // driverController.buttonUp()
    //   .whileTrue(
    //     Commands.parallel(
    //       // Commands.run(() -> intakeWrist.setPositionDegrees(desiredAngle)),
    //       Commands.run(() -> intakeWrist.setPosition(desiredRotation)),
    //       intakeWrist.setEnabledCommand()
    //   ))
    //   .onFalse(intakeWrist.setDisabledCommand()
    //   );

    // driverController.buttonDown()
    //   .onTrue(Commands.runOnce(() -> {
    //     desiredRotation = -0.19;
    //     // desiredAngle = 90.0; //Top: -85.4
    //   }));
  
    // driverController.buttonUp()
    //   .onTrue(intakeWrist.setEnabledCommand())
    //   .onFalse(intakeWrist.setDisabledCommand());
    
    driverController.buttonLeft()
      .onTrue(intakeWrist.moveToReefL14())
      .onFalse(intakeWrist.moveToStow());


    
    driverController.buttonRight()
      .onTrue(intakeWrist.moveToReefL23())
      .onFalse(intakeWrist.setDisabledCommand()); 
    

    /*******************************
     * Pivot Section
     *******************************/
    
      // Voltage Ramp Pivot Positive
      /*operatorController.bumperRight()
        .whileTrue(Commands.run(() -> {
          voltage += 0.2 / 50.0;
          elevatorPivot.setPivotVoltage(voltage);
      }, elevatorPivot));

      // Voltage Ramp Pivot Negative
      operatorController.bumperLeft()
        .whileTrue(Commands.run(() -> {
          voltage -= 0.2 / 50.0;
          elevatorPivot.setPivotVoltage(voltage);
      }, elevatorPivot));*/
	  
	  // Position Ramp Pivot Positive
    operatorController.bumperRight()
      .whileTrue(Commands.run(() -> {
        desiredRotation += (1.0 / 360.0); // 1 degree per second in terms of rotations
        elevatorPivot.setTargetPosition(desiredRotation);
    }));

    // POSITIVE POSITION = UP
    // TODO: Find min and max position rotations of pivot. Find where 0 'ground' is, needs to be parallel to floor. See which way is rotation positive.
    // Min (Parallel): -25.9      Vertical: 
    // TODO: Fill in sensortomechaanism ratio for pivot and pivotRight. Calculate kP After.

    // // Position Ramp Pivot Negative
    operatorController.bumperLeft()
      .whileTrue(Commands.run(() -> {
        desiredRotation -= (1.0 / 360.0); // 1 degree per second in terms of rotations
        elevatorPivot.setTargetPosition(desiredRotation);
    }));
    


    operatorController.buttonUp()
      .whileTrue(Commands.run(() -> elevatorPivot.setTargetPosition(V1ElevatorConstants.kElevatorPivotStowPosition))
    );

    operatorController.buttonRight()
      .whileTrue(Commands.run(() -> elevatorPivot.setTargetPosition(V1ElevatorConstants.kElevatorPivotPosition30))
    );

    operatorController.buttonDown()
      .whileTrue(Commands.run(() -> elevatorPivot.setTargetPosition(V1ElevatorConstants.kElevatorPivotPosition60))
    );

    operatorController.buttonLeft()
      .whileTrue(Commands.run(() -> elevatorPivot.setTargetPosition(V1ElevatorConstants.kElevatorPivotPositionVertical))
    );


    // Enable Pivot
    operatorController.triggerRight()
      .whileTrue(elevatorPivot.setEnabledCommand(true)
      // ,elevatorPivot.resetEncoders() // Don't think there's a way to do this
    );

    /*******************************
    * Elevtor Section
    *******************************/


    /*******************************
    * Intake Section
    *******************************/

    

    /*******************************
    * Intake Section
    *******************************/

  }
  


  




  /**
   * 
   * TEST mode section! 
   * Caution: The code here will only be used for testing purposes. 
   * 
   */ 
   public void initDefaultCommands_test() {
    swerveDrive.setDriveMode(DRIVE_MODE.ROBOT_ORIENTED);
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

    driverController.bumperRight()
      .whileTrue(Commands.run(() -> {
        desiredAngle -= 1 / 50; // 1 degree per second ish
        // voltage = -1.928;
      }));
    driverController.bumperLeft()
      .whileTrue(Commands.run(() -> {
        intakeWrist.setPositionDegrees(desiredAngle);
      }));
    driverController.buttonDown()
      .onTrue(Commands.runOnce(() -> {
        desiredAngle = 90.0; //Top: -85.4
      }));

      // driverController.bumperRight()
      // .whileTrue(Commands.run(() -> {
      //   voltage += 0.2 / 50.0;
      //   // voltage = -1.928;
      // }));
      // driverController.bumperLeft()
      //   .whileTrue(Commands.run(() -> {
      //     intakeWrist.getMotor().setControl(voltageRequest.withOutput(voltage));
      // }));
      // driverController.buttonDown()
      //   .onTrue(Commands.runOnce(() -> {
      //     voltage = 0;
      // }));
  }
  
}