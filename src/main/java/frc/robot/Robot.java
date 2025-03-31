// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DataLogManager.start("/media/sda1/logs");
    DataLogManager.logNetworkTables(true);
    m_robotContainer.swerveDrive.refreshModulePID();

    // Start CameraServer for video streaming
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // m_robotContainer.superSystemCommand.updateDependencies();
    // Access Limelight data from NetworkTables
    NetworkTable tableBackLeft = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.kLimelightBackLeftName);
    NetworkTable tableBackRight = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.kLimelightBackRightName);

    double txLeft = tableBackLeft.getEntry("tx").getDouble(0.0);
    double tyLeft = tableBackLeft.getEntry("ty").getDouble(0.0);
    double taLeft = tableBackLeft.getEntry("ta").getDouble(0.0);

    double txRight = tableBackRight.getEntry("tx").getDouble(0.0);
    double tyRight = tableBackRight.getEntry("ty").getDouble(0.0);
    double taRight = tableBackRight.getEntry("ta").getDouble(0.0);

    // Display Limelight data on SmartDashboard
    SmartDashboard.putNumber("Limelight Left X", txLeft);
    SmartDashboard.putNumber("Limelight Left Y", tyLeft);
    SmartDashboard.putNumber("Limelight Left Area", taLeft);

    SmartDashboard.putNumber("Limelight Right X", txRight);
    SmartDashboard.putNumber("Limelight Right Y", tyRight);
    SmartDashboard.putNumber("Limelight Right Area", taRight);

    // Optionally display the camera feed URLs (adjust for your Limelight IPs)
    SmartDashboard.putString("Limelight Left Feed URL", "http://10.6.87.5:5800");
    SmartDashboard.putString("Limelight Right Feed URL", "http://10.6.87.7:5800");
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.swerveDrive.disableLimelightCommand();

    if (RobotContainer.USE_SUBSYSTEMS) {
      m_robotContainer.pivot.setEnabled(false);
      m_robotContainer.elevator.setEnabled(false);
      m_robotContainer.wrist.setEnabled(false);
      m_robotContainer.intakeRoller.setEnabled(false);
      m_robotContainer.climbMotor.setEnabled(false);
    }
  }

  @Override
  public void disabledPeriodic() {
    // m_robotContainer.elevatorPivot.setTargetPosition(m_robotContainer.elevatorPivot.getPosition());
    // m_robotContainer.elevator.setTargetPosition(m_robotContainer.elevator.getPosition());
    // m_robotContainer.intakeWrist.setTargetPosition(m_robotContainer.intakeWrist.getPosition());

    // m_robotContainer.elevatorPivot.setTargetPosition(m_robotContainer.elevatorPivot.getPosition());
    //m_robotContainer.elevator.setTargetPosition(0);
    //m_robotContainer.intakeWrist.setTargetPosition(m_robotContainer.intakeWrist.getPosition());

  }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.refreshAlliance();
    m_robotContainer.imu.zeroAll();
    m_robotContainer.swerveDrive.enableLimeLight();

    if (RobotContainer.USE_SUBSYSTEMS) {
      m_robotContainer.superSystem.setNeutralMode(NeutralModeValue.Brake);
      m_robotContainer.superSystem.initialize();
    }
  // schedule the autonomous command (example)
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    RobotContainer.refreshAlliance();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.swerveDrive.enableLimeLight();
    // need them once it comes back from Test Mode
    if (RobotContainer.USE_SUBSYSTEMS) {
      m_robotContainer.superSystem.setNeutralMode(NeutralModeValue.Brake);
      m_robotContainer.superSystem.initialize();
    }

    m_robotContainer.initDefaultCommands_teleop();
    m_robotContainer.configureBindings_teleop();
    m_robotContainer.initDefaultCommands_teleop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    if (RobotContainer.USE_SUBSYSTEMS) {
      m_robotContainer.superSystem.setNeutralMode(NeutralModeValue.Coast);
    }
    // m_robotContainer.superSystem.initialize();
    m_robotContainer.initDefaultCommands_test();
    m_robotContainer.configureBindings_test();

    m_robotContainer.DisableAllMotors_Test();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
