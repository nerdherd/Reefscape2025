// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

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
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DataLogManager.start("/media/sda1/logs");
    DataLogManager.logNetworkTables(true);
    m_robotContainer.swerveDrive.refreshModulePID();
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // m_robotContainer.superSystemCommand.updateDependencies();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.swerveDrive.setBreak(true);
    
    m_robotContainer.elevatorPivot.setEnabled(false);
    m_robotContainer.elevator.setEnabled(false);
    m_robotContainer.intakeWrist.setEnabled(false);
    m_robotContainer.intakeRoller.setEnabled(false);
    m_robotContainer.climbMotor.setEnabled(false);
  }
  
  @Override
  public void disabledPeriodic() {
    // m_robotContainer.elevatorPivot.setTargetPosition(m_robotContainer.elevatorPivot.getPosition());
    // m_robotContainer.elevator.setTargetPosition(m_robotContainer.elevator.getPosition());
    // m_robotContainer.intakeWrist.setTargetPosition(m_robotContainer.intakeWrist.getPosition());
    m_robotContainer.elevator.stopMotion();
    m_robotContainer.elevatorPivot.stopMotion();
    m_robotContainer.intakeWrist.stopMotion();
    // m_robotContainer.elevatorPivot.setTargetPosition(m_robotContainer.elevatorPivot.getPosition());
    //m_robotContainer.elevator.setTargetPosition(0);
    //m_robotContainer.intakeWrist.setTargetPosition(m_robotContainer.intakeWrist.getPosition());

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.refreshAlliance();
    m_robotContainer.imu.zeroAll();
    
    // need them once it comes back from Test Mode
    m_robotContainer.elevator.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.elevatorPivot.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.intakeWrist.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.climbMotor.setNeutralMode(NeutralModeValue.Brake);

    m_robotContainer.refreshSupersystem();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
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
    
    // need them once it comes back from Test Mode
    m_robotContainer.elevator.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.elevatorPivot.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.intakeWrist.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.climbMotor.setNeutralMode(NeutralModeValue.Brake);

    m_robotContainer.initDefaultCommands_teleop();
    m_robotContainer.configureBindings_teleop();
    m_robotContainer.initDefaultCommands_teleop();

    m_robotContainer.refreshSupersystem();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.superSystem.reConfigureMotors();
    

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
