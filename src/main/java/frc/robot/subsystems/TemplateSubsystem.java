// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class TemplateSubsystem extends SubsystemBase {
  private final TalonFX motor1;
  private final TalonFX motor2;

  private final MotionMagicVoltage positionController;
  private final VelocityVoltage velocityController;
  private final Follower followerController;

  private final NeutralOut neutralRequest = new NeutralOut();

  private double desiredValue; //  target position or velocity depending on mode

  private boolean enabled = false;

  private final ShuffleboardTab shuffleboardTab;
  
  public enum SubsystemMode {
    POSITION, 
    VELOCITY
  }
  private final SubsystemMode mode;

  public enum LoggingLevel {
    ALL(4),
    MEDIUM(3),
    MINIMAL(2),
    NONE(1)
    ;

    public int level = 0;
    LoggingLevel(int level) { this.level = level; }
  }

  public TemplateSubsystem(String name, int motor1ID, int motor2ID, boolean reverseMotor2, SubsystemMode mode, double defaultValue) {
    this.motor1 = new TalonFX(motor1ID);
    if(motor2ID != -1){
      this.motor2 = new TalonFX(motor2ID);
      followerController = new Follower(motor1ID, reverseMotor2);
    }else{
      this.motor2 = null;
      followerController = null;
    }
    this.mode = mode;
    this.desiredValue = defaultValue;
    if (this.mode == SubsystemMode.POSITION){
      positionController = new MotionMagicVoltage(defaultValue);
      velocityController = null;
    }else{
      positionController = null;
      velocityController = new VelocityVoltage(defaultValue);
    }
    shuffleboardTab = Shuffleboard.getTab(name);
  }

  public TemplateSubsystem(String name, int motor1ID, SubsystemMode mode, double defaultValue){
    this(name, motor1ID, -1, false, mode, defaultValue);
  }

  @Override
  public void periodic() {
    if(!enabled){
      stop();
      return;
    }

    if(mode == SubsystemMode.POSITION) 
      motor1.setControl(positionController.withPosition(this.desiredValue));
    else if(mode == SubsystemMode.VELOCITY) 
      motor1.setControl(velocityController.withVelocity(this.desiredValue));

    if(motor2 != null) motor2.setControl(followerController);
  }

  // ------------------------------------ Helper Functions ------------------------------------ //
  
  public void stop(){
    motor1.setControl(neutralRequest);
    if (motor2 != null) motor2.setControl(neutralRequest);
  }

  public void setEnabled(boolean enabled){
    this.enabled = enabled;
    if (!this.enabled) stop();
  }

  public void disable() {
    this.setEnabled(false);
  }
  
  public void enable(){
    this.setEnabled(true);
  }

  public void setDesiredValue(double value) {
    this.desiredValue = value;
  }

  public double getDesiredValue(){
    return desiredValue;
  }

  // ------------------------------------ Command Functions ------------------------------------ //

  public Command setEnabledCommand(boolean newEnabled) {
    return Commands.runOnce(() -> setEnabled(newEnabled));
  }

  public Command setDesiredCommand(double newValue) {
    return Commands.runOnce(() -> setDesiredValue(newValue));
  }

  public Command stopCommand() {
    return Commands.runOnce(() -> stop());
  }

  // ------------------------------------ Logging Functions ------------------------------------ //

  public abstract void initializeLogging();

  private void addNumber(String name, DoubleSupplier supplier, LoggingLevel loggingLevel) {
    if(Constants.LOGGING_LEVEL.level >= loggingLevel.level) shuffleboardTab.addNumber(name, supplier);
  }

  private void addBoolean(String name, BooleanSupplier supplier, LoggingLevel loggingLevel) {
    if(Constants.LOGGING_LEVEL.level >= loggingLevel.level) shuffleboardTab.addBoolean(name, supplier);
  }

  private void addString(String name, Supplier<String> supplier, LoggingLevel loggingLevel) {
    if(Constants.LOGGING_LEVEL.level >= loggingLevel.level) shuffleboardTab.addString(name, supplier);
  }
}
