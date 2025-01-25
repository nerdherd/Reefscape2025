package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.util.NerdyMath;

public class CoralWrist extends SubsystemBase implements Reportable{
    private final TalonFX motor;
    private final TalonFXConfigurator motorConfigurator;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final NeutralOut brakeRequest = new NeutralOut();

    private double desiredPosition = CoralConstants.kWristStowPosition;
    private boolean enabled = false;

    public CoralWrist(){
        motor = new TalonFX(CoralConstants.kWristMotorID);
        motorConfigurator = motor.getConfigurator();

        // configure motor
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        configurePID(motorConfigs);
        
        motor.setNeutralMode(NeutralModeValue.Brake);
        zeroEncoder();
    }

    //****************************** SETUP METHODS ******************************//
    
    private void configurePID(TalonFXConfiguration motorConfigs) {
        motorConfigurator.refresh(motorConfigs);
    
        motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfigs.Feedback.SensorToMechanismRatio = -12.0/54.0;
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 25;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 30;
        motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
        CoralConstants.kPWristMotor.loadPreferences();
        CoralConstants.kIWristMotor.loadPreferences();
        CoralConstants.kDWristMotor.loadPreferences();
        CoralConstants.kVWristMotor.loadPreferences();
        CoralConstants.kSWristMotor.loadPreferences();
        CoralConstants.kGWristMotor.loadPreferences();
        CoralConstants.kWristAcceleration.loadPreferences();
        CoralConstants.kWristJerk.loadPreferences();
    
        motorConfigs.Slot0.kP = CoralConstants.kPWristMotor.get();
        motorConfigs.Slot0.kI = CoralConstants.kIWristMotor.get();
        motorConfigs.Slot0.kD = CoralConstants.kDWristMotor.get();
        motorConfigs.Slot0.kV = CoralConstants.kVWristMotor.get();
        motorConfigs.Slot0.kS = CoralConstants.kSWristMotor.get();
        motorConfigs.Slot0.kG = CoralConstants.kGWristMotor.get();

        motorConfigs.MotionMagic.MotionMagicAcceleration = CoralConstants.kWristAcceleration.get();
        motorConfigs.MotionMagic.MotionMagicJerk = CoralConstants.kWristJerk.get();
    
        StatusCode response = motorConfigurator.apply(motorConfigs);
        if (!response.isOK()){
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
        }

    }

    private void zeroEncoder() {
        motor.setPosition(0);
    }

    @Override
    public void periodic() {
        motionMagicRequest.Position = desiredPosition;
        if (enabled) {
            motor.setControl(motionMagicRequest);
        }
        else {
            motor.setControl(brakeRequest);
        }
    }

    // ****************************** STATE METHODS ****************************** //

    private void setEnabled(boolean e) {
        this.enabled = e;
    }
    
    private void setPosition(double position) {
        desiredPosition = position;
    }

    // ****************************** COMMAND METHODS ****************************** //

    private Command setDisabledCommand() {
        return Commands.runOnce(() -> this.setEnabled(false));
    }
    private Command setEnabledCommand() {
        return Commands.runOnce(() -> this.setEnabled(true));
    }

    private Command setPositionCommand(double position) {
        return Commands.sequence(
            setEnabledCommand(),
            Commands.runOnce(() -> setPosition(position))
        );
    }

    private Command stopCommand() {
        return Commands.sequence(
            setDisabledCommand(),
            Commands.runOnce(() -> motor.setControl(brakeRequest))
        );
    }

    // ****************************** NAMED COMMANDS ****************************** //

    public Command stow() {
        return setPositionCommand(CoralConstants.kWristStowPosition);
    }
    
    public Command raise() {
        return setPositionCommand(CoralConstants.kWristUpPostion);
    }

    public Command stop() {
        return stopCommand();
    }

    // ****************************** LOGGING METHODS ****************************** //

    @Override
    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
                SmartDashboard.putNumber("Coral Wrist Position", motor.getPosition().getValueAsDouble());
            case MEDIUM:
                SmartDashboard.putNumber("Coral Wrist Current", motor.getStatorCurrent().getValueAsDouble());
            case MINIMAL:
                break;
        }
    }
    
    @Override
    public void initShuffleboard(LOG_LEVEL level) { 
        if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Coral Wrist");
        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addString("Control Mode", motor.getControlMode()::toString);
            case MEDIUM:
                tab.addDouble("MM Position", () -> motionMagicRequest.Position);
                tab.addDouble("Desired Position", () -> desiredPosition);
            case MINIMAL:
                tab.addBoolean("Enabled", () -> enabled);
                tab.addNumber("Current Coral Wrist Angle", () -> motor.getPosition().getValueAsDouble());
                break;
        }
    }

}