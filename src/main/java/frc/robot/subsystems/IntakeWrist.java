package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeWrist extends SubsystemBase implements Reportable{
    private final TalonFX motor;
    private final TalonFXConfigurator motorConfigurator;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final NeutralOut brakeRequest = new NeutralOut();

    private double desiredPosition = IntakeConstants.kWristStowPosition;
    private boolean enabled = true;

    public IntakeWrist() {
        motor = new TalonFX(IntakeConstants.kWristMotorID);
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
        motorConfigs.Feedback.SensorToMechanismRatio = 12.0/54.0;
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 25;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 30;
        motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
        motorConfigs.Slot0.kP = IntakeConstants.kPWristMotor;
        motorConfigs.Slot0.kI = IntakeConstants.kIWristMotor;
        motorConfigs.Slot0.kD = IntakeConstants.kDWristMotor;
        motorConfigs.Slot0.kV = IntakeConstants.kVWristMotor;
        motorConfigs.Slot0.kS = IntakeConstants.kSWristMotor;
        motorConfigs.Slot0.kG = IntakeConstants.kGWristMotor;

        motorConfigs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.kWristCruiseVelocity;
        motorConfigs.MotionMagic.MotionMagicAcceleration = IntakeConstants.kWristAcceleration;
        motorConfigs.MotionMagic.MotionMagicJerk = IntakeConstants.kWristJerk;
    
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

    private Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> this.setEnabled(enabled));
    }

    private Command setPositionCommand(double position) {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.runOnce(() -> setPosition(position))
        );
    }

    private Command stopCommand() {
        return Commands.sequence(
            setEnabledCommand(false),
            Commands.runOnce(() -> motor.setControl(brakeRequest))
        );
    }

    // ****************************** NAMED COMMANDS ****************************** //

    public Command stow() {
        return setPositionCommand(IntakeConstants.kWristStowPosition);
    }

    public Command moveToStation() {
        return setPositionCommand(IntakeConstants.kWristStationPosition);
    }

    public Command moveToPickup() {
        return setPositionCommand(IntakeConstants.kWristPickupPosition);
    }

    public Command moveToReefL14() {
        return setPositionCommand(IntakeConstants.kWristL14Position);
    }

    public Command moveToReefL23() {
        return setPositionCommand(IntakeConstants.kWristL23Position);
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