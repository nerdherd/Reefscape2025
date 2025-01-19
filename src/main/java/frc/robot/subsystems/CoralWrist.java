package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.Constants.ControllerConstants;
import frc.robot.util.NerdyMath;
import frc.robot.util.filters.ExponentialSmoothingFilter;

public class CoralWrist extends SubsystemBase implements Reportable{
    private final TalonFX motor;
    private final TalonFXConfigurator motorConfigurator;

    private final PIDController motorPID;
    private double desiredPosition;
    private double desiredVelocity;
    public boolean enabled = false;

    public CoralWrist(){
        motor = new TalonFX(CoralConstants.kWristMotorID);
        motorConfigurator = motor.getConfigurator();

        // configure motor
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        configurePID(motorConfigs);

        motorPID = new PIDController(0.2, 0, 0);
        
        motor.setNeutralMode(NeutralModeValue.Brake);
        zeroEncoder();
        setEnabled(false);
    }
    
    private void configurePID(TalonFXConfiguration motorConfigs) {
        motorConfigurator.refresh(motorConfigs);
    
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 25;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 30;
        motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
        CoralConstants.kPWristMotor.loadPreferences();
        CoralConstants.kIWristMotor.loadPreferences();
        CoralConstants.kDWristMotor.loadPreferences();
        CoralConstants.kVWristMotor.loadPreferences();
    
        motorConfigs.Slot0.kP = CoralConstants.kPWristMotor.get();
        motorConfigs.Slot0.kI = CoralConstants.kIWristMotor.get();
        motorConfigs.Slot0.kD = CoralConstants.kDWristMotor.get();
        motorConfigs.Slot0.kV = CoralConstants.kVWristMotor.get();
    
        StatusCode response = motorConfigurator.apply(motorConfigs);
        if (!response.isOK()){
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
        }

    }

    public void zeroEncoder() {
        motor.setPosition(0);
    }

    @Override
    public void periodic() {
        if (enabled) {
            setVelocity((motorPID.calculate(motor.getPosition().getValueAsDouble(), desiredPosition))/2);
            if (desiredPosition == CoralConstants.kWristStowPosition) {     //TODO see if deletable also see elevator
                setVelocity((motorPID.calculate(motor.getPosition().getValueAsDouble(), desiredPosition))/2);
            }
        }
        else {
            setVelocity((motorPID.calculate(motor.getPosition().getValueAsDouble(), CoralConstants.kWristStowPosition))/2);
        }
    }

    //****************************** POSITION METHODS ******************************//

    private void setEnabled(boolean e) {
        this.enabled = e;
        if (!e) desiredPosition = CoralConstants.kWristStowPosition;
    }
    public Command setDisabledCommand() {
        return Commands.runOnce(() -> this.setEnabled(false));
    }
    public Command setEnabledCommand() {
        return Commands.runOnce(() -> this.setEnabled(true));
    }
    
    private void setPosition(double position) {
        desiredPosition = position;
    }
    public Command setPositionCommand(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    public Command goToPosition(double targetPosition) {
        return Commands.sequence(
            setEnabledCommand(),
            setPositionCommand(targetPosition)
        );
    }

    private void setVelocity(double velocity) {
        desiredVelocity = NerdyMath.clamp(velocity, -CoralConstants.kWristSpeed, CoralConstants.kWristSpeed);
        motor.set(desiredVelocity);
    }
    public Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    //****************************** NAMED COMMANDS ******************************//

    public Command stowCommand() {
        return goToPosition(CoralConstants.kWristStowPosition);
    }
    
    public Command upCommand() {
        return goToPosition(CoralConstants.kWristUpPostion);
    }

    //****************************** LOGGING METHODS ******************************//
    @Override
    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
                // SmartDashboard.putNumber("Coral Wrist Motor Output", pivot.getMotorOutputPercent());
                SmartDashboard.putNumber("Coral Wrist Position", motor.getPosition().getValueAsDouble());
                // SmartDashboard.putNumber("Coral Wrist Velocity", pivot.getSelectedSensorVelocity());
            case MEDIUM:
                SmartDashboard.putNumber("Coral Wrist Current", motor.getStatorCurrent().getValueAsDouble());
                // SmartDashboard.putNumber("Coral Wrist Voltage", pivot.getMotorOutputVoltage());
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

            case MINIMAL:
                tab.addNumber("Current Coral Wrist Angle", () -> motor.getPosition().getValueAsDouble());
                break;
        }
    }
}