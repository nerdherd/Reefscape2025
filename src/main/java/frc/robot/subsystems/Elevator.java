package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.NerdyMath;

public class Elevator extends SubsystemBase implements Reportable {
    private final TalonFX elevatorMotor;
    private final TalonFX elevatorMotor2;

    // private final PIDController elevatorPID;
    private double desiredPosition;
    private double desiredVelocity;
    private boolean enabled = false;
    private TalonFXConfigurator motorConfigurator;
    private TalonFXConfigurator motorConfigurator2;
    private MotionMagicVoltage motionMagicVoltage;
    private final Follower followRequest;
    private final NeutralOut brakeRequest;
    private double ff; 
    private double pivotAngle = 0.25; // TODO: Change this to 0 when supersystem tuned

    public Elevator() {
        elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorID, "rio");
        elevatorMotor2 = new TalonFX(ElevatorConstants.kElevatorMotorID2, "rio");
        motionMagicVoltage = new MotionMagicVoltage(0);
        brakeRequest = new NeutralOut();
        // elevatorPID = new PIDController(
        //     ElevatorConstants.kPElevatorMotor,
        //     ElevatorConstants.kIElevatorMotor,
        //     ElevatorConstants.kDElevatorMotor);
        elevatorMotor.setPosition(0.0);

        motorConfigurator = elevatorMotor.getConfigurator();
        motorConfigurator2 = elevatorMotor2.getConfigurator();

        setMotorConfigs();

        followRequest = new Follower(ElevatorConstants.kElevatorMotorID, true);
        motionMagicVoltage.withSlot(0);
    }
    
    private void setMotorConfigs() {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        motorConfigurator.refresh(motorConfigs);
        motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfigs.Feedback.SensorToMechanismRatio = 16; 
        motorConfigs.Feedback.RotorToSensorRatio = 1;
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false; // TODO: change
        motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 45;
        motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs.MotionMagic.MotionMagicCruiseVelocity =  ElevatorConstants.kElevatorCruiseVelocity;
        motorConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kElevatorCruiseAcceleration;
        motorConfigs.MotionMagic.MotionMagicJerk = ElevatorConstants.kElevatorJerk;

        motorConfigs.Slot0.kP = ElevatorConstants.kPElevatorMotor;
        // motorConfigs.Slot0.kG = ElevatorConstants.kGElevatorMotor;
        // motorConfigs.Slot0.kS = ElevatorConstants.kSElevatorMotor;

        StatusCode response = motorConfigurator.apply(motorConfigs);
        if (!response.isOK()){
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
        }

        TalonFXConfiguration motorConfigs2 = new TalonFXConfiguration();
        motorConfigurator2.refresh(motorConfigs2);
        motorConfigs2.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfigs2.Feedback.SensorToMechanismRatio = 16; 
        motorConfigs2.Feedback.RotorToSensorRatio = 1;  
        motorConfigs2.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfigs2.CurrentLimits.SupplyCurrentLimitEnable = false; // TODO: change
        motorConfigs2.CurrentLimits.SupplyCurrentLowerLimit = 45;
        motorConfigs2.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        motorConfigs2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs2.MotionMagic.MotionMagicCruiseVelocity =  ElevatorConstants.kElevatorCruiseVelocity;
        motorConfigs2.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kElevatorCruiseAcceleration;
        motorConfigs2.MotionMagic.MotionMagicJerk = ElevatorConstants.kElevatorJerk;

        motorConfigs2.Slot0.kP = ElevatorConstants.kPElevatorMotor;
        // motorConfigs2.Slot0.kG = ElevatorConstants.kGElevatorMotor;
        // motorConfigs2.Slot0.kS = ElevatorConstants.kSElevatorMotor;

        StatusCode response2 = motorConfigurator2.apply(motorConfigs2);
        if (!response2.isOK()){
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
        }
    }


    @Override
    public void periodic() {
        if (!enabled) {
            elevatorMotor.setControl(brakeRequest);
            return;
        }
        elevatorMotor2.setControl(followRequest);
        ff = (ElevatorConstants.kGElevatorMotor + ElevatorConstants.kSElevatorMotor) * Math.cos(pivotAngle * 2 * Math.PI);
        elevatorMotor.setControl(motionMagicVoltage.withFeedForward(ff));
    }

    // ****************************** STATE METHODS ****************************** //

    private void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) desiredPosition = ElevatorConstants.kElevatorStowPosition;
    }
    
    private void setPosition(double position) {
        desiredPosition = position;
        motionMagicVoltage.Position = desiredPosition;
    }

    public void setPivotAngle(double pivotAngle) {
        this.pivotAngle = pivotAngle;
    }

    // ****************************** GET METHODS ***************************** //

    public double getPosition() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    // ****************************** COMMAND METHODS ***************************** //

    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> this.setEnabled(enabled));
    }

    public Command setPositionCommand(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    private Command stopCommand() {
        return Commands.sequence(
            setEnabledCommand(false)
        );
    }

    // ****************************** NAMED COMMANDS ****************************** //

    public Command stow() {
        return setPositionCommand(ElevatorConstants.kElevatorStowPosition);
    }

    public Command moveToStation() {
        return setPositionCommand(ElevatorConstants.kElevatorStationPosition);
    }

    public Command moveToReefL1() {
        return setPositionCommand(ElevatorConstants.kElevatorL1Position);
    }

    public Command moveToReefL2() {
        return setPositionCommand(ElevatorConstants.kElevatorL2Position);
    }

    public Command moveToReefL3() {
        return setPositionCommand(ElevatorConstants.kElevatorL3Position);
    }

    public Command moveToReefL4() {
        return setPositionCommand(ElevatorConstants.kElevatorL4Position);
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
            case MEDIUM:
            case MINIMAL:
                SmartDashboard.putNumber("Elevator Desired Position", desiredPosition);
                SmartDashboard.putNumber("Elevator Current Position", elevatorMotor.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("Elevator Desired Velocity", desiredVelocity);
                SmartDashboard.putNumber("Elevator Current Velocity", elevatorMotor.getVelocity().getValueAsDouble());
                SmartDashboard.putBoolean("Elevator Enabled", this.enabled);
        }
    }

    @Override
    public void initShuffleboard(LOG_LEVEL level) {
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator");;
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
            case MINIMAL:
                tab.addNumber("Elevator Desired Velocity", () -> desiredVelocity);
                tab.addNumber("Elevator Current Velocity", () -> elevatorMotor.getVelocity().getValueAsDouble());
                tab.addNumber("Elevator Desired Position", () -> desiredPosition);
                tab.addNumber("Elevator Current Position", () -> elevatorMotor.getPosition().getValueAsDouble());
                tab.addBoolean("Elevator Enabled", () -> enabled);
                break;
        }
    }

}
