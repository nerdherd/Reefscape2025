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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.NerdyMath;

public class Elevator extends SubsystemBase implements Reportable {
    private final TalonFX elevatorMotor;
    private final TalonFX elevatorMotor2;

    // private final PIDController elevatorPID;
    private double desiredPosition = 0.0;
    private boolean enabled = false;
    private TalonFXConfigurator motorConfigurator;
    private TalonFXConfigurator motorConfigurator2;
    private MotionMagicVoltage motionMagicVoltage;
    private final Follower followRequest;
    private final NeutralOut brakeRequest = new NeutralOut();;
    private double ff = 0.0; 
    private double pivotAngle = 0.0; // TODO: Change this to 0 when supersystem tuned
    
    private NeutralModeValue neutralMode = NeutralModeValue.Brake;

    public Elevator() {
        elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorID, "rio");
        elevatorMotor2 = new TalonFX(ElevatorConstants.kElevatorMotorID2, "rio");
        motionMagicVoltage = new MotionMagicVoltage(0);
        
        elevatorMotor.setPosition(0.0);

        motorConfigurator = elevatorMotor.getConfigurator();
        motorConfigurator2 = elevatorMotor2.getConfigurator();

        setMotorConfigs();

        followRequest = new Follower(ElevatorConstants.kElevatorMotorID, true);
        motionMagicVoltage.withSlot(0);
        zeroEncoder();
        CommandScheduler.getInstance().registerSubsystem(this);
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
        motorConfigs.MotorOutput.NeutralMode = neutralMode;
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
        motorConfigs2.MotorOutput.NeutralMode = neutralMode;
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
        ff = ElevatorConstants.kGElevatorMotor * Math.sin(pivotAngle * 2 * Math.PI);
        elevatorMotor.setControl(motionMagicVoltage.withFeedForward(ff));
    }

    // ****************************** STATE METHODS ****************************** //

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        this.neutralMode = neutralMode;
    }

    public void stopMotion() {
        elevatorMotor.setControl(brakeRequest);
        elevatorMotor2.setControl(brakeRequest);
    }
    
    public void setTargetPosition(double position) {
        //TODO NerdyMath.clamp(
        desiredPosition = position;
        motionMagicVoltage.Position = desiredPosition;
    }

    public void setPivotAngle(double pivotAngle) {
        this.pivotAngle = pivotAngle;
    }

    public void zeroEncoder() {
        elevatorMotor.setPosition(0.0);
        elevatorMotor2.setPosition(0.0);
        desiredPosition = 0.0;
    }

    // ****************************** GET METHODS ***************************** //

    public double getPosition() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    public boolean atPosition() {
        return NerdyMath.inRange(elevatorMotor.getPosition().getValueAsDouble(), 
        desiredPosition - 0.125,
        desiredPosition + 0.125);
    }

    public boolean atPositionWide() {
        return NerdyMath.inRange(elevatorMotor.getPosition().getValueAsDouble(), 
        // desiredPosition - 0.25,
        // desiredPosition + 0.25);
        desiredPosition - 0.5,
        desiredPosition + 0.5);

    }

    // ****************************** COMMAND METHODS ***************************** //

    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> this.setEnabled(enabled));
    }

    public Command setPositionCommand(double position) {
        return Commands.runOnce(() -> setTargetPosition(position));
    }

    public Command stopCommand() {
        return Commands.sequence(
            setEnabledCommand(false)
        );
    }

    public Command setPivotAngleCommand(double pivotAngle) {
        return Commands.runOnce(() -> setPivotAngle(pivotAngle));
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
                tab.addNumber("Elevator Current Velocity", () -> elevatorMotor.getVelocity().getValueAsDouble());
                tab.addNumber("Elevator Desired Position", () -> desiredPosition);
                tab.addNumber("Elevator Current Position", () -> elevatorMotor.getPosition().getValueAsDouble());
                tab.addBoolean("Elevator Enabled", () -> enabled);
                tab.addBoolean("At position", () -> atPosition());
                tab.addNumber("Supply Current", () -> elevatorMotor.getSupplyCurrent().getValueAsDouble());
                tab.addNumber("Applied Voltage", () -> elevatorMotor.getMotorVoltage().getValueAsDouble());    

                break;
        }
    }

}
