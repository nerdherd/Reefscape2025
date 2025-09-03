package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.sims.mechanisms.Mechanator;
import frc.robot.sims.simulations.ElevatorSimulation;
import frc.robot.util.NerdyMath;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorMotor;
    // private final TalonFX elevatorMotor2;

    // private final PIDController elevatorPID;
    private double desiredPosition = 0.0; // speed
    private boolean enabled = true;
    // private TalonFXConfigurator motorConfigurator;
    // private TalonFXConfigurator motorConfigurator2;
    private MotionMagicVoltage motionMagicRequest; // velocityvoltage
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    // private final Follower followRequest;
    private final NeutralOut neutralRequest = new NeutralOut();
    // private double ff = 0.0; 
    // private double pivotAngle = 0.0; // TODO: Change this to 0 when supersystem tuned
    
    private NeutralModeValue neutralMode = NeutralModeValue.Brake;

    private ElevatorSimulation elevatorSimulation;

    public Elevator() {
        elevatorMotor = new TalonFX(0);
        // elevatorMotor2 = new TalonFX(ElevatorConstants.kElevatorMotorID2, "rio");
        motionMagicRequest = new MotionMagicVoltage(0); // velocityvoltage
        
        elevatorMotor.setPosition(0.0);

        // motorConfigurator = elevatorMotor.getConfigurator(); // moved to setmotorconfigs
        // motorConfigurator2 = elevatorMotor2.getConfigurator();
        
        setMotorConfigs();
        
        // followRequest = new Follower(ElevatorConstants.kElevatorMotorID, true);
        motionMagicRequest.withSlot(0);
        zeroEncoder();
        CommandScheduler.getInstance().registerSubsystem(this);
        MechanismLigament2d ligament = Mechanator.getInstance().getLigament("elevator", 1.5, 1.5, 0.8, 90.0);
        elevatorSimulation = new ElevatorSimulation(elevatorMotor, ligament, 10.0, 0.5, 0.0508, 0.1, 1, 0);
    }
    
    public void setMotorConfigs() {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfigs.Feedback.SensorToMechanismRatio = 16; 
        motorConfigs.Feedback.RotorToSensorRatio = 1;

        motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 45;
        motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        motorConfigs.MotorOutput.NeutralMode = neutralMode;
        motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // motorConfigs.MotionMagic.MotionMagicCruiseVelocity =  ElevatorConstants.kElevatorCruiseVelocity;
        // motorConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kElevatorCruiseAcceleration;
        // motorConfigs.MotionMagic.MotionMagicJerk = ElevatorConstants.kElevatorJerk;

        // motorConfigs.Slot0.kP = 0;
        // motorConfigs.Slot0.kG = 0;
        // motorConfigs.Slot0.kS = 0;

        StatusCode response = elevatorMotor.getConfigurator().apply(motorConfigs);
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
        motorConfigs2.Slot0.kG = 0;
        motorConfigs2.Slot0.kS = 0;

        StatusCode response2 = motorConfigurator2.apply(motorConfigs2);
        if (!response2.isOK()){
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
        }
    }


    @Override
    public void periodic() {
        if (!enabled) {
            return;
        }
        
        // motionMagicRequest.Position = desiredPosition;
        velocityVoltage.Velocity = desiredPosition; // but with speed

        // ff = ElevatorConstants.kGElevatorMotor * Math.sin(pivotAngle * 2 * Math.PI);
        elevatorMotor.setControl(motionMagicRequest);
        // elevatorMotor2.setControl(followRequest); 
    }

    // ****************************** STATE METHODS ****************************** //

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if(!enabled) { 
            elevatorMotor.setControl(neutralRequest);
        }
    }

    // public void setNeutralMode(NeutralModeValue neutralMode) {
    //     this.neutralMode = neutralMode;
    // }

    // public void stopMotion() {
    //     elevatorMotor.setControl(neutralRequest);
    //     elevatorMotor2.setControl(neutralRequest);
    // }
    
    public void setTargetPosition(double position) { // but speed
        //TODO NerdyMath.clamp(
        desiredPosition = position;
    }

    // public void setPivotAngle(double pivotAngle) {
    //     this.pivotAngle = pivotAngle;
    // }

    // public void zeroEncoder() {
    //     elevatorMotor.setPosition(0.0);
    //     elevatorMotor2.setPosition(0.0);
    //     desiredPosition = 0.0;
    // }

    // ****************************** GET METHODS ***************************** //

    public double getPosition() { // but speed
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    public boolean atPosition() { // but speed
        return elevatorMotor.getPosition().getValueAsDouble() > desiredPosition;
    }

    // public boolean atPositionWide() {
    //     return NerdyMath.inRange(elevatorMotor.getPosition().getValueAsDouble(), 
    //     desiredPosition - 0.125,
    //     desiredPosition + 0.125);

    // }

    public double getTargetPosition() { // but speed
        return desiredPosition;
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

    // @Override
    // public void reportToSmartDashboard(LOG_LEVEL level) {
    //     switch (level) {
    //         case OFF:
    //             break;
    //         case ALL:
    //         case MEDIUM:
    //         SmartDashboard.putNumber("Elevator Desired Position", desiredPosition);
    //         SmartDashboard.putBoolean("Elevator Enabled", this.enabled);
    //         case MINIMAL:
    //             SmartDashboard.putNumber("Elevator Current Position", elevatorMotor.getPosition().getValueAsDouble());
    //             SmartDashboard.putNumber("Elevator Current Velocity", elevatorMotor.getVelocity().getValueAsDouble());
    //     }
    // }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        if (priority == LOG_LEVEL.OFF) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
        switch (priority) {
            case OFF:
                break;
            case ALL:
                tab.addString("Elevator Control Mode", elevatorMotor.getControlMode()::toString);
                tab.addBoolean("Elevator At Position", () -> atPosition());
                tab.addNumber("Elevator FF", () -> motionMagicRequest.FeedForward);
            case MEDIUM:
                tab.addNumber("Elevator Supply Current", () -> elevatorMotor.getSupplyCurrent().getValueAsDouble());
                tab.addNumber("Elevator Desired Position", ()-> motionMagicRequest.Position);
            case MINIMAL:
                tab.addNumber("Elevator Temperature 1", () -> elevatorMotor.getDeviceTemp().getValueAsDouble());
                tab.addNumber("Elevator Temperature 2", () -> elevatorMotor2.getDeviceTemp().getValueAsDouble());
                tab.addNumber("Elevator Current Position", () -> getPosition());
                tab.addNumber("Elevator Voltage", () -> elevatorMotor.getMotorVoltage().getValueAsDouble());
                break;
            }        
    }

}
