package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.util.NerdyMath;;

public class Wrist extends SubsystemBase implements Reportable{
    private final TalonFX motor;
    private final TalonFXConfigurator motorConfigurator;
    private Pigeon2 pigeon;

    private final MotionMagicVoltage motionMagicRequest; // Was 0 during initialization
    private final NeutralOut brakeRequest = new NeutralOut();

    private double desiredPosition; // Should be ~90 or wherever initial position is
    private double desiredAngle;
    private boolean enabled = false;
    private double ff = 0;
    private double pivotAngle = 0;
    public TalonFXConfiguration motorConfigs;

    private NeutralModeValue neutralMode = NeutralModeValue.Brake;

    public Wrist() {
        motor = new TalonFX(WristConstants.kMotorID);
        motorConfigurator = motor.getConfigurator();
        
        // pigeon = new Pigeon2(WristConstants.kPigeonID, "rio");
        desiredPosition = 0;
        // desiredPosition = pigeon.getRoll().getValueAsDouble();
        motionMagicRequest = new MotionMagicVoltage(desiredPosition);
        motor.setControl(brakeRequest);

        // configure motor
         motorConfigs = new TalonFXConfiguration();
        configurePID(motorConfigs);
        
        zeroEncoder();
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    //****************************** SETUP METHODS ******************************//

    public void configurePID(TalonFXConfiguration motorConfigs) {
        motorConfigurator.refresh(motorConfigs);

        // motorConfigs.Feedback.FeedbackRemoteSensorID = V1IntakeConstants.kPigeonID;
        motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfigs.Feedback.SensorToMechanismRatio = 13.89; 
        // motorConfigs.Feedback.RotorToSensorRatio;
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 45;
        motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
        motorConfigs.Slot0.kP = WristConstants.kPMotor;
        motorConfigs.Slot0.kI = WristConstants.kItMotor;
        motorConfigs.Slot0.kD = WristConstants.kDMotor;
        motorConfigs.Slot0.kV = WristConstants.kVMotor;
        motorConfigs.Slot0.kS = WristConstants.kSMotor;
        motorConfigs.Slot0.kG = WristConstants.kGMotor;

        motorConfigs.MotionMagic.MotionMagicCruiseVelocity =  WristConstants.kCruiseVelocity;
        motorConfigs.MotionMagic.MotionMagicAcceleration = WristConstants.kAcceleration;
        motorConfigs.MotionMagic.MotionMagicJerk = WristConstants.kJerk;
    
        motorConfigs.MotorOutput.NeutralMode = neutralMode;

        StatusCode response = motorConfigurator.apply(motorConfigs);
        if (!response.isOK()){
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
        }
    }

    @Override
    public void periodic() {
        if(!enabled) {
            motor.setControl(brakeRequest);
            return;
        }

        // desiredPosition + pivot * constantToChangeUnit
        // ff = (-3.2787 * desiredPosition) - 1.5475; Harder method
        ff = WristConstants.kGMotor * Math.cos((getPosition() + 0.33 + pivotAngle) * 2 * Math.PI); // 0.5437 is wrist horizontal 
        motor.setControl(motionMagicRequest.withFeedForward(ff));
    }

    // ****************************** STATE METHODS ****************************** //

    public void setEnabled(boolean e) {
        this.enabled = e;
    }

    public void stopMotion() {
        motor.setControl(brakeRequest);
    }
    
    public void setNeutralMode(NeutralModeValue neutralMode) {
        this.neutralMode = neutralMode;
    }
    
    public void setTargetPosition(double position) {
        //TODO NerdyMath.clamp(
        desiredPosition = position;
        motionMagicRequest.Position = desiredPosition;
    }

    // public void setPositionDegrees(double positionDegrees) {
    //     desiredAngle = positionDegrees;
    //     desiredPosition = (desiredAngle * 0.015432) - 0.00011; // Depricated values
        
    //     // double newPos = NerdyMath.clamp(
    //     //     positionDegrees, 

    //     // );

    //     motionMagicRequest.Position = (desiredPosition);  // = (newPos / 360.0)
    // }

    public void zeroEncoder() {
        motor.setPosition(0);
        desiredPosition = 0;
    }

    public void setPivotAngle(double pivotAngle) {
        this.pivotAngle = pivotAngle;
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public boolean atPosition() {
        return NerdyMath.inRange(motor.getPosition().getValueAsDouble(), 
                                desiredPosition - 0.05,
                                desiredPosition + 0.05);
    }

    public boolean atPositionWide() {
        return NerdyMath.inRange(motor.getPosition().getValueAsDouble(), 
                                desiredPosition - 0.15,
                                desiredPosition + 0.15);
    }

    // ****************************** COMMAND METHODS ****************************** //
    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> this.setEnabled(enabled));
    }

    public Command setPositionCommand(double position) {
        return Commands.sequence(
            Commands.runOnce(() -> setTargetPosition(position))
        );
    }

    public Command stopCommand() {
        return Commands.sequence(
            setEnabledCommand(false),
            Commands.runOnce(() -> motor.setControl(brakeRequest))
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
                SmartDashboard.putNumber("Coral Wrist Position", motor.getPosition().getValueAsDouble());
            case MEDIUM:
                SmartDashboard.putNumber("Coral Wrist Current", motor.getStatorCurrent().getValueAsDouble());
            case MINIMAL:
                break;
        }
    }
    
    @Override
    public void initShuffleboard(LOG_LEVEL priority) { 
        if (priority == LOG_LEVEL.OFF) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Intake Wrist");
        switch (priority) {
            case OFF:
                break;
            case ALL:
                tab.addString("Wrist Control Mode", motor.getControlMode()::toString);
                tab.addBoolean("Wrist At Position", () -> atPosition());
                tab.addNumber("Wrist FF", () -> motionMagicRequest.FeedForward);
            case MEDIUM:
                tab.addNumber("Wrist Supply Current", () -> motor.getSupplyCurrent().getValueAsDouble());
            case MINIMAL:
                tab.addNumber("Wrist Temperature", () -> motor.getDeviceTemp().getValueAsDouble());
                tab.addNumber("Wrist Desired Position", () -> desiredPosition);
                tab.addNumber("Wrist Current Position", () -> motor.getPosition().getValueAsDouble());
                tab.addNumber("Wrist Voltage", () -> motor.getMotorVoltage().getValueAsDouble());
                break;
        }
    }
}