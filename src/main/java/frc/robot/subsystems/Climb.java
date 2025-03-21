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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.NerdyMath;

public class Climb extends SubsystemBase implements Reportable{
    private final TalonFX motor;
    private final TalonFXConfigurator motorConfigurator;

    private final MotionMagicVoltage motionMagicRequest; // Was 0 during initialization
    private final NeutralOut neutralRequest = new NeutralOut();
    private NeutralModeValue neutralMode = NeutralModeValue.Brake;

    private double desiredPosition; // Should be ~90 or wherever initial position is
    private boolean enabled = true;
    private double desiredVoltage = 0;
    public TalonFXConfiguration motorConfigs;

    public Climb() {
        motor = new TalonFX(ClimbConstants.kMotorID);
        motorConfigurator = motor.getConfigurator();
        
        desiredPosition = 0;
        motionMagicRequest = new MotionMagicVoltage(desiredPosition);
        motor.setControl(neutralRequest);

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
        motorConfigs.Feedback.SensorToMechanismRatio = 0.5; 
        // motorConfigs.Feedback.RotorToSensorRatio;
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 45;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 45;
        motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
        motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
        motorConfigs.Slot0.kP = ClimbConstants.kPMotor;
        motorConfigs.Slot0.kI = ClimbConstants.kIMotor;
        motorConfigs.Slot0.kD = ClimbConstants.kDMotor;
        motorConfigs.Slot0.kV = ClimbConstants.kVMotor;
        motorConfigs.Slot0.kS = ClimbConstants.kSMotor;
        motorConfigs.Slot0.kG = ClimbConstants.kGMotor;

        motorConfigs.MotionMagic.MotionMagicCruiseVelocity =  ClimbConstants.kCruiseVelocity;
        motorConfigs.MotionMagic.MotionMagicAcceleration = ClimbConstants.kAcceleration;
        motorConfigs.MotionMagic.MotionMagicJerk = ClimbConstants.kJerk;
        motorConfigs.MotorOutput.NeutralMode = neutralMode;
    
        StatusCode response = motorConfigurator.apply(motorConfigs);
        if (!response.isOK()){
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
        }
    }

    @Override
    public void periodic() {
        if (!enabled) {
            motor.setControl(neutralRequest);
            return;
        } 
        else {
            motor.setVoltage(desiredVoltage);  
        }
    }

    // ****************************** STATE METHODS ****************************** //
    private void setVoltage(double volt) {
        desiredVoltage = volt;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void stopMotion() {
        motor.setControl(neutralRequest);
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        this.neutralMode = neutralMode;
    }
    
    public void setTargetPosition(double position) {
        //TODO NerdyMath.clamp(
        desiredPosition = position;
        motionMagicRequest.Position = desiredPosition;
    }

    public void zeroEncoder() {
        motor.setPosition(0);
        desiredPosition = 0;
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
                                desiredPosition - 0.05,
                                desiredPosition + 0.05);
    }

    // ****************************** COMMAND METHODS ****************************** //

    public Command setVoltageCommand(double volt) {
        return Commands.runOnce(() -> setVoltage(volt));
    }
    
    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    public Command setPositionCommand(double position) {
        return Commands.sequence(
            Commands.runOnce(() -> setTargetPosition(position))
        );
    }

    private Command stopCommand() {
        return Commands.sequence(
            setEnabledCommand(false),
            Commands.runOnce(() -> motor.setControl(neutralRequest))
        );
    }

    // ****************************** NAMED COMMANDS ****************************** //

    public TalonFX getMotor()
    {
        return motor;
    }

    public Command open() {
        return setPositionCommand(ClimbConstants.kOpenPosition);
    }

    public Command close() {
        return setPositionCommand(ClimbConstants.kClosedPosition);
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
                SmartDashboard.putNumber("Climb Position", motor.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("Climb Current", motor.getStatorCurrent().getValueAsDouble());
            case MEDIUM:
            case MINIMAL:
                break;
        }
    }
    
    @Override
    public void initShuffleboard(LOG_LEVEL level) { 
        if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Climb");
        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addString("Control Mode", motor.getControlMode()::toString);
                tab.addNumber("Climb FF", () -> motionMagicRequest.FeedForward);
                tab.addDouble("MM Position", () -> motionMagicRequest.Position);
                tab.addDouble("Desired Position", () -> desiredPosition);
                tab.addBoolean("At position", () -> atPosition());
                tab.addNumber("Current Climb Angle", () -> motor.getPosition().getValueAsDouble());
                tab.addBoolean("Enabled", () -> enabled);
            case MEDIUM:
                tab.addDouble("Supply Current", () -> motor.getSupplyCurrent().getValueAsDouble());
            case MINIMAL:
                tab.addDouble("Motor Temp", () -> motor.getDeviceTemp().getValueAsDouble());
                tab.addNumber("Climb Voltage", () -> motor.getMotorVoltage().getValueAsDouble());
                break;
        }
    }

}
