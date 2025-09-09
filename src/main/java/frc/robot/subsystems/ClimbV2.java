package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

public class ClimbV2 extends SubsystemBase implements Reportable {
    private final TalonFX motorLeft = new TalonFX(ClimbConstants.kLeftMotorID);
    private final TalonFX motorRight = new TalonFX(ClimbConstants.kRightMotorID);
    private final Follower followerRight = new Follower(ClimbConstants.kLeftMotorID, true);
    
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final NeutralOut neutralRequest = new NeutralOut();
    private NeutralModeValue neutralMode = NeutralModeValue.Brake;

    private double desiredSpeed = 0;
    private boolean enabled;

    public ClimbV2() {
        motorRight.setControl(followerRight);
        configurePID();
        zeroEncoders();

        setEnabled(true);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    //****************************** SETUP METHODS ******************************//

    public void configurePID() {
        TalonFXConfigurator configuratorLeft = motorLeft.getConfigurator();
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configuratorLeft.refresh(configs);

        configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configs.Feedback.SensorToMechanismRatio = 0.5; 
        configs.CurrentLimits.SupplyCurrentLimit = 45;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLowerTime = 0;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
        configs.Slot0.kP = ClimbConstants.kPMotor;
        configs.Slot0.kI = ClimbConstants.kIMotor;
        configs.Slot0.kD = ClimbConstants.kDMotor;
        configs.Slot0.kV = ClimbConstants.kVMotor;
        configs.Slot0.kS = ClimbConstants.kSMotor;
        configs.Slot0.kG = ClimbConstants.kGMotor;

        configs.MotionMagic.MotionMagicCruiseVelocity =  ClimbConstants.kCruiseVelocity;
        configs.MotionMagic.MotionMagicAcceleration = ClimbConstants.kAcceleration;
        configs.MotionMagic.MotionMagicJerk = ClimbConstants.kJerk;
        configs.MotorOutput.NeutralMode = neutralMode;
    
        StatusCode response = configuratorLeft.apply(configs);
        if (!response.isOK()){
            DriverStation.reportError("Could not apply left motor configs, error code: " + response.toString(), true);
        }
    }

    @Override
    public void periodic() {
        
    }

    // ****************************** STATE METHODS ****************************** //
    
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            desiredSpeed = 0;
            motorLeft.setControl(neutralRequest);
        } else {
            motorLeft.setControl(velocityRequest);
        }
    }
    
    public void zeroEncoders() {
        motorLeft.setPosition(0);
        motorRight.setPosition(0);
    }
    
    public void setNeutralMode(NeutralModeValue neutralMode) {
        this.neutralMode = neutralMode;
    }

    private void setSpeed(double speed) {
        desiredSpeed = speed;
        velocityRequest.Velocity = desiredSpeed;
    }

    public double getSpeed() {
        return motorLeft.getVelocity().getValueAsDouble();
    }

    public boolean atSpeed() {
        return getSpeed() > desiredSpeed;
    }

    // ****************************** COMMAND METHODS ****************************** //
    
    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    public Command setSpeedCommand(double speed) {
        return Commands.runOnce(() -> setSpeed(speed));
    }

    // ****************************** NAMED COMMANDS ****************************** //

    public TalonFX getMotorLeft() {
        return motorLeft;
    }

    public Command startClimb() {
        return setSpeedCommand(10); // TODO find climb speed
    }

    public Command stopClimb() {
        return setSpeedCommand(0);
    }

    // ****************************** LOGGING METHODS ****************************** //

    @Override
    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
                SmartDashboard.putNumber("Climb Velocity", motorLeft.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("Climb Current", motorLeft.getStatorCurrent().getValueAsDouble());
            case MEDIUM:
            case MINIMAL:
                break;
        }
    }
    
    @Override
    public void initShuffleboard(LOG_LEVEL level) { 
        ShuffleboardTab tab = Shuffleboard.getTab("Climb");

        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addString("Control Mode", motorLeft.getControlMode()::toString);
                tab.addDouble("Desired Speed", () -> desiredSpeed);
                tab.addBoolean("At Speed", () -> atSpeed());
                tab.addBoolean("Enabled", () -> enabled);
            case MEDIUM:
                tab.addDouble("Supply Current", () -> motorLeft.getSupplyCurrent().getValueAsDouble());
            case MINIMAL:
                tab.addDouble("Motor Temp", () -> motorLeft.getDeviceTemp().getValueAsDouble());
                tab.addNumber("Motor Voltage", () -> motorLeft.getMotorVoltage().getValueAsDouble());
                break;
        }
    }

}
