package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    private final TalonFX motor = new TalonFX(ClimbConstants.kMotorID);
    
    private final NeutralOut neutralRequest = new NeutralOut();

    private double desiredVoltage = 0.0;
    private boolean enabled = false;

    public ClimbV2() {
        motor.setControl(neutralRequest);
        configurePID();
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    //****************************** SETUP METHODS ******************************//

    public void configurePID() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configs.Feedback.SensorToMechanismRatio = 0.5; 
        configs.CurrentLimits.SupplyCurrentLimit = 45;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLowerTime = 0;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
        // configs.Slot0.kP = ClimbConstants.kPMotor;
        // configs.Slot0.kI = ClimbConstants.kIMotor;
        // configs.Slot0.kD = ClimbConstants.kDMotor;
        // configs.Slot0.kV = ClimbConstants.kVMotor;
        // configs.Slot0.kS = ClimbConstants.kSMotor;

        // configs.MotionMagic.MotionMagicCruiseVelocity =  ClimbConstants.kCruiseVelocity;
        // configs.MotionMagic.MotionMagicAcceleration = ClimbConstants.kAcceleration;
        // configs.MotionMagic.MotionMagicJerk = ClimbConstants.kJerk;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
        StatusCode response = motor.getConfigurator().apply(configs);
        if (!response.isOK()){
            DriverStation.reportError("Could not apply left motor configs, error code: " + response.toString(), true);
        }
    }

    @Override
    public void periodic() {
        if (!enabled) {
            stopMotion();
            return;
        }
        
        motor.setVoltage(desiredVoltage);  
    }

    // ****************************** STATE METHODS ****************************** //
    
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            desiredVoltage = 0.0;
            motor.setControl(neutralRequest);
            motor.setVoltage(0.0);
        }
    }

    public void stopMotion() {
        motor.setControl(neutralRequest);
        motor.setVoltage(0.0);
    }

    private void setVoltage(double voltage) {
        desiredVoltage = voltage;
    }

    public double getSpeed() {
        return motor.getVelocity().getValueAsDouble();
    }

    public boolean atSpeed() {
        return getSpeed() > desiredVoltage;
    }

    // ****************************** COMMAND METHODS ****************************** //
    
    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    public Command setVoltageCommand(double speed) {
        return Commands.runOnce(() -> setVoltage(speed));
    }

    // ****************************** NAMED COMMANDS ****************************** //

    public TalonFX getMotor() {
        return motor;
    }

    public Command startClimb() {
        return setVoltageCommand(ClimbConstants.kOpenSpeed); // TODO find climb speed
    }

    public Command startClimbGrip() {
        return setVoltageCommand(ClimbConstants.kGripSpeed); // TODO find climb speed
    }

    public Command stopClimb() {
        return setVoltageCommand(0.0);
    }

    // ****************************** LOGGING METHODS ****************************** //
    
    @Override
    public void initShuffleboard(LOG_LEVEL level) { 
        ShuffleboardTab tab = Shuffleboard.getTab("Climb");
        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addString("Control Mode", motor.getControlMode()::toString);
                tab.addDouble("Supply Current", () -> motor.getSupplyCurrent().getValueAsDouble());
                tab.addDouble("Desired Speed", () -> desiredVoltage);
                tab.addBoolean("Enabled", () -> enabled);
            case MEDIUM:
                tab.addBoolean("At Speed", () -> atSpeed());
            case MINIMAL:
                tab.addDouble("Motor Temp", () -> motor.getDeviceTemp().getValueAsDouble());
                tab.addNumber("Motor Voltage", () -> motor.getMotorVoltage().getValueAsDouble());
                break;
        }
    }

}
