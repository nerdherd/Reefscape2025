package frc.robot.subsystems;
 
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
 
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.NerdyMath;
 
public class AlgaeRoller extends SubsystemBase implements Reportable {
    private final TalonFX shooter;
    private final TalonFXConfigurator shooterConfigurator;
 
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
   
    private boolean enabled = true;
    public boolean velocityControl = true;
 
    public AlgaeRoller() {
        shooter = new TalonFX(AlgaeConstants.kRollerMotorID);
        shooterConfigurator = shooter.getConfigurator();
        voltageRequest.EnableFOC = true;

        CommandScheduler.getInstance().registerSubsystem(this);
 
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        configureMotor(motorConfigs);
        configurePID(motorConfigs);
    }

    //****************************** SETUP METHODS ******************************//
 
    public void configureMotor(TalonFXConfiguration motorConfigs) {
        shooterConfigurator.refresh(motorConfigs);

        motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfigs.Voltage.PeakForwardVoltage = 11.5;
        motorConfigs.Voltage.PeakReverseVoltage = -11.5;
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfigs.MotorOutput.DutyCycleNeutralDeadband = AlgaeConstants.kRollerNeutralDeadband;
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        // leftMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        // leftMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        motorConfigs.CurrentLimits.StatorCurrentLimit = 100;
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.Audio.AllowMusicDurDisable = true;
 
        StatusCode response = shooterConfigurator.apply(motorConfigs);
        if (!response.isOK())
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
    }
 
    public void configurePID(TalonFXConfiguration motorConfigs) {
        shooterConfigurator.refresh(motorConfigs);

        AlgaeConstants.kPRollerMotor.loadPreferences();
        AlgaeConstants.kIRollerMotor.loadPreferences();
        AlgaeConstants.kDRollerMotor.loadPreferences();
        AlgaeConstants.kVRollerMotor.loadPreferences();
        motorConfigs.Slot0.kP = AlgaeConstants.kPRollerMotor.get();
        motorConfigs.Slot0.kI = AlgaeConstants.kIRollerMotor.get();
        motorConfigs.Slot0.kD = AlgaeConstants.kDRollerMotor.get();
        motorConfigs.Slot0.kV = AlgaeConstants.kVRollerMotor.get();
 
        StatusCode response = shooterConfigurator.apply(motorConfigs);
        if (!response.isOK())
            DriverStation.reportError("Could not apply PID configs, error code:" + response.toString(), new Error().getStackTrace());
    }
 
    @Override
    public void periodic() {
        if (!enabled) {
            velocityRequest.Velocity = 0;
            shooter.setControl(velocityRequest);
            return;
        }
 
        if (velocityControl) {
            shooter.setControl(velocityRequest);
            return;
        } 
 
        voltageRequest.Output = velocityRequest.Velocity * 12 / 100.0;
        shooter.setControl(voltageRequest);
    }
 
    //****************************** VELOCITY METHODS ******************************//
 
    public void setVelocity(double velocity) {
        // desiredVelocity = Math.min(Math.max(velocity, -ElevatorConstants.kElevatorSpeed), ElevatorConstants.kElevatorSpeed);
        shooter.set(velocity);
    }

    public double getVelocity() {
        return shooter.getVelocity().getValueAsDouble();
    }

    public double getTargetVelocity() {
        return velocityRequest.Velocity;
    }

    // public boolean atVelocity(double velocity) {
    //     return shooter.getVelocity().getValueAsDouble() > velocity;
    // }

    //****************************** COMMAND METHODS ******************************//

    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> {this.enabled = enabled;});
    }
 
    public Command setVelocityCommand(double velocity) {
        setEnabledCommand(true);
        Command command = Commands.runOnce(() -> setVelocity(velocity));
        command.addRequirements(this);
        return command;
    }

    //****************************** NAMED COMMANDS ******************************//

    public Command intake() {
        return setVelocityCommand(AlgaeConstants.kIntakePower);
    }

    public Command shootBarge() {
        return setVelocityCommand(AlgaeConstants.kBargeOuttake.get());
    }

    public Command shootProcessor() {
        return setVelocityCommand(AlgaeConstants.kProcessorOuttake.get());
    }

    public Command stop() {
        return setVelocityCommand(0);
    }
 
    //****************************** LOGGING METHODS ******************************//
 
    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}
 
    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

        switch (priority) {
            case ALL:
            case MEDIUM:
            tab.addNumber("Position", () -> shooter.getPosition().getValueAsDouble());
            case MINIMAL:
            tab.addBoolean("Algae Shooter Enabled", () -> this.enabled);
            tab.addNumber("Velocity", () -> getVelocity());
            tab.addNumber("Target Velocity", () -> velocityRequest.Velocity);
            tab.addNumber("Supply Current", () -> shooter.getSupplyCurrent().getValueAsDouble());
            tab.addNumber("Stator Current", () -> shooter.getStatorCurrent().getValueAsDouble());
            tab.addNumber("Applied Voltage", () -> shooter.getMotorVoltage().getValueAsDouble());    
                break;
        
            default:
                break;
        }
        
    }
}
 
 
