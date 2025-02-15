package frc.robot.subsystems;
 
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.Constants.IntakeConstants;
 
public class AlgaeRoller extends SubsystemBase implements Reportable {
    private final TalonFX rollerMotor;
    private final TalonFXConfigurator rollerConfigurator;

 
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final NeutralOut brakeRequest = new NeutralOut();

    private boolean enabled = false;
    private boolean velocityControl = true;

    public AlgaeRoller(Orchestra orchestra) {
        rollerMotor = new TalonFX(IntakeConstants.kRollerMotorID);
        rollerConfigurator = rollerMotor.getConfigurator();
        velocityRequest.EnableFOC = true;
        velocityRequest.Acceleration = 0;
        velocityRequest.FeedForward = 0;
        velocityRequest.Slot = 0;
        velocityRequest.OverrideBrakeDurNeutral = false;
        velocityRequest.LimitForwardMotion = false;
        velocityRequest.LimitReverseMotion = false;

        CommandScheduler.getInstance().registerSubsystem(this);
 
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        configureMotor(motorConfigs);
        configurePID(motorConfigs);
        orchestra.addInstrument(rollerMotor);
    }

    //****************************** SETUP METHODS ******************************//
 
    private void configureMotor(TalonFXConfiguration motorConfigs) {
        rollerConfigurator.refresh(motorConfigs);

        

        motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfigs.Voltage.PeakForwardVoltage = 11.5;
        motorConfigs.Voltage.PeakReverseVoltage = -11.5;
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfigs.MotorOutput.DutyCycleNeutralDeadband = IntakeConstants.kRollerNeutralDeadband;
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        motorConfigs.CurrentLimits.StatorCurrentLimit = 100;
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.Audio.AllowMusicDurDisable = true;
 
        StatusCode response = rollerConfigurator.apply(motorConfigs);
        if (!response.isOK())
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
    }
 
    private void configurePID(TalonFXConfiguration motorConfigs) {
        rollerConfigurator.refresh(motorConfigs);

        IntakeConstants.kPRollerMotor.loadPreferences();
        IntakeConstants.kIRollerMotor.loadPreferences();
        IntakeConstants.kDRollerMotor.loadPreferences();
        IntakeConstants.kVRollerMotor.loadPreferences();

        motorConfigs.Slot0.kP = IntakeConstants.kPRollerMotor.get();
        motorConfigs.Slot0.kI = IntakeConstants.kIRollerMotor.get();
        motorConfigs.Slot0.kD = IntakeConstants.kDRollerMotor.get();
        motorConfigs.Slot0.kV = IntakeConstants.kVRollerMotor.get();
 
        StatusCode response = rollerConfigurator.apply(motorConfigs);
        if (!response.isOK())
            DriverStation.reportError("Could not apply PID configs, error code:" + response.toString(), new Error().getStackTrace());
    }
 
    @Override
    public void periodic() {
        if (!enabled) {
            rollerMotor.setControl(brakeRequest);
        }
        else {
            rollerMotor.setControl(velocityRequest);          
        } 
    }
 
    // ****************************** STATE METHODS ***************************** //

    private void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    private void setVelocity(double velocity) {
        velocityRequest.Velocity = velocity;
    }

    private double getTargetVelocity() {
        return velocityRequest.Velocity;
    }

    // ****************************** COMMAND METHODS ****************************** //

    private Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }
 
    private Command setVelocityCommand(double velocity) {
        velocityRequest.Velocity = velocity;
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    private Command stopCommand() {
        return Commands.sequence(
            setVelocityCommand(0),
            setEnabledCommand(false)
        );
    }

    // ****************************** NAMED COMMANDS ****************************** //

    public Command intake() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommand(IntakeConstants.kIntakePower.get())
        );
    }

    public Command outtake() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommand(IntakeConstants.kOuttakePower.get())
        );
    }

    public Command stop() {
        return stopCommand();
    }
 
    // ****************************** LOGGING METHODS ****************************** //
 
    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}
 
    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Algae Roller");
        switch (priority) {
            case ALL:
            case MEDIUM:
                tab.addNumber("Position", () -> this.rollerMotor.getPosition().getValueAsDouble());
            case MINIMAL:
                tab.addBoolean("Algae Shooter Enabled", () -> this.enabled);
                tab.addBoolean("Algae Shooter Velocity Control", () -> this.velocityControl);
                tab.addNumber("Velocity", () -> rollerMotor.getVelocity().getValueAsDouble());
                tab.addNumber("NEW Velocity", () -> velocityRequest.Velocity);
                tab.addNumber("Target Velocity", () -> this.getTargetVelocity());
                tab.addNumber("Feed Forward", () -> velocityRequest.FeedForward);
                tab.addNumber("Supply Current", () -> this.rollerMotor.getSupplyCurrent().getValueAsDouble());
                tab.addNumber("Stator Current", () -> this.rollerMotor.getStatorCurrent().getValueAsDouble());
                tab.addNumber("Applied Voltage", () -> this.rollerMotor.getMotorVoltage().getValueAsDouble());    
                break;
            default:
                break;
        }
    }

}
 
 
