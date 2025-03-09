package frc.robot.subsystems;
 
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
import frc.robot.Constants.RollerConstants;
 
public class IntakeRoller extends SubsystemBase implements Reportable {
    private final TalonFX rollerMotor;
    private final TalonFX rollerMotorRight;
    private final TalonFXConfigurator rollerConfigurator;
    private final TalonFXConfigurator rollerConfiguratorRight;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VelocityVoltage velocityRequestRight = new VelocityVoltage(0);

    private final NeutralOut brakeRequest = new NeutralOut();

    private boolean enabled = false;
    private boolean velocityControl = true;

    private double desiredVoltageLeft = 0;
    private double desiredVoltageRight = 0;

    public IntakeRoller() {
        rollerMotor = new TalonFX(RollerConstants.kLeftMotorID);
        rollerMotorRight = new TalonFX(RollerConstants.kRightMotorID);
        rollerConfigurator = rollerMotor.getConfigurator();
        rollerConfiguratorRight = rollerMotorRight.getConfigurator();
        velocityRequestRight.EnableFOC = true;
        velocityRequestRight.Acceleration = 0;
        velocityRequestRight.FeedForward = 0;
        velocityRequestRight.Slot = 0;
        velocityRequestRight.OverrideBrakeDurNeutral = false;
        velocityRequestRight.LimitForwardMotion = false;
        velocityRequestRight.LimitReverseMotion = false;
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
    }

    //****************************** SETUP METHODS ******************************//
 
    private void configureMotor(TalonFXConfiguration motorConfigs) {
        rollerConfigurator.refresh(motorConfigs);
        motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfigs.Voltage.PeakForwardVoltage = 11.5;
        motorConfigs.Voltage.PeakReverseVoltage = -11.5;
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfigs.MotorOutput.DutyCycleNeutralDeadband = RollerConstants.kNeutralDeadband;
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        motorConfigs.CurrentLimits.StatorCurrentLimit = 100;
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.Audio.AllowMusicDurDisable = true;
 
        StatusCode response = rollerConfigurator.apply(motorConfigs);
        if (!response.isOK())
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
        rollerConfiguratorRight.refresh(motorConfigs);

        StatusCode responseRight = rollerConfiguratorRight.apply(motorConfigs);

        if (!responseRight.isOK())
        DriverStation.reportError("Could not apply motor configs, error code:" + responseRight.toString(), new Error().getStackTrace());

    }
 
    private void configurePID(TalonFXConfiguration motorConfigs) {
        rollerConfigurator.refresh(motorConfigs);

        motorConfigs.Slot0.kP = RollerConstants.kPMotor;
        motorConfigs.Slot0.kI = RollerConstants.kIMotor;
        motorConfigs.Slot0.kD = RollerConstants.kDMotor;
        motorConfigs.Slot0.kV = RollerConstants.kVMotor;

        motorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = 100;
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
 
        StatusCode response = rollerConfigurator.apply(motorConfigs);
        if (!response.isOK())
            DriverStation.reportError("Could not apply PID configs, error code:" + response.toString(), new Error().getStackTrace());
        rollerConfiguratorRight.refresh(motorConfigs);
        StatusCode responseRight = rollerConfiguratorRight.apply(motorConfigs);
        if (!response.isOK())
            DriverStation.reportError("Could not apply PID configs, error code:" + responseRight.toString(), new Error().getStackTrace());


    }
 
    @Override
    public void periodic() {
        if (!enabled) {
            rollerMotor.setControl(brakeRequest);
            rollerMotorRight.setControl(brakeRequest);
        }
        else {
            rollerMotor.setVoltage(desiredVoltageLeft);  
            rollerMotorRight.setVoltage(desiredVoltageRight);
        } 

    }
 
    // ****************************** STATE METHODS ***************************** //

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    private void setVelocity(double velocity) {
        velocityRequest.Velocity = velocity;
    }

    private double getTargetVelocity() {
        return velocityRequest.Velocity;
    }

    private void setVoltage(double volt) {
        desiredVoltageLeft = volt;
        desiredVoltageRight = -volt;
    }

    // ****************************** COMMAND METHODS ****************************** //

    private Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }
 
    private Command setVelocityCommand(double velocity) {
        velocityRequest.Velocity = velocity;
        velocityRequestRight.Velocity = -velocity;
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    private Command setVelocityCommandLeft(double velocity) {
        velocityRequest.Velocity = velocity;
        return Commands.runOnce(() -> setVelocity(velocity));
    }
    private Command setVelocityCommandRight(double velocity) {
        velocityRequestRight.Velocity = -velocity;
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public Command setVoltageCommand(double volt) {
        return Commands.runOnce(() -> setVoltage(volt));
    }

    public Command setVoltageCommandLeft(double volt) {
        return Commands.runOnce(() -> setVoltage(volt));
    }

    public Command setVoltageCommandRight(double volt) {
        return Commands.runOnce(() -> setVoltage(volt));
    }

    private Command stopCommand() {
        return Commands.sequence(
            setVelocityCommand(0),
            setEnabledCommand(false)
        );
    }

    // ****************************** NAMED COMMANDS ****************************** //
    public Command intakeAlgae() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommandLeft(RollerConstants.kIntakePower),
            setVelocityCommandRight(RollerConstants.kIntakePower)
        );
    }

    public Command intakeLeft() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommandLeft(RollerConstants.kIntakePower)
        );
    }

    public Command intakeCoral() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommandRight(RollerConstants.kIntakePower)
        );
    }

    public Command outtake() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommand(RollerConstants.kOuttakePower)
        );
    }

    public Command outtakeL1() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommandLeft(RollerConstants.kOuttakePower)
            
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
 
 
