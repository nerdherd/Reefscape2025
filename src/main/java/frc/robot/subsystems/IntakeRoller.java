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
    private final TalonFX algaeMotor;
    private final TalonFX coralMotor;
    private final TalonFXConfigurator rollerConfigurator;
    private final TalonFXConfigurator rollerConfiguratorRight;
    private final VelocityVoltage velocityRequestAlgae = new VelocityVoltage(0);
    private final VelocityVoltage velocityRequestCoral = new VelocityVoltage(0);
    public TalonFXConfiguration motorConfigs;

    private final NeutralOut brakeRequest = new NeutralOut();

    private boolean enabled = false;
    private boolean velocityControl = true;

    private double desiredVoltageAlgae = 0;
    private double desiredVoltageCoral = 0;

    public IntakeRoller() {
        algaeMotor = new TalonFX(RollerConstants.kAlgaeMotorID);
        coralMotor = new TalonFX(RollerConstants.kCoralMotorID);
        rollerConfigurator = algaeMotor.getConfigurator();
        rollerConfiguratorRight = coralMotor.getConfigurator();
        velocityRequestCoral.EnableFOC = true;
        velocityRequestCoral.Acceleration = 0;
        velocityRequestCoral.FeedForward = 0;
        velocityRequestCoral.Slot = 0;
        velocityRequestCoral.OverrideBrakeDurNeutral = false;
        velocityRequestCoral.LimitForwardMotion = false;
        velocityRequestCoral.LimitReverseMotion = false;
        velocityRequestAlgae.EnableFOC = true;
        velocityRequestAlgae.Acceleration = 0;
        velocityRequestAlgae.FeedForward = 0;
        velocityRequestAlgae.Slot = 0;
        velocityRequestAlgae.OverrideBrakeDurNeutral = false;
        velocityRequestAlgae.LimitForwardMotion = false;
        velocityRequestAlgae.LimitReverseMotion = false;

        CommandScheduler.getInstance().registerSubsystem(this);
 
        motorConfigs = new TalonFXConfiguration();
        configureMotor(motorConfigs);
        configurePID(motorConfigs);
    }

    //****************************** SETUP METHODS ******************************//
 
    public void configureMotor(TalonFXConfiguration motorConfigs) {
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

        motorConfigs.CurrentLimits.SupplyCurrentLimit = 20; //arbitrarayay 
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = 50; //arbitrarayay 
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
            algaeMotor.setControl(brakeRequest);
            coralMotor.setControl(brakeRequest);
        }
        else {
            algaeMotor.setVoltage(desiredVoltageAlgae);  
            coralMotor.setVoltage(desiredVoltageCoral * 1.15);
        } 

    }
 
    // ****************************** STATE METHODS ***************************** //

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    private void setVelocity(double velocity) {
        velocityRequestAlgae.Velocity = velocity;
        velocityRequestCoral.Velocity = velocity;
    }
    
    private void setVelocityCoral(double velocity) {
        velocityRequestCoral.Velocity = velocity;
    }

    private void setVelocityAlgae(double velocity) {
        velocityRequestAlgae.Velocity = velocity;
    }

    private double getTargetVelocity() {
        return velocityRequestAlgae.Velocity;
    }
    private void setVoltage(double volt) {
        desiredVoltageCoral = volt;
        desiredVoltageAlgae = volt;
    }

    private void setVoltageCoral(double volt) {
        desiredVoltageCoral = volt;
    }

    private void setVoltageAlgae(double volt) {
        desiredVoltageAlgae = volt;
    }

    // ****************************** COMMAND METHODS ****************************** //

    private Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }
 
    private Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    private Command setVelocityCommandAlgae(double velocity) {
        return Commands.runOnce(() -> setVelocityAlgae(velocity));
    }
    private Command setVelocityCommandCoral(double velocity) {
        return Commands.runOnce(() -> setVelocityCoral(velocity));
    }

    public Command setVoltageCommand(double volt) {
        return Commands.runOnce(() -> setVoltage(volt));
    }

    public Command setVoltageCommandCoral(double volt) {
        return Commands.runOnce(() -> setVoltageCoral(volt));
    }

    public Command setVoltageCommandAlgae(double volt) {
        return Commands.runOnce(() -> setVoltageAlgae(volt));
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
            setVelocityCommandAlgae(RollerConstants.kAlgaeIntakePower)
        );
    }

    public Command intakeCoral() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommandCoral(RollerConstants.kCoralIntakePower)
        );
    }

    public Command outtakeCoral() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommandCoral(RollerConstants.kCoralOuttakePower)
        );
    }

    public Command outtakeAlgae() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommandAlgae(RollerConstants.kAlgaeOuttakePower)
        );
    }

    public Command outtakeL1() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommand(RollerConstants.kL1OuttakePower)
            
        );
    }

    public Command outtake() {
        return Commands.sequence(
          setEnabledCommand(true),
          Commands.parallel(
            outtakeAlgae(),
            outtakeCoral()
          )  
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
        ShuffleboardTab tab = Shuffleboard.getTab("Intake Roller");
        switch (priority) {
            case ALL:
                tab.addNumber("Intake Stator Current", () -> this.algaeMotor.getStatorCurrent().getValueAsDouble());
                tab.addBoolean("Intake Velocity Control", () -> this.velocityControl);
                tab.addNumber("Intake Desired Velocity", () -> velocityRequestAlgae.Velocity);
                tab.addNumber("Intake Position", () -> this.algaeMotor.getPosition().getValueAsDouble());
                tab.addNumber("Intake Velocity", () -> algaeMotor.getVelocity().getValueAsDouble());
            case MEDIUM:
                tab.addNumber("Intake Supply Current", () -> this.algaeMotor.getSupplyCurrent().getValueAsDouble());
                tab.addBoolean("Intake Enabled", () -> this.enabled);
            case MINIMAL:
                tab.addNumber("Intake Applied Voltage Right", () -> this.coralMotor.getMotorVoltage().getValueAsDouble());    
                tab.addNumber("Intake Applied Voltage Left", () -> this.algaeMotor.getMotorVoltage().getValueAsDouble());    
                tab.addNumber("Intake Temperature", () -> this.algaeMotor.getDeviceTemp().getValueAsDouble());    
                break;
            default:
                break;
        }
    }

}
 
 
