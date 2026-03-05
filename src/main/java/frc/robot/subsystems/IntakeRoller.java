package frc.robot.subsystems;

/** importing DriverStation, Shuffleboard, Motor Configs, etc. */
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
 
/** Defining Motors for Algae, and creating New Instances */
public class IntakeRoller extends SubsystemBase implements Reportable {
    private final TalonFX algaeMotor;
    private final TalonFX coralMotor;
    private final TalonFXConfigurator rollerConfigurator;
    private final TalonFXConfigurator rollerConfiguratorRight;
    private final VelocityVoltage velocityRequestAlgae = new VelocityVoltage(0);
    private final VelocityVoltage velocityRequestCoral = new VelocityVoltage(0);
    public TalonFXConfiguration motorConfigs;

    private final NeutralOut brakeRequest = new NeutralOut();

/** Enabling and Disabling Booleans, and Resetting Values to 0 */
    private boolean enabled = false;
    private boolean velocityControl = true;

    private double desiredVoltageAlgae = 0;
    public double desiredVoltageCoral = 0;

/**Creating new function IntakeRoller()
 * Enabling and Disabling Booleans, and Resetting Values to 0
 * Defining Algae and Coral Motor
 */
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
 
/**
 * Enabling and Disabling boolean values.
 * @param motorConfigs Resetting motorConfigs, and setting Voltage and Current values.
 */
    public void configureMotor(TalonFXConfiguration motorConfigs) {
        rollerConfigurator.refresh(motorConfigs);
        motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfigs.Voltage.PeakForwardVoltage = 11.5;
        motorConfigs.Voltage.PeakReverseVoltage = -11.5;
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfigs.MotorOutput.DutyCycleNeutralDeadband = RollerConstants.kNeutralDeadband;
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        motorConfigs.CurrentLimits.StatorCurrentLimit = 100;
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.Audio.AllowMusicDurDisable = true;

        /**
         * Calling if statements for different functions for errors in applying motorConfigs
         */
        StatusCode response = rollerConfigurator.apply(motorConfigs);
        if (!response.isOK())
            DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
        rollerConfiguratorRight.refresh(motorConfigs);

        StatusCode responseRight = rollerConfiguratorRight.apply(motorConfigs);

        if (!responseRight.isOK())
        DriverStation.reportError("Could not apply motor configs, error code:" + responseRight.toString(), new Error().getStackTrace());

    }
/**
 * Configuring PID settings,
 * @param motorConfigs Calling if statements for different functions for errors in configuring PID
 */
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
/** Overide if enabled statements*/
    @Override
    public void periodic() {
        if (!enabled) {
            return;
        }
/** Setting voltage for algae and coral motors */
        algaeMotor.setVoltage(desiredVoltageAlgae);  
        coralMotor.setVoltage(desiredVoltageCoral);
    }
 
    // ****************************** STATE METHODS ***************************** //
/**
 * Calling if statement, and setting values if true for algae and coral
 * @param enabled Defining setEnabled function and setting value
 */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            desiredVoltageAlgae = 0.0;
            desiredVoltageCoral = 0.0;
            algaeMotor.setControl(brakeRequest);
            coralMotor.setControl(brakeRequest);
        }
    }
/**
 * Making privare class and setting Voltage for algae and coral
 * @param velocity Making private class to set Velocity for algae and coral
 */
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

    /**
     * Setting Command function
     * @param enabled Setting EnabledCommand boolean
     * @return Returning value to run once
     */
    Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }
    /**
     * Making private class for setVelocityCommand for algae and coral
     * Making private class setVoltage Command for algae and coral
     * @param velocity Set VelocityCommand value, and create a new instance of it.  
     * @return returning value to run once
     */
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
    
    /**
     * Making public class stopCommand
     * Setting Voltage and Enabled Commands
     * @return return Commands values (should be off, and reset)
     */
    public Command stopCommand() {
        return Commands.sequence(
            setVoltageCommand(0),
            setEnabledCommand(false)
        );
    }

    // ****************************** NAMED COMMANDS ****************************** //
    /**
     * Making public class intakeAlgae
     * @return returning the setEnabledCommand and setVoltageCommandAlgae values
     */
    public Command intakeAlgae() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVoltageCommandAlgae(RollerConstants.kAlgaeIntakePower)
        );
    }
    /**
     * Making public class holdAlgae
     * @return returning setEnabledCommand and setVoltageCommandAlgae
     */
    public Command holdAlgae() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVoltageCommandAlgae(RollerConstants.kAlgaeHoldPower)
        );
    }
    /**
     * Making public class intakeCoral
     * @return returning setEnabledCommand, setVoltageCommandCoral, setVoltageCommandAlgae values
     */
    public Command intakeCoral() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVoltageCommandCoral(RollerConstants.kCoralIntakePower),
            setVoltageCommandAlgae(RollerConstants.kCoralButAlgaeIntakePower)
        );
    }
    /**
     * public class intakeCoralSlow
     * @return setEnabledCommand and setVoltageCommandCoral values
     */
    public Command intakeCoralSlow() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVoltageCommandCoral(RollerConstants.kCoralSlowIntakePower)
        );
    }
    /**
     * Making public class outtakeCoral
     * @return returning setEnabledCommand, setVoltageCommandCoral, and setVoltageCommandAlgae values
     */
    public Command outtakeCoral() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVoltageCommandCoral(RollerConstants.kCoralOuttakePower),
            setVoltageCommandAlgae(RollerConstants.kCoralOuttakePower)
        );
    }
    /**
     * Making public class outtakeAlgae
     * @return returning setEnabledCommand and setVoltageCommandAlgae values
     */ 
    public Command outtakeAlgae() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVoltageCommandAlgae(RollerConstants.kAlgaeOuttakePower)
        );
    }
    /**
     * Making public class outtakeL1
     * @return setEnabledCommand, setVoltageCommandCoral, setVoltageCommandAlgae values
     */
    public Command outtakeL1() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVoltageCommandCoral(RollerConstants.kL1OuttakePower),
            setVoltageCommandAlgae(-RollerConstants.kL1OuttakePower)
        );
    }
 
    // ****************************** LOGGING METHODS ****************************** //
  
    /**
     * Overide funtion, making a pulic class initShuffleboard and adding parameters
     * switch statement that has cases for ALL, MEDIUM, and MINIMAL priorities
     */
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
 
 
