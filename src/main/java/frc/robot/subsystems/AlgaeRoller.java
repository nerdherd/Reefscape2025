package frc.robot.subsystems;
 
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
 
public class AlgaeRoller extends SubsystemBase implements Reportable {
    private final TalonFX rollerMotor;
    private final TalonFXConfigurator rollerConfigurator;
 
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final NeutralOut brakeRequest = new NeutralOut();

    private boolean enabled = false;
    public boolean velocityControl = true;

    public AlgaeRoller() {
        rollerMotor = new TalonFX(AlgaeConstants.kRollerMotorID);
        rollerConfigurator = rollerMotor.getConfigurator();
        velocityRequest.EnableFOC = true;
        velocityRequest.Acceleration = 0;
        velocityRequest.FeedForward = 0;
        velocityRequest.Slot = 0;
        velocityRequest.OverrideBrakeDurNeutral = false;
        velocityRequest.LimitForwardMotion = false;
        velocityRequest.LimitReverseMotion = false;
        // velocityRequest.

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
        motorConfigs.MotorOutput.DutyCycleNeutralDeadband = AlgaeConstants.kRollerNeutralDeadband;
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

        AlgaeConstants.kPRollerMotor.loadPreferences();
        AlgaeConstants.kIRollerMotor.loadPreferences();
        AlgaeConstants.kDRollerMotor.loadPreferences();
        AlgaeConstants.kVRollerMotor.loadPreferences();
        motorConfigs.Slot0.kP = AlgaeConstants.kPRollerMotor.get();
        motorConfigs.Slot0.kI = AlgaeConstants.kIRollerMotor.get();
        motorConfigs.Slot0.kD = AlgaeConstants.kDRollerMotor.get();
        motorConfigs.Slot0.kV = AlgaeConstants.kVRollerMotor.get();
 
        StatusCode response = rollerConfigurator.apply(motorConfigs);
        if (!response.isOK())
            DriverStation.reportError("Could not apply PID configs, error code:" + response.toString(), new Error().getStackTrace());
    }
 
    @Override
    public void periodic() {
        if (!enabled) {
            rollerMotor.setControl(brakeRequest);
            // rollerMotor.set(velocityRequest.Velocity);
        }
 
        else {
            // voltageRequest.Output = velocityRequest.Velocity * 12 / 100;
            // rollerMotor.setControl(voltageRequest);

            // ************************************* FIX THIS ******************************
            //moves w/o set control SLKJDFLKJSDLFKJSLDKFJ !!!!!!!
            rollerMotor.setControl(velocityRequest);
            // want feedforward to change based on desired velocity which is what setcontrol was supposed to do
            // but it doesnt work, so figure out another way to change feedforward to figure out why
            // set control isnt working
            velocityRequest.FeedForward = 0.5;

            // velocityRequest.FeedForward = rollerMotor.setControl(velocityRequest).value;
            // rollerMotor.set(velocityRequest.Velocity);
        } 
    }
 
    //****************************** VELOCITY METHODS ******************************//
//  
    private void stop() {
        velocityRequest.Velocity = 0;
        rollerMotor.setControl(brakeRequest);
    }
    private void setVelocity(double velocity) {
        velocityRequest.Velocity = velocity;
        rollerMotor.setControl(velocityRequest);
    }

    public double getTargetVelocity() {
        return velocityRequest.Velocity;
    }

    private void setEnabled(boolean e) {
        this.enabled = e;
    }

    //****************************** COMMAND METHODS ******************************//

    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }
 
    public Command setVelocityCommand(double velocity) {
        velocityRequest.Velocity = velocity;
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    //****************************** NAMED COMMANDS ******************************//

    public Command intake() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommand(AlgaeConstants.kIntakePower.get())
        );
    }
    
    public Command shootBarge() { // TODO when design finished
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommand(AlgaeConstants.kBargeOuttake.get())
        );
    }
    
    public Command shootProcessor() {
        return Commands.sequence(
            setEnabledCommand(true),
            setVelocityCommand(AlgaeConstants.kProcessorOuttake.get())
        );
    }
    
    public Command stopCommand() {
        return Commands.sequence(
            setEnabledCommand(false),
            Commands.runOnce(() -> stop())
        );
    }
 
    //****************************** LOGGING METHODS ******************************//
 
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
                // tab.addNumber("Supply Current", () -> this.rollerMotor.getSupplyCurrent().getValueAsDouble());
                // tab.addNumber("Stator Current", () -> this.rollerMotor.getStatorCurrent().getValueAsDouble());
                // tab.addNumber("Applied Voltage", () -> this.rollerMotor.getMotorVoltage().getValueAsDouble());    
                break;
            default:
                break;
        }
        
    }
}
 
 
