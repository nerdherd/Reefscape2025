package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.V1ElevatorConstants;
import frc.robot.util.NerdyMath;

public class ElevatorPivot extends SubsystemBase implements Reportable{
    private TalonFX pivotMotor;
    private TalonFX pivotMotorRight;

    private TalonFXConfigurator pivotConfigurator;
    private TalonFXConfigurator pivotConfiguratorRight; 
    private Pigeon2 pigeon;
    private double elevatorPosition;

    public boolean enabled = false; // Change back to true
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(ElevatorConstants.kElevatorPivotStowPosition);
    private final NeutralOut brakeRequest = new NeutralOut();
    private final Follower followRequest = new Follower(ElevatorConstants.kLeftPivotMotorID, true);

    public ElevatorPivot (boolean V1) {
        pivotMotor = new TalonFX(ElevatorConstants.kLeftPivotMotorID);
        pivotConfigurator = pivotMotor.getConfigurator();
        if(V1) {
            pivotMotorRight = new TalonFX(V1ElevatorConstants.kRightPivotMotorID);
            pigeon = new Pigeon2(V1ElevatorConstants.kPivotPigeonID);
            pivotMotorRight.setControl(followRequest);
            pivotConfiguratorRight = pivotMotorRight.getConfigurator();
        }
        configureMotorV1();
        configurePIDV1();
        pivotMotor.setPosition(ElevatorConstants.kElevatorPivotStowPosition);
    }
    
    // ******************************** SETUP METHODS *************************************** //
    private void configurePIDV1() {
        TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
        
        pivotConfigurator.refresh(pivotConfiguration);

        pivotConfiguration.Slot0.kP = V1ElevatorConstants.kPElevatorPivot;
        pivotConfiguration.Slot0.kI = V1ElevatorConstants.kIElevatorPivot;
        pivotConfiguration.Slot0.kD = V1ElevatorConstants.kDElevatorPivot;
        pivotConfiguration.Slot0.kV = V1ElevatorConstants.kVElevatorPivot;
        pivotConfiguration.Slot0.kS = V1ElevatorConstants.kSElevatorPivot;
        pivotConfiguration.Slot0.kA = V1ElevatorConstants.kAElevatorPivot;
        pivotConfiguration.Slot0.kG = V1ElevatorConstants.kGElevatorPivot;
        
        pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity = V1ElevatorConstants.kEPivotCruiseVelocity;
        pivotConfiguration.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kElevatorPivotCruiseAcceleration;
        pivotConfiguration.MotionMagic.MotionMagicJerk = ElevatorConstants.kElevatorPivotJerk;
        pivotConfiguration.MotionMagic.MotionMagicExpo_kV = 0;
        pivotConfiguration.MotionMagic.MotionMagicExpo_kA = 0;
        
        TalonFXConfiguration pivotConfigurationRight = new TalonFXConfiguration();

        pivotConfiguratorRight.refresh(pivotConfigurationRight);

        pivotConfigurationRight.Slot0.kP = V1ElevatorConstants.kPElevatorPivot;
        pivotConfigurationRight.Slot0.kI = V1ElevatorConstants.kIElevatorPivot;
        pivotConfigurationRight.Slot0.kD = V1ElevatorConstants.kDElevatorPivot;
        pivotConfigurationRight.Slot0.kV = V1ElevatorConstants.kVElevatorPivot;
        pivotConfigurationRight.Slot0.kS = V1ElevatorConstants.kSElevatorPivot;
        pivotConfigurationRight.Slot0.kA = V1ElevatorConstants.kAElevatorPivot;
        pivotConfigurationRight.Slot0.kG = V1ElevatorConstants.kGElevatorPivot;
        
        pivotConfigurationRight.MotionMagic.MotionMagicCruiseVelocity = V1ElevatorConstants.kEPivotCruiseVelocity;
        pivotConfigurationRight.MotionMagic.MotionMagicAcceleration = V1ElevatorConstants.kElevatorPivotCruiseAcceleration;
        pivotConfigurationRight.MotionMagic.MotionMagicJerk = V1ElevatorConstants.kElevatorPivotJerk;
        pivotConfigurationRight.MotionMagic.MotionMagicExpo_kV = 0;
        pivotConfigurationRight.MotionMagic.MotionMagicExpo_kA = 0;
        
        StatusCode statusCode = pivotConfigurator.apply(pivotConfiguration);
        if(!statusCode.isOK()){
            DriverStation.reportError(" Could not apply elevator pivot configs, error code =(", true);
        }

        StatusCode statusCodeRight = pivotConfiguratorRight.apply(pivotConfigurationRight);
        if(!statusCodeRight.isOK()){
            DriverStation.reportError(" Could not apply elevator pivot configs, error code =(", true);
        }
    }

    private void configureMotorV1() {
        TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
        
        pivotConfigurator.refresh(pivotConfiguration);
        // pivotConfiguration.Feedback.FeedbackRemoteSensorID = FeedbackSensorSourceValue.RotorSensor;
        pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; //TODO change orientation later
        pivotConfiguration.Feedback.RotorToSensorRatio = V1ElevatorConstants.kElevatorPivotGearRatio;
        pivotConfiguration.Feedback.SensorToMechanismRatio = 1.0; //TODO change later
        pivotConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO change later
        pivotConfiguration.Voltage.PeakForwardVoltage = 11.5;
        pivotConfiguration.Voltage.PeakReverseVoltage = -11.5;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = false;
        pivotConfiguration.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = false;
        pivotConfiguration.Audio.AllowMusicDurDisable = true;

        StatusCode statusCode = pivotConfigurator.apply(pivotConfiguration);
        if (!statusCode.isOK()){
            DriverStation.reportError("Could not apply Elevator configs, fix code??? =(", true);
        }

        TalonFXConfiguration pivotConfigurationRight = new TalonFXConfiguration();

        pivotConfiguratorRight.refresh(pivotConfigurationRight);
        // pivotConfigurationRight.Feedback.FeedbackRemoteSensorID = V1ElevatorConstants.kPivotPigeonID;
        pivotConfigurationRight.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; //TODO change orientation later
        pivotConfigurationRight.Feedback.RotorToSensorRatio = V1ElevatorConstants.kElevatorPivotGearRatio;
        pivotConfigurationRight.Feedback.SensorToMechanismRatio = 1.0; //TODO change later
        pivotConfigurationRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO change later
        pivotConfigurationRight.Voltage.PeakForwardVoltage = 11.5;
        pivotConfigurationRight.Voltage.PeakReverseVoltage = -11.5;
        pivotConfigurationRight.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfigurationRight.CurrentLimits.SupplyCurrentLimitEnable = false;
        pivotConfigurationRight.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfigurationRight.CurrentLimits.StatorCurrentLimitEnable = false;
        pivotConfigurationRight.Audio.AllowMusicDurDisable = true;

        StatusCode RightstatusCode = pivotConfiguratorRight.apply(pivotConfigurationRight);
        if (!RightstatusCode.isOK()){
            DriverStation.reportError("Could not apply Elevator configs, fix code??? =(", true);
        }
    }

    @Override
    public void periodic() {
        if (enabled){
            // pivotMotor.setControl(motionMagicRequest);
            // pivotMotor.setControl(motionMagicRequest);
            // pivotMotorRight.setControl(followRequest);
            // DriverStation.reportWarning("SDKLJLDSHFKJSFGKJFS: " + Double.toString(motionMagicRequest.Position), false);
        } else {
            // pivotMotor.setControl(brakeRequest);
        }

    }

    // ****************************** STATE METHODS ***************************** //

    private void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    private void setPositionDegrees(double positionDegrees) {
        double newPos = NerdyMath.clamp(
            positionDegrees, 
            ElevatorConstants.kElevatorPivotMin, 
            ElevatorConstants.kElevatorPivotMax
        );

        motionMagicRequest.Position = (newPos / 360.0);  
    }

    private void incrementPosition(double incrementDegrees) {
        if(Math.abs(incrementDegrees) <= 0.001) {
            return;
        }
        setPositionDegrees(getTargetPositionDegrees() + incrementDegrees);
    }

    public void setElevatorPosition(double elevatorPosition) {
        this.elevatorPosition = elevatorPosition;
    }

    // ****************************** GET METHODS ***************************** //

    private double getTargetPositionRev() {
        return motionMagicRequest.Position;
    }

    private double getTargetPositionDegrees() {
        return getTargetPositionRev() * 360;
    }

    private double getPositionRev() {
        return pivotMotor.getPosition().getValueAsDouble();
    }
    
    public double getPositionDegrees() {
        return getPositionRev() * 360;
    }

    public boolean hasReachedPosition(double positionDegrees) {
        return NerdyMath.inRange(
            getPositionDegrees(),
                positionDegrees - ElevatorConstants.kElevatorPivotDeadBand,
                positionDegrees + ElevatorConstants.kElevatorPivotDeadBand
            );
    }

    public boolean atTargetPosition() {
        return hasReachedPosition(getPositionDegrees());
    }

    // ****************************** COMMAND METHODS ***************************** //

    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    public Command stopCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> pivotMotor.setControl(brakeRequest)),
            setEnabledCommand(false)
        );
    }

    public Command setPositionCommand(double position) {
        return Commands.runOnce(() -> setPositionDegrees(position));
    }
    
    public Command incrementPositionCommand(double increment) {
        return Commands.runOnce(() -> incrementPosition(increment));
    }

    // ****************************** NAMED COMMANDS ****************************** //

    public Command moveToStow() {
        SmartDashboard.putBoolean("Pushsss", false);
        return Commands.runOnce(() -> setPositionDegrees(ElevatorConstants.kElevatorPivotStowPosition));
        //return Commands.runOnce(() -> setPositionRev(-0.5));
    }

    public Command moveToStart() {
        return Commands.runOnce(() -> setPositionDegrees(ElevatorConstants.kElevatorPivotStartPosition));
    }

    public Command moveToPickup() {
        SmartDashboard.putBoolean("Pushsss", true);
        return Commands.runOnce(() -> setPositionDegrees(ElevatorConstants.kElevatorPivotPickUpPosition));
        //return Commands.runOnce(() -> setPositionRev(0.5));
    }

    public Command stop() {
        return stopCommand();
    }

    // ****************************** LOGGING METHODS ****************************** //

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        throw new UnsupportedOperationException("Unimplemented method 'reportToSmartDashboard'");
    }
    
    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator Pivot");
        tab.addNumber("Position Rev", () -> getPositionRev());
        tab.addNumber("Position Degrees", () -> getPositionDegrees());
        tab.addNumber("Set Position Rev", () -> motionMagicRequest.Position);
        tab.addNumber("Velocity", () -> pivotMotor.getVelocity().getValueAsDouble());
        tab.addBoolean("Enabled", ()-> enabled);
    }
    
}
