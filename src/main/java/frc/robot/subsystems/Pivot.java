package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.util.NerdyMath;

public class Pivot extends SubsystemBase implements Reportable{
    private TalonFX pivotMotor;
    private TalonFX pivotMotorRight;
    private final CANdi candi;

    private TalonFXConfigurator pivotConfigurator;
    private TalonFXConfigurator pivotConfiguratorRight; 
    private Pigeon2 pigeon;
    private double elevatorPosition;

    public boolean enabled = false; 
    private double desiredPosition; 
    private final MotionMagicVoltage motionMagicRequest;  
    private final NeutralOut brakeRequest = new NeutralOut();
    private NeutralModeValue neutralMode = NeutralModeValue.Brake;

    private final Follower followRequest = new Follower(PivotConstants.kLeftPivotMotorID, true);
    // public final VoltageOut voltageRequest = new VoltageOut(0);

    private double ff;
    private double ffInverse;

    private double commandedVoltage = 0.0;

    public Pivot () {
        desiredPosition = 0.0;
        motionMagicRequest = new MotionMagicVoltage(desiredPosition);
        candi = new CANdi(PivotConstants.kPivotCandiID);

        pivotMotor = new TalonFX(PivotConstants.kLeftPivotMotorID);
        pivotConfigurator = pivotMotor.getConfigurator();

        pivotMotorRight = new TalonFX(PivotConstants.kRightPivotMotorID);
        // pigeon = new Pigeon2(V1ElevatorConstants.kPivotPigeonID); // Not using Pigeon as of 2/23

        pivotConfiguratorRight = pivotMotorRight.getConfigurator();

        configureMotorV1();
        configurePIDV1();
        
        zeroEncoder();
        CommandScheduler.getInstance().registerSubsystem(this);
    }
    
    // ******************************** SETUP METHODS *************************************** //
    private void configurePIDV1() {
        TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
        
        pivotConfigurator.refresh(pivotConfiguration);

        pivotConfiguration.Slot0.kP = PivotConstants.kPElevatorPivot; 
        pivotConfiguration.Slot0.kI = PivotConstants.kIPivot;
        pivotConfiguration.Slot0.kD = PivotConstants.kDPivot;
        pivotConfiguration.Slot0.kV = PivotConstants.kVPivot;
        pivotConfiguration.Slot0.kS = PivotConstants.kSPivot;
        pivotConfiguration.Slot0.kA = PivotConstants.kAPivot;
        pivotConfiguration.Slot0.kG = PivotConstants.kGPivot;
        
        pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.kPivotCruiseVelocity;
        pivotConfiguration.MotionMagic.MotionMagicAcceleration = PivotConstants.kPivotCruiseAcceleration;
        pivotConfiguration.MotionMagic.MotionMagicJerk = PivotConstants.kPivotJerk;
        pivotConfiguration.MotionMagic.MotionMagicExpo_kV = 0;
        pivotConfiguration.MotionMagic.MotionMagicExpo_kA = 0;
        
        TalonFXConfiguration pivotConfigurationRight = new TalonFXConfiguration();

        pivotConfiguratorRight.refresh(pivotConfigurationRight);

        pivotConfigurationRight.Slot0.kP = PivotConstants.kPElevatorPivot; 
        pivotConfigurationRight.Slot0.kI = PivotConstants.kIPivot;
        pivotConfigurationRight.Slot0.kD = PivotConstants.kDPivot;
        pivotConfigurationRight.Slot0.kV = PivotConstants.kVPivot;
        pivotConfigurationRight.Slot0.kS = PivotConstants.kSPivot;
        pivotConfigurationRight.Slot0.kA = PivotConstants.kAPivot;
        pivotConfigurationRight.Slot0.kG = PivotConstants.kGPivot;
        
        pivotConfigurationRight.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.kPivotCruiseVelocity;
        pivotConfigurationRight.MotionMagic.MotionMagicAcceleration = PivotConstants.kPivotCruiseAcceleration;
        // pivotConfigurationRight.MotionMagic.MotionMagicJerk = V1ElevatorConstants.kElevatorPivotJerk; // TODO
        pivotConfigurationRight.MotionMagic.MotionMagicExpo_kV = 0;
        pivotConfigurationRight.MotionMagic.MotionMagicExpo_kA = 0;
        
        StatusCode statusCode = pivotConfigurator.apply(pivotConfiguration);
        if(!statusCode.isOK()){
            DriverStation.reportError(" Could not apply elevator pivot configs, error code =(", true);
        }

        StatusCode statusCodeRight = pivotConfiguratorRight.apply(pivotConfigurationRight);
        if(!statusCodeRight.isOK()){
            DriverStation.reportError(" Could not apply elevator RIGHT pivot configs, error code =(", true);
        }
    }

    public void configureMotorV1() {
        TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
        
        pivotConfigurator.refresh(pivotConfiguration);
        // pivotConfiguration.Feedback.FeedbackRemoteSensorID = PivotConstants.kPivotPigeonID;
        pivotConfiguration.Feedback.FeedbackRemoteSensorID = PivotConstants.kPivotCandiID;
        pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM1; 
        // pivotConfiguration.Feedback.RotorToSensorRatio = 360;
        // pivotConfiguration.Feedback.SensorToMechanismRatio = -1.068376; 
        pivotConfiguration.Feedback.RotorToSensorRatio = 1.0;
        pivotConfiguration.Feedback.SensorToMechanismRatio = PivotConstants.kPivotGearRatio; 
        pivotConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        pivotConfiguration.Voltage.PeakForwardVoltage = 11.5;
        pivotConfiguration.Voltage.PeakReverseVoltage = -11.5;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfiguration.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfiguration.Audio.AllowMusicDurDisable = true;
        pivotConfiguration.MotorOutput.NeutralMode = neutralMode;

        StatusCode statusCode = pivotConfigurator.apply(pivotConfiguration);
        if (!statusCode.isOK()){
            DriverStation.reportError("Could not apply Elevator configs, fix code??? =(", true);
        }

        TalonFXConfiguration pivotConfigurationRight = new TalonFXConfiguration();

        pivotConfiguratorRight.refresh(pivotConfigurationRight);
        // pivotConfigurationRight.Feedback.FeedbackRemoteSensorID = V1ElevatorConstants.kPivotPigeonID;
        pivotConfigurationRight.Feedback.FeedbackRemoteSensorID = PivotConstants.kPivotCandiID;
        pivotConfigurationRight.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM1; //TODO change orientation later
        pivotConfigurationRight.Feedback.RotorToSensorRatio = 1.0;
        // pivotConfigurationRight.Feedback.RotorToSensorRatio = V1ElevatorConstants.kElevatorPivotGearRatio;
        pivotConfigurationRight.Feedback.SensorToMechanismRatio = PivotConstants.kPivotGearRatio; 
        pivotConfigurationRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        pivotConfigurationRight.Voltage.PeakForwardVoltage = 11.5;
        pivotConfigurationRight.Voltage.PeakReverseVoltage = -11.5;
        pivotConfigurationRight.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfigurationRight.CurrentLimits.SupplyCurrentLimitEnable = false;
        pivotConfigurationRight.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfigurationRight.CurrentLimits.StatorCurrentLimitEnable = false;
        pivotConfigurationRight.Audio.AllowMusicDurDisable = true;
        pivotConfigurationRight.MotorOutput.NeutralMode = neutralMode;

        StatusCode RightstatusCode = pivotConfiguratorRight.apply(pivotConfigurationRight);
        if (!RightstatusCode.isOK()){
            DriverStation.reportError("Could not apply Elevator configs, fix code??? =(", true);
        }
        CANdiConfiguration candiConfiguration = new CANdiConfiguration();
        candiConfiguration.PWM1.AbsoluteSensorOffset = PivotConstants.kPivotOffset;
        candiConfiguration.PWM1.SensorDirection = false;
        candiConfiguration.PWM1.AbsoluteSensorDiscontinuityPoint = 0.5;
        candi.getConfigurator().apply(candiConfiguration);
    }

    @Override
    public void periodic() {
        if (!enabled) {
            return;
        }

        //ff = (ElevatorConstants.kElevatorPivotStowedFF + ElevatorConstants.kElevatorPivotDiffFF * (elevatorPosition / ElevatorConstants.kElevatorPivotExtendedFFPosition)) * Math.cos(2 * Math.PI * getPosition());
        
        ff = PivotConstants.kFPivot * Math.cos(2 * Math.PI * getPosition());
        pivotMotor.setControl(motionMagicRequest.withFeedForward(ff)); 
        // pivotMotorRight.setControl(followRequest); 
    }

    // ****************************** STATE METHODS ***************************** //
    public void zeroEncoder() {
        desiredPosition = PositionEquivalents.Stow.coralPos.pivotPosition;//PivotConstants.kPivotOffset; 
        pivotMotor.setPosition(desiredPosition); // Start position is based off of difference between flat starting pose and hard-stopped starting pose
        pivotMotorRight.setPosition(desiredPosition);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if(enabled) {
            pivotMotorRight.setControl(followRequest);
        } else {
            stopMotion();
        }
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        this.neutralMode = neutralMode;
    }

    public void stopMotion() {
        pivotMotor.setControl(brakeRequest);
        pivotMotorRight.setControl(brakeRequest);
    }

    public void setTargetPosition(double position) {
        desiredPosition = Math.min(position, PivotConstants.kPivotMax);
        motionMagicRequest.Position = desiredPosition; 
        // motionMagicRequest.Position = desiredPosition - PivotConstants.kPigeonOffset; // Only for use with pigeon
    }

    public void setPivotVoltage(double voltage) {
        //TODO NerdyMath.clamp(
        commandedVoltage = voltage;
        pivotMotor.setVoltage(commandedVoltage);
    }

    public boolean atPosition() {
        return NerdyMath.inRange(pivotMotor.getPosition().getValueAsDouble(), 
        desiredPosition - PivotConstants.atPositionDeadband,
        desiredPosition + PivotConstants.atPositionDeadband);
    }
    
    public boolean atPositionWide() {
        return NerdyMath.inRange(pivotMotor.getPosition().getValueAsDouble(), 
        desiredPosition - PivotConstants.atPositionWideDeadband,
        desiredPosition + PivotConstants.atPositionWideDeadband);
    }

    // private void setPositionDegrees(double positionDegrees) {
    //     double newPos = NerdyMath.clamp(
    //         positionDegrees, 
    //         ElevatorConstants.kElevatorPivotMin, 
    //         ElevatorConstants.kElevatorPivotMax
    //     );

    //     motionMagicRequest.Position = (newPos / 360.0);  
    // }

    // private void incrementPosition(double incrementDegrees) {
    //     if(Math.abs(incrementDegrees) <= 0.001) {
    //         return;
    //     }
    //     setPositionDegrees(getTargetPositionDegrees() + incrementDegrees);
    // }


    // private double getTargetPositionRev() {
    //     return motionMagicRequest.Position;
    // }

    // private double getTargetPositionDegrees() {
    //     return getTargetPositionRev() * 360;
    // }

    public double getPosition() {
        ///////////////////
        /// TODO: we do need the offset for pivot. because current "reset 0" is not the real horizontal zero
        /// ????
        // return (pivotMotor.getPosition().getValueAsDouble() + PivotConstants.kPigeonOffset); 
        return (pivotMotor.getPosition().getValueAsDouble()); 
    }

    

    public double getPositionDegrees() {
        return getPosition() * 360;
    }

    public void setElevatorLength(double len) {
        // TODO
    }

    // ****************************** COMMAND METHODS ***************************** //

    // public Object resetEncoders() { // Reset both pivot encoders to 0. Can delete after use
    //     pivotMotor.setSelectedSensorPosition(0);

    // }

    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    public Command stopCommand() {
        return setEnabledCommand(false);
    }

    // ****************************** LOGGING METHODS ****************************** //
    
    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        if (priority == LOG_LEVEL.OFF) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator Pivot");
        switch (priority) {
            case OFF:
                break;
            case ALL:
                tab.addString("Pivot Control Mode", pivotMotor.getControlMode()::toString);
                tab.addNumber("Pivot MM Position", () -> motionMagicRequest.Position);
                tab.addNumber("Pivot Supply Current", () -> pivotMotor.getSupplyCurrent().getValueAsDouble());
                tab.addBoolean("Pivot At Position", () -> atPosition());
                tab.addBoolean("Pivot At Position Wide", () -> atPositionWide());
            case MEDIUM:
                tab.addBoolean("Pivot Enabled", () -> enabled);
            case MINIMAL:
                tab.addNumber("Pivot FF", () -> motionMagicRequest.FeedForward);
                tab.addNumber("Pivot Current Position", () -> getPosition());
                tab.addNumber("Pivot Desired Position", ()-> desiredPosition);
                tab.addNumber("Pivot Voltage", () -> pivotMotor.getMotorVoltage().getValueAsDouble());    
                tab.addNumber("Pivot Temperature 1", () -> pivotMotor.getDeviceTemp().getValueAsDouble());
                tab.addNumber("Pivot Temperature 2", () -> pivotMotorRight.getDeviceTemp().getValueAsDouble());
                
                break;
            }
    }
    
}
