package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    public boolean enabled = false; 
    private double desiredPosition; 
    private final MotionMagicVoltage motionMagicRequest;  
    private final NeutralOut brakeRequest = new NeutralOut();

    private final Follower followRequest = new Follower(V1ElevatorConstants.kLeftPivotMotorID, true);
    // public final VoltageOut voltageRequest = new VoltageOut(0);

    private double ff;
    private double ffInverse;

    private double commandedVoltage = 0.0;

    public ElevatorPivot () {
        desiredPosition = 0.0;
        motionMagicRequest = new MotionMagicVoltage(desiredPosition);

        pivotMotor = new TalonFX(V1ElevatorConstants.kLeftPivotMotorID);
        pivotConfigurator = pivotMotor.getConfigurator();

        pivotMotorRight = new TalonFX(V1ElevatorConstants.kRightPivotMotorID);
        // pigeon = new Pigeon2(V1ElevatorConstants.kPivotPigeonID); // Not using Pigeon as of 2/23

        pivotConfiguratorRight = pivotMotorRight.getConfigurator();

        configureMotorV1();
        configurePIDV1();
        
        zeroEncoder();
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

    private void configureMotorV1() {
        TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
        
        pivotConfigurator.refresh(pivotConfiguration);
        // pivotConfiguration.Feedback.FeedbackRemoteSensorID = FeedbackSensorSourceValue.RotorSensor;
        pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 
        // pivotConfiguration.Feedback.RotorToSensorRatio = ; // 0.1
        pivotConfiguration.Feedback.SensorToMechanismRatio = V1ElevatorConstants.kElevatorPivotGearRatio; 
        pivotConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        pivotConfiguration.Voltage.PeakForwardVoltage = 11.5;
        pivotConfiguration.Voltage.PeakReverseVoltage = -11.5;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = false;
        pivotConfiguration.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = false;
        pivotConfiguration.Audio.AllowMusicDurDisable = true;
        pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        StatusCode statusCode = pivotConfigurator.apply(pivotConfiguration);
        if (!statusCode.isOK()){
            DriverStation.reportError("Could not apply Elevator configs, fix code??? =(", true);
        }

        TalonFXConfiguration pivotConfigurationRight = new TalonFXConfiguration();

        pivotConfiguratorRight.refresh(pivotConfigurationRight);
        // pivotConfigurationRight.Feedback.FeedbackRemoteSensorID = V1ElevatorConstants.kPivotPigeonID;
        pivotConfigurationRight.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; //TODO change orientation later
        // pivotConfigurationRight.Feedback.RotorToSensorRatio = V1ElevatorConstants.kElevatorPivotGearRatio;
        pivotConfigurationRight.Feedback.SensorToMechanismRatio = V1ElevatorConstants.kElevatorPivotGearRatio; 
        pivotConfigurationRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        pivotConfigurationRight.Voltage.PeakForwardVoltage = 11.5;
        pivotConfigurationRight.Voltage.PeakReverseVoltage = -11.5;
        pivotConfigurationRight.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfigurationRight.CurrentLimits.SupplyCurrentLimitEnable = false;
        pivotConfigurationRight.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfigurationRight.CurrentLimits.StatorCurrentLimitEnable = false;
        pivotConfigurationRight.Audio.AllowMusicDurDisable = true;
        pivotConfigurationRight.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        StatusCode RightstatusCode = pivotConfiguratorRight.apply(pivotConfigurationRight);
        if (!RightstatusCode.isOK()){
            DriverStation.reportError("Could not apply Elevator configs, fix code??? =(", true);
        }
    }

    @Override
    public void periodic() {
        ff = (ElevatorConstants.kElevatorPivotStowedFF + ElevatorConstants.kElevatorPivotDiffFF * (elevatorPosition / ElevatorConstants.kElevatorPivotExtendedFFPosition)) * Math.cos(2 * Math.PI * getPosition());

        if (enabled) {
            pivotMotor.setControl(motionMagicRequest.withFeedForward(ff)); 

            pivotMotorRight.setControl(followRequest); 
        }
        else {
            pivotMotor.setControl(brakeRequest);
        }
    }

    // ****************************** STATE METHODS ***************************** //
    public void zeroEncoder() {
        pivotMotor.setPosition(0);
        pivotMotorRight.setPosition(0);
        desiredPosition = 0.0;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void stopMotion() {
        pivotMotor.setControl(brakeRequest);
        pivotMotorRight.setControl(brakeRequest);
    }

    public void setTargetPosition(double position) {
        //TODO NerdyMath.clamp(
        desiredPosition = position;
        motionMagicRequest.Position = desiredPosition;
    }

    public void setPivotVoltage(double voltage) {
        //TODO NerdyMath.clamp(
        commandedVoltage = voltage;
        pivotMotor.setVoltage(commandedVoltage);
    }

    public boolean atPosition() {
        return NerdyMath.inRange(pivotMotor.getPosition().getValueAsDouble(), 
        desiredPosition - 0.01,
        desiredPosition + 0.01);
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
        return pivotMotor.getPosition().getValueAsDouble(); 
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
        return Commands.sequence(
            Commands.runOnce(() -> pivotMotor.setControl(brakeRequest)),
            setEnabledCommand(false)
        );
    }

    public Command stop() {
        return stopCommand();
    }

    // ****************************** LOGGING METHODS ****************************** //

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        SmartDashboard.putNumber("Pivot Voltage (ID 17)", pivotMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Voltage (ID 18)", pivotMotorRight.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Pivot Current Rotations (ID 17)", pivotMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Pivot Current Rotations (ID 18)", pivotMotorRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Target Rotations", desiredPosition);
        // SmartDashboard.putNumber("Error Left (Rotations)", error);
        

        // SmartDashboard.putNumber("Adjusted Offset Pivot Current Rotations (ID 17)", getPositionRev());
        // SmartDashboard.putNumber("Adjusted Offset Pivot Commanded Rotations (ID 17)", desiredPosition - pivotPositionOffset);
        // SmartDashboard.putNumber("Commanded kP Pivot Voltage", commandedVoltage);

        // SmartDashboard.putNumber("Applied feedforward", ff);
        SmartDashboard.putNumber("Feedforward inverse", ffInverse);

        // SmartDashboard.putNumber("Error Right (Rotations)", errorRight);

    }
    
    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator Pivot");
        tab.addNumber("Position Rev", () -> getPosition());
        // tab.addNumber("Position Degrees", () -> getPositionDegrees());
        tab.addNumber("Set Position Rev", () -> motionMagicRequest.Position);
        tab.addNumber("Velocity", () -> pivotMotor.getVelocity().getValueAsDouble());
        tab.addBoolean("Enabled", ()-> enabled);
        tab.addNumber("Actual Voltage", ()-> pivotMotor.getMotorVoltage().getValueAsDouble());
        tab.addNumber("Desired Position", ()-> desiredPosition);
        tab.addNumber("Actual Position", ()-> pivotMotor.getPosition().getValueAsDouble());
        tab.addNumber("feedForward", ()-> ff);
        tab.addBoolean("At position", () -> atPosition());
    }
    
}
