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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
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

    private final Follower followRequest = new Follower(PivotConstants.kLeftPivotMotorID, true);
    // public final VoltageOut voltageRequest = new VoltageOut(0);

    private double ff;
    private double ffInverse;

    private double commandedVoltage = 0.0;

    public ElevatorPivot () {
        desiredPosition = 0.0;
        motionMagicRequest = new MotionMagicVoltage(desiredPosition);

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
        pivotConfiguration.Slot0.kI = PivotConstants.kIElevatorPivot;
        pivotConfiguration.Slot0.kD = PivotConstants.kDElevatorPivot;
        pivotConfiguration.Slot0.kV = PivotConstants.kVElevatorPivot;
        pivotConfiguration.Slot0.kS = PivotConstants.kSElevatorPivot;
        pivotConfiguration.Slot0.kA = PivotConstants.kAElevatorPivot;
        pivotConfiguration.Slot0.kG = PivotConstants.kGElevatorPivot;
        
        pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.kEPivotCruiseVelocity;
        pivotConfiguration.MotionMagic.MotionMagicAcceleration = PivotConstants.kElevatorPivotCruiseAcceleration;
        pivotConfiguration.MotionMagic.MotionMagicJerk = PivotConstants.kElevatorPivotJerk;
        pivotConfiguration.MotionMagic.MotionMagicExpo_kV = 0;
        pivotConfiguration.MotionMagic.MotionMagicExpo_kA = 0;
        
        TalonFXConfiguration pivotConfigurationRight = new TalonFXConfiguration();

        pivotConfiguratorRight.refresh(pivotConfigurationRight);

        pivotConfigurationRight.Slot0.kP = PivotConstants.kPElevatorPivot; 
        pivotConfigurationRight.Slot0.kI = PivotConstants.kIElevatorPivot;
        pivotConfigurationRight.Slot0.kD = PivotConstants.kDElevatorPivot;
        pivotConfigurationRight.Slot0.kV = PivotConstants.kVElevatorPivot;
        pivotConfigurationRight.Slot0.kS = PivotConstants.kSElevatorPivot;
        pivotConfigurationRight.Slot0.kA = PivotConstants.kAElevatorPivot;
        pivotConfigurationRight.Slot0.kG = PivotConstants.kGElevatorPivot;
        
        pivotConfigurationRight.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.kEPivotCruiseVelocity;
        pivotConfigurationRight.MotionMagic.MotionMagicAcceleration = PivotConstants.kElevatorPivotCruiseAcceleration;
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
        pivotConfiguration.Feedback.SensorToMechanismRatio = PivotConstants.kElevatorPivotGearRatio; 
        pivotConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        pivotConfiguration.Voltage.PeakForwardVoltage = 11.5;
        pivotConfiguration.Voltage.PeakReverseVoltage = -11.5;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfiguration.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
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
        pivotConfigurationRight.Feedback.SensorToMechanismRatio = PivotConstants.kElevatorPivotGearRatio; 
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
        // why do we change it? -Duan
        //ff = (ElevatorConstants.kElevatorPivotStowedFF + ElevatorConstants.kElevatorPivotDiffFF * (elevatorPosition / ElevatorConstants.kElevatorPivotExtendedFFPosition)) * Math.cos(2 * Math.PI * getPosition());
        ff =  PivotConstants.kFElevatorPivot * Math.cos(2 * Math.PI * getPosition());

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
    
    public boolean atPositionWide() {
        return NerdyMath.inRange(pivotMotor.getPosition().getValueAsDouble(), 
        desiredPosition - 0.04,
        desiredPosition + 0.04);
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
        tab.addNumber("Supply Current", () -> pivotMotor.getSupplyCurrent().getValueAsDouble());
        tab.addNumber("Applied Voltage", () -> pivotMotor.getMotorVoltage().getValueAsDouble());    
    }
    
}
