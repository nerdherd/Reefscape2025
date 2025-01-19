package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.NerdyMath;

public class ElevatorPivot extends SubsystemBase implements Reportable{
    private TalonFX pivotMotor;
    private TalonFXConfigurator pivotConfigurator;
    private Pigeon2 pigeon;
    public boolean enabled = false;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(ElevatorConstants.kElevatorPivotStowPosition.get()/360);
    private final NeutralOut brakeRequest = new NeutralOut();
    

    public ElevatorPivot (){
        pivotMotor = new TalonFX(ElevatorConstants.kPivotMotorID);
        pigeon = new Pigeon2(ElevatorConstants.kPivotPigeonID);
        CommandScheduler.getInstance().registerSubsystem(this);
        configureMotor();
        configurePID();
    }
    //********************************SETUP METHODS ***************************************/
    private void configurePID(){
        ElevatorConstants.kElevatorPivotStowPosition.loadPreferences();
        ElevatorConstants.kElevatorPivotStartPosition.loadPreferences();
        ElevatorConstants.kElevatorPivotPickUpPosition.loadPreferences();
        ElevatorConstants.kPElevatorPivot.loadPreferences();
        ElevatorConstants.kIElevatorPivot.loadPreferences();
        ElevatorConstants.kDElevatorPivot.loadPreferences();
        ElevatorConstants.kVElevatorPivot.loadPreferences();
        TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
        pivotConfigurator.refresh(pivotConfiguration);
        pivotConfiguration.Slot0.kP = ElevatorConstants.kPElevatorPivot.get();
        pivotConfiguration.Slot0.kI = ElevatorConstants.kIElevatorPivot.get();
        pivotConfiguration.Slot0.kD = ElevatorConstants.kDElevatorPivot.get();
        pivotConfiguration.Slot0.kV = ElevatorConstants.kVElevatorPivot.get();
        pivotConfiguration.Slot0.kS = ElevatorConstants.kSElevatorPivot.get();
        pivotConfiguration.Slot0.kA = ElevatorConstants.kAElevatorPivot.get();
        pivotConfiguration.Slot0.kG = ElevatorConstants.kGElevatorPivot.get();
        ElevatorConstants.kEPivotCruiseVelocity.loadPreferences();
        ElevatorConstants.kElevatorPivotCruiseAcceleration.loadPreferences();
        StatusCode statusCode = pivotConfigurator.apply(pivotConfiguration);
        if(!statusCode.isOK()){
            DriverStation.reportError(" Could not apply elevator pivot configs, error code =(", true);
        }
    }

    private void configureMotor(){
        TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
        pivotConfigurator.refresh(pivotConfiguration);
        pivotConfiguration.Feedback.FeedbackRemoteSensorID = ElevatorConstants.kPivotPigeonID;
        pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemotePigeon2_Roll; //TODO change orientation later
        pivotConfiguration.Feedback.RotorToSensorRatio = -ElevatorConstants.kElevatorPivotGearRatio / 360;
        pivotConfiguration.Feedback.SensorToMechanismRatio = -1; //TODO change later
        pivotConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO change later
        pivotConfiguration.Voltage.PeakForwardVoltage = 11.5;
        pivotConfiguration.Voltage.PeakReverseVoltage = -11.5;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfiguration.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = false;
        pivotConfiguration.Audio.AllowMusicDurDisable = true;

        StatusCode statusCode = pivotConfigurator.apply(pivotConfiguration);
        if (!statusCode.isOK()){
            DriverStation.reportError("Could not apply Elevator configs, fix code??? =(", true);
        }
    }
    /*SETTER METHODS */
    public void setEnabled(boolean enabled){
        this.enabled = enabled;
    }
    public Command setEnabledCommand(boolean enabled){
        return Commands.runOnce(() -> setEnabled(enabled));
    }
    public void setPosition(double positionDegrees){
        double newPos = NerdyMath.clamp(
            mapDegrees(positionDegrees), 
            ElevatorConstants.kElevatorPivotMin, 
            ElevatorConstants.kElevatorPivotMax
            );
        motionMagicRequest.Position = newPos;  
    }
    public void stop() {
        setEnabled(false);
        pivotMotor.setControl(brakeRequest);
    }
    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }
    public Command setPositionCommand(boolean enabled){
        return Commands.runOnce(() -> setEnabled(enabled));
    }
    public double mapDegrees(double deg){
        deg = deg - (Math.floor(deg / 360.0 ) * 360.0 );
        if(deg > 180)deg -=360;
        return deg;
    }
    public void increamentPosition(){

    }
    public double getTargetPositionRev(){
        return motionMagicRequest.Position;
    }
    public double getTargetPositionDegrees(){
        return getTargetPositionRev() * 360;
    }
    public double getPositionRev(){
        return pivotMotor.getPosition().getValueAsDouble();
    }
    public double getPositionDegrees(){
        return getPositionRev() * 360;
    }
    public void incrementPosition(double incrementDegrees) {
        if(Math.abs(incrementDegrees) <= 0.001) {
            return;
        }
        setPosition(getTargetPositionDegrees() + incrementDegrees);
    }
    public Command incrementPositionCommand(double increment){
        return Commands.runOnce(() -> setPosition(increment));
    }
    public Command moveToStow(){
        return Commands.runOnce(() -> setPosition(ElevatorConstants.kElevatorPivotStowPosition.get()));
    }
    public Command moveToStart(){
        return Commands.runOnce(() -> setPosition(ElevatorConstants.kElevatorPivotStartPosition.get()));
    }
    public Command moveToPickUp(){
        return Commands.runOnce(() -> setPosition(ElevatorConstants.kElevatorPivotPickUpPosition.get()));
    }
    public boolean hasReachedPosition(double positionDegrees){
        return NerdyMath.inRange(
            getPositionDegrees(),
            positionDegrees - ElevatorConstants.kElevatorPivotDeadBand.get(),
            positionDegrees + ElevatorConstants.kElevatorPivotDeadBand.get()
        ) && NerdyMath.inRange(
            getTargetPositionDegrees(), 
            positionDegrees - ElevatorConstants.kElevatorPivotDeadBand.get(), 
            positionDegrees + ElevatorConstants.kElevatorPivotDeadBand.get());
    }
    public boolean atTargetPosition(){
        return hasReachedPosition(getPositionDegrees());
    }
    @Override
    public void periodic() {
        if (enabled){
           pivotMotor.setControl(motionMagicRequest);
        } else {
            pivotMotor.setControl(brakeRequest);
        }

    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reportToSmartDashboard'");
    }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initShuffleboard'");
    }
    
}
