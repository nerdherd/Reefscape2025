package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.Constants.WristConstants;
import frc.robot.util.NerdyMath;;

public class IntakeWrist extends SubsystemBase implements Reportable{
    private final TalonFX motor;
    private TalonFXConfigurator motorConfigurator;
    private Pigeon2 pigeon;

    private final MotionMagicVoltage motionMagicRequest; // Was 0 during initialization
    private final NeutralOut brakeRequest = new NeutralOut();

    private double desiredPosition; // Should be ~90 or wherever initial position is
    private double desiredAngle;
    public static boolean enabled = false;
    private double ff = 0;

    public IntakeWrist() {
        motor = new TalonFX(WristConstants.kMotorID);
        motionMagicRequest = new MotionMagicVoltage(0);     
        
        resetWrist();
    }

    //****************************** SETUP METHODS ******************************//


    public void resetWrist() 
    {
        enabled = false;
        motorConfigurator = motor.getConfigurator();       
        // TODO: Took out immediate stow to work on Wrist tuning
        
        // pigeon = new Pigeon2(WristConstants.kPigeonID, "rio");
        desiredPosition = 0;
        desiredAngle= 0;
        // desiredPosition = pigeon.getRoll().getValueAsDouble();
        
        motor.setControl(brakeRequest);

        // configure motor
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        configurePID(motorConfigs);
        
        motor.setNeutralMode(NeutralModeValue.Brake);
        zeroEncoder();
    }

    private void configurePID(TalonFXConfiguration motorConfigs) {
            motorConfigurator.refresh(motorConfigs);

            // motorConfigs.Feedback.FeedbackRemoteSensorID = V1IntakeConstants.kPigeonID;
            motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            motorConfigs.Feedback.SensorToMechanismRatio = 13.89; 
            // motorConfigs.Feedback.RotorToSensorRatio;
            motorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
            motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
            motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 45;
            motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;
            motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
            motorConfigs.Slot0.kP = WristConstants.kPMotor;
            motorConfigs.Slot0.kI = WristConstants.kItMotor;
            motorConfigs.Slot0.kD = WristConstants.kDMotor;
            motorConfigs.Slot0.kV = WristConstants.kVMotor;
            motorConfigs.Slot0.kS = WristConstants.kSMotor;
            motorConfigs.Slot0.kG = WristConstants.kGMotor;

            motorConfigs.MotionMagic.MotionMagicCruiseVelocity =  WristConstants.kCruiseVelocity;
            motorConfigs.MotionMagic.MotionMagicAcceleration = WristConstants.kAcceleration;
            motorConfigs.MotionMagic.MotionMagicJerk = WristConstants.kJerk;
        
            StatusCode response = motorConfigurator.apply(motorConfigs);
            if (!response.isOK()){
                DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
            }
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Wrist Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Current Rotations", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Current Velocity", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Current Acceleration", motor.getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber("Commanded Rotations", desiredPosition);
        SmartDashboard.putNumber("Commanded Degrees", desiredAngle);

        // TODO: Uncomment when ready to do position control

        // desiredPosition + pivot * constantToChangeUnit
        // ff = (-3.2787 * desiredPosition) - 1.5475; Harder method
        ff = WristConstants.kGMotor * Math.cos(((getPosition() + 0.5437) % 1) * 2 * Math.PI); // 0.5437 is wrist horizontal 
        if (enabled) {
            motor.setControl(motionMagicRequest.withFeedForward(ff));
        }
        else {
            motor.setControl(brakeRequest);
        }
    }

    // ****************************** STATE METHODS ****************************** //

    private void setEnabled(boolean e) {
        this.enabled = e;
    }
    
    public void setPosition(double position) {
        desiredPosition = position;
        motionMagicRequest.Position = desiredPosition;
    }

    public void setPositionDegrees(double positionDegrees) {
        desiredAngle = positionDegrees;
        desiredPosition = (desiredAngle * 0.015432) - 0.00011; // Depricated values
        
        // double newPos = NerdyMath.clamp(
        //     positionDegrees, 

        // );

        motionMagicRequest.Position = (desiredPosition);  // = (newPos / 360.0)
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    private void zeroEncoder() {
        motor.setPosition(0);
    }

    // Check if Motion Magic has reached the target
    public boolean hasReachedPosition(double position) {
        return true; //todo
        // return NerdyMath.inRange(
        //     getPosition(),
        //         positionDegrees - ElevatorConstants.kElevatorPivotDeadBand,
        //         positionDegrees + ElevatorConstants.kElevatorPivotDeadBand
        //     );
    }

    // ****************************** COMMAND METHODS ****************************** //

    public Command setDisabledCommand() {
        return Commands.runOnce(() -> this.setEnabled(false));
    }
    public Command setEnabledCommand() {
        return Commands.runOnce(() -> this.setEnabled(true));
    }

    private Command setPositionCommand(double position) {
        return Commands.sequence(
            Commands.runOnce(() -> setPosition(position))
        );
    }

    private Command stopCommand() {
        return Commands.sequence(
            setDisabledCommand(),
            Commands.runOnce(() -> motor.setControl(brakeRequest))
        );
    }

    // ****************************** NAMED COMMANDS ****************************** //

    // public StatusCode setControl(VoltageOut request)
    // {
    //     return motor.setControlPrivate(request);
    // }

    public TalonFX getMotor()
    {
        return motor;
    }

    public Command moveToStow() {
        return setPositionCommand(WristConstants.kStowPosition);
    }

    public Command moveToStation() {
        return setPositionCommand(WristConstants.kStationPosition);
    }

    public Command moveToReefL14() {
        return setPositionCommand(WristConstants.kL14Position);
    }

    public Command moveToReefL23() {
        return setPositionCommand(WristConstants.kL23Position);
    }

    public Command stop() {
        return stopCommand();
    }

    // ****************************** LOGGING METHODS ****************************** //

    @Override
    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
                SmartDashboard.putNumber("Coral Wrist Position", motor.getPosition().getValueAsDouble());
            case MEDIUM:
                SmartDashboard.putNumber("Coral Wrist Current", motor.getStatorCurrent().getValueAsDouble());
            case MINIMAL:
                break;
        }
    }
    
    @Override
    public void initShuffleboard(LOG_LEVEL level) { 
        if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Coral Wrist");
        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addString("Control Mode", motor.getControlMode()::toString);
            case MEDIUM:
                tab.addDouble("MM Position", () -> motionMagicRequest.Position);
                tab.addDouble("Desired Position", () -> desiredPosition);
            case MINIMAL:
                tab.addBoolean("Enabled", () -> enabled);
                tab.addNumber("Current Coral Wrist Angle", () -> motor.getPosition().getValueAsDouble());
                // tab.addNumber("Coral Wrist pigeon angle", () -> pigeon.getRoll().getValueAsDouble());
                tab.addNumber("Wrist Voltage", () -> motor.getMotorVoltage().getValueAsDouble());
                tab.addNumber("Wrist FF", () -> motionMagicRequest.FeedForward);
                break;
        }
    }

    public void setTargetAngleRaw(double wristAngleRaw) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetAngleRaw'");
    }

}