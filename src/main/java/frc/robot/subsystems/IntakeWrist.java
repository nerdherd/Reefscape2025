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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.V1IntakeConstants;
import frc.robot.util.NerdyMath;;

public class IntakeWrist extends SubsystemBase implements Reportable{
    private final TalonFX motor;
    private final TalonFXConfigurator motorConfigurator;
    private Pigeon2 pigeon;




    private final MotionMagicVoltage motionMagicRequest; // Was 0 during initialization
    private final NeutralOut brakeRequest = new NeutralOut();

    private double desiredPosition; // Should be ~90 or wherever initial position is
    private double desiredAngle;
    private boolean enabled = true;
    private boolean V1 = true;
    double ff = 0;



    public IntakeWrist(boolean V1) {
        this.V1 = V1;
        motor = new TalonFX(V1IntakeConstants.kMotorID);
        motorConfigurator = motor.getConfigurator();
        
        // TODO: Took out immediate stow to work on Wrist tuning
        // desiredPosition = IntakeConstants.kWristStowPosition;

        

        if(V1) {
            pigeon = new Pigeon2(V1IntakeConstants.kPigeonID, "rio");
            // desiredPosition = 89;
            desiredPosition = pigeon.getRoll().getValueAsDouble();
            motionMagicRequest = new MotionMagicVoltage(desiredPosition);
        }
        else {
            pigeon = new Pigeon2(V1IntakeConstants.kPigeonID, "rio");
            // desiredPosition = 89;
            desiredPosition = pigeon.getRoll().getValueAsDouble();


            motionMagicRequest = new MotionMagicVoltage(desiredPosition);
        }

        // configure motor
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        configurePID(motorConfigs);
        
        motor.setNeutralMode(NeutralModeValue.Brake);
        zeroEncoder();
        
    }

    //****************************** SETUP METHODS ******************************//

    private void configurePID(TalonFXConfiguration motorConfigs) {
        if (V1){
            motorConfigurator.refresh(motorConfigs);

            // motorConfigs.Feedback.FeedbackRemoteSensorID = V1IntakeConstants.kPigeonID;
            motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            motorConfigs.Feedback.SensorToMechanismRatio = 1/(5.5555555556);
            // motorConfigs.Feedback.RotorToSensorRatio;
            motorConfigs.CurrentLimits.SupplyCurrentLimit = 25;
            motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
            motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 30;
            motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;
            motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
            motorConfigs.Slot0.kP = V1IntakeConstants.kPMotor;
            motorConfigs.Slot0.kI = V1IntakeConstants.kItMotor;
            motorConfigs.Slot0.kD = V1IntakeConstants.kDMotor;
            motorConfigs.Slot0.kV = V1IntakeConstants.kVMotor;
            motorConfigs.Slot0.kS = V1IntakeConstants.kSMotor;
            motorConfigs.Slot0.kG = V1IntakeConstants.kGMotor;

            motorConfigs.MotionMagic.MotionMagicCruiseVelocity =  V1IntakeConstants.kCruiseVelocity;
            motorConfigs.MotionMagic.MotionMagicAcceleration = V1IntakeConstants.kAcceleration;
            motorConfigs.MotionMagic.MotionMagicJerk = V1IntakeConstants.kJerk;
        
            StatusCode response = motorConfigurator.apply(motorConfigs);
            if (!response.isOK()){
                DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
            }
        } else {
            motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            motorConfigs.Feedback.SensorToMechanismRatio = 12.0/54.0;
            motorConfigs.CurrentLimits.SupplyCurrentLimit = 25;
            motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
            motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 30;
            motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;
            motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
            motorConfigs.Slot0.kP = IntakeConstants.kPWristMotor;
            motorConfigs.Slot0.kI = IntakeConstants.kIWristMotor;
            motorConfigs.Slot0.kD = IntakeConstants.kDWristMotor;
            motorConfigs.Slot0.kV = IntakeConstants.kVWristMotor;
            motorConfigs.Slot0.kS = IntakeConstants.kSWristMotor;
            motorConfigs.Slot0.kG = IntakeConstants.kGWristMotor;
    
            motorConfigs.MotionMagic.MotionMagicAcceleration = V1IntakeConstants.kAcceleration;
            motorConfigs.MotionMagic.MotionMagicJerk = V1IntakeConstants.kJerk;
        
            StatusCode response = motorConfigurator.apply(motorConfigs);
            if (!response.isOK()){
                DriverStation.reportError("Could not apply motor configs, error code:" + response.toString(), new Error().getStackTrace());
            }
        }
    }

    private void zeroEncoder() {
        motor.setPosition(0);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Wrist Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Wrist Roll", pigeon.getRoll().getValueAsDouble());
        SmartDashboard.putNumber("Commanded Rotations", desiredPosition);
        SmartDashboard.putNumber("Commanded Degrees", desiredAngle);



        // ff = Math.abs(1.9 * Math.cos(pigeon.getRoll().getValueAsDouble() - 120));

        // TODO: Uncomment when ready to do position control

        ff = 0.356 + (-0.015 * desiredPosition);

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
        desiredPosition = (desiredAngle * 0.015432) - 0.00011;
        
        // double newPos = NerdyMath.clamp(
        //     positionDegrees, 
        //     V1IntakeConstants.kMinPosition, 
        //     V1IntakeConstants.kMaxPosition
        // );

        motionMagicRequest.Position = (desiredPosition);  // = (newPos / 360.0)
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
        return setPositionCommand(IntakeConstants.kWristStowPosition);
    }

    public Command moveToStation() {
        return setPositionCommand(IntakeConstants.kWristStationPosition);
    }

    public Command moveToReefL14() {
        return setPositionCommand(IntakeConstants.kWristL14Position);
    }

    public Command moveToReefL23() {
        return setPositionCommand(IntakeConstants.kWristL23Position);
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
                tab.addNumber("Coral Wrist pigeon angle", () -> pigeon.getRoll().getValueAsDouble());
                tab.addNumber("Wrist Voltage", () -> motor.getMotorVoltage().getValueAsDouble());
                tab.addNumber("Wrist FF", () -> motionMagicRequest.FeedForward);
                break;
        }
    }

}