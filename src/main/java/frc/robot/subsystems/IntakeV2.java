package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;

public class IntakeV2 extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final TalonFX positionMotor;
    private final TalonFXConfigurator rollerConfigurator;
    private final TalonFXConfigurator positionConfigurator;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final NeutralOut brakeRequest = new NeutralOut();

    private double desiredPosition;
    private double desiredAngle;
    private double desiredVelocity;
    private boolean enabled = false;

    public IntakeV2() {
        rollerMotor = new TalonFX(IntakeConstants.kRollerMotorID);
        rollerConfigurator = rollerMotor.getConfigurator();


        positionMotor = new TalonFX(IntakeConstants.kPositionMotorID);
        positionConfigurator = positionMotor.getConfigurator();

        TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
        TalonFXConfiguration positionConfigs = new TalonFXConfiguration();


        configurePID(rollerConfigs, positionConfigs);
        // zeroEncoder();
    }

    //****************************** SETUP METHODS ******************************//

    private void configurePID(TalonFXConfiguration rollerConfigs, TalonFXConfiguration positionConfigs) {
        // roller configs
        rollerConfigurator.refresh(rollerConfigs);

        rollerConfigs.Slot0.kP = IntakeConstants.kPRollerMotor;
        rollerConfigs.Slot0.kI = IntakeConstants.kIRollerMotor;
        rollerConfigs.Slot0.kD = IntakeConstants.kDRollerMotor;
        rollerConfigs.Slot0.kV = IntakeConstants.kVRollerMotor;
 
        StatusCode rollerResponse = rollerConfigurator.apply(rollerConfigs);
        if (!rollerResponse.isOK())
            DriverStation.reportError("Could not apply PID configs, error code:" + rollerResponse.toString(), new Error().getStackTrace());
        rollerConfigurator.refresh(rollerConfigs);


        // position configs
        positionConfigurator.refresh(positionConfigs);
        positionConfigs.Slot0.kP = IntakeConstants.kPPositionMotor;
        positionConfigs.Slot0.kI = IntakeConstants.kIPositionMotor;
        positionConfigs.Slot0.kD = IntakeConstants.kDPositionMotor;
        positionConfigs.Slot0.kV = IntakeConstants.kVPositionMotor;
        positionConfigs.Slot0.kS = IntakeConstants.kSPositionMotor;
        positionConfigs.Slot0.kG = IntakeConstants.kGPositionMotor;

        positionConfigs.MotionMagic.MotionMagicCruiseVelocity =  WristConstants.kCruiseVelocity;
        positionConfigs.MotionMagic.MotionMagicAcceleration = WristConstants.kAcceleration;
        positionConfigs.MotionMagic.MotionMagicJerk = WristConstants.kJerk;
    
        StatusCode positionResponse = positionConfigurator.apply(positionConfigs);
        if (!positionResponse.isOK()){
            DriverStation.reportError("Could not apply motor configs to position motor, error code:" + positionResponse.toString(), new Error().getStackTrace());
        } 
    }

    private void zeroEncoder() {
        // rollerMotor.setPosition(0);
        positionMotor.setPosition(0);    
    }

    @Override
    public void periodic() {
        if (!enabled){
            rollerMotor.setControl(brakeRequest);
            positionMotor.setControl(brakeRequest);
        } else {
            rollerMotor.setControl(velocityRequest);
            positionMotor.setControl(motionMagicRequest);
        }

        SmartDashboard.putNumber("Roller Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Roller Current Rotations", rollerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Roller Desired Velocity", desiredVelocity);
        SmartDashboard.putNumber("Roller Current Velocity", rollerMotor.getVelocity().getValueAsDouble());
        
        SmartDashboard.putNumber("Position Wrist Voltage", positionMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Position Current Rotations", positionMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Position Desired Rotations", desiredPosition);
    }

    // ****************************** STATE METHODS ***************************** //

    private void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    private void setVelocity(double velocity) {
        desiredVelocity = velocity;
        velocityRequest.Velocity = velocity;
    }

    private void setPosition(double position){
        desiredPosition = position;
        motionMagicRequest.Position = desiredPosition;
    }

    // ****************************** COMMAND METHODS ****************************** //

    private Command setPositionCommand(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    private Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public Command setEnabledCommand(boolean enable) {
        return Commands.runOnce(() -> setEnabled(enable));
    }

     // ****************************** NAMED COMMANDS ****************************** //
    public Command intakeAlgae() {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.parallel(
                setPositionCommand(IntakeConstants.kAlgaePosition),
                setVelocityCommand(IntakeConstants.kIntakePower)
            )
        );
    }
    
    public Command intakeCoral() {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.parallel(
                setPositionCommand(IntakeConstants.kCoralPosition),
                setVelocityCommand(IntakeConstants.kIntakePower)
            )
        );
    }

    public Command goToStow() {
        return Commands.sequence(
            setEnabledCommand(true),
            setPositionCommand(IntakeConstants.kStowPosition),
            setVelocityCommand(0)  
        );
    }
}