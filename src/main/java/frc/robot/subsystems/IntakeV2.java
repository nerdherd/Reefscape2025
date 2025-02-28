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
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ClawConstants;

public class IntakeV2 extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final TalonFX positionMotor;
    private final TalonFXConfigurator rollerConfigurator;
    private final TalonFXConfigurator positionConfigurator;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final NeutralOut brakeRequest = new NeutralOut();

    private double desiredPosition;
    private double desiredVelocity;
    private boolean enabled = false;

    public IntakeV2() {
        rollerMotor = new TalonFX(RollerConstants.kMotorID);
        rollerConfigurator = rollerMotor.getConfigurator();


        positionMotor = new TalonFX(ClawConstants.kMotorID);
        positionConfigurator = positionMotor.getConfigurator();

        TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
        TalonFXConfiguration positionConfigs = new TalonFXConfiguration();


        configurePID(rollerConfigs, positionConfigs);
        zeroEncoder();
    }

    //****************************** SETUP METHODS ******************************//

    private void configurePID(TalonFXConfiguration rollerConfigs, TalonFXConfiguration positionConfigs) {
        // roller configs
        rollerConfigurator.refresh(rollerConfigs);

        rollerConfigs.Slot0.kP = RollerConstants.kPMotor;
        rollerConfigs.Slot0.kI = RollerConstants.kIMotor;
        rollerConfigs.Slot0.kD = RollerConstants.kDMotor;
        rollerConfigs.Slot0.kV = RollerConstants.kVMotor;
 
        StatusCode rollerResponse = rollerConfigurator.apply(rollerConfigs);
        if (!rollerResponse.isOK())
            DriverStation.reportError("Could not apply roller PID configs, error code:" + rollerResponse.toString(), new Error().getStackTrace());
        rollerConfigurator.refresh(rollerConfigs);


        // position configs
        positionConfigurator.refresh(positionConfigs);
        positionConfigs.Slot0.kP = ClawConstants.kPMotor;
        positionConfigs.Slot0.kI = ClawConstants.kIMotor;
        positionConfigs.Slot0.kD = ClawConstants.kDMotor;
        positionConfigs.Slot0.kV = ClawConstants.kVMotor;
        positionConfigs.Slot0.kS = ClawConstants.kSMotor;
        positionConfigs.Slot0.kG = ClawConstants.kGMotor;

        positionConfigs.Feedback.SensorToMechanismRatio = 42.0 / 18.0 * 5.0;

        positionConfigs.MotionMagic.MotionMagicCruiseVelocity =  ClawConstants.kCruiseVelocity;
        positionConfigs.MotionMagic.MotionMagicAcceleration = ClawConstants.kAcceleration;
        positionConfigs.MotionMagic.MotionMagicJerk = ClawConstants.kJerk;
    
        StatusCode positionResponse = positionConfigurator.apply(positionConfigs);
        if (!positionResponse.isOK()){
            DriverStation.reportError("Could not apply claw PID configs, error code:" + positionResponse.toString(), new Error().getStackTrace());
        } 
    }

    private void zeroEncoder() {
        positionMotor.setPosition(0);    
    }

    @Override
    public void periodic() {
        double ff = 0.37 * Math.cos(positionMotor.getPosition().getValueAsDouble() * 2 * Math.PI);

        if (!enabled){
            rollerMotor.setControl(brakeRequest);
            positionMotor.setControl(brakeRequest);
        } else {
            rollerMotor.setControl(velocityRequest);
            positionMotor.setControl(motionMagicRequest.withFeedForward(ff));
        }

        SmartDashboard.putNumber("Roller Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Roller Current Rotations", rollerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Roller Desired Velocity", desiredVelocity);
        SmartDashboard.putNumber("Roller Current Velocity", rollerMotor.getVelocity().getValueAsDouble());
        
        SmartDashboard.putNumber("Claw Wrist Voltage", positionMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Claw Current Rotations", positionMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Claw Desired Rotations", desiredPosition);
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
                setPositionCommand(ClawConstants.kAlgaePosition),
                setVelocityCommand(RollerConstants.kIntakePower)
            )
        );
    }
    
    public Command intakeCoral() {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.parallel(
                setPositionCommand(ClawConstants.kCoralPosition),
                setVelocityCommand(RollerConstants.kIntakePower)
            )
        );
    }

    public Command goToStow() {
        return Commands.sequence(
            setEnabledCommand(true),
            setPositionCommand(ClawConstants.kStowPosition),
            setVelocityCommand(0)  
        );
    }
}