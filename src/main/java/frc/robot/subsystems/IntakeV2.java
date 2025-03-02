package frc.robot.subsystems;

import org.ejml.dense.row.decomposition.eig.WatchedDoubleStepQRDecomposition_FDRM;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ClawConstants;

public class IntakeV2 extends SubsystemBase implements Reportable{
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
    private double pivotAngle = 0;
    private double wristAngle = 0.0; //need to find

    private double voltage = 0.0;

    public IntakeV2() {
        rollerMotor = new TalonFX(RollerConstants.kMotorID);
        rollerConfigurator = rollerMotor.getConfigurator();


        positionMotor = new TalonFX(ClawConstants.kMotorID);
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

    public void zeroEncoder() {
        positionMotor.setPosition(0);    
    }

    @Override
    public void periodic() {
        double ff = 0.4; // 0.4 * Math.cos((positionMotor.getPosition().getValueAsDouble() + pivotAngle + wristAngle) * 2 * Math.PI);// 0.788

        if (!enabled){
            rollerMotor.setControl(brakeRequest);
            positionMotor.setControl(brakeRequest);
        } else {
            // rollerMotor.setControl(velocityRequest);
            rollerMotor.setVoltage(voltage);
            positionMotor.setControl(motionMagicRequest.withFeedForward(ff));
        }

        SmartDashboard.putNumber("Roller Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Roller Current Rotations", rollerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Roller Desired Velocity", desiredVelocity);
        SmartDashboard.putNumber("Roller Current Velocity", rollerMotor.getVelocity().getValueAsDouble());
        
        SmartDashboard.putNumber("Claw Wrist Voltage", positionMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Claw Current Rotations", positionMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Claw Desired Rotations", desiredPosition);
        SmartDashboard.putNumber("Claw ff", ff);
    }

    // ****************************** STATE METHODS ***************************** //

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    private void setVelocity(double velocity) {
        // desiredVelocity = velocity;
        // velocityRequest.Velocity = velocity;
        if(velocity == 0.0) {
            voltage = 0.0;
            return;
        }
        voltage = velocity < 0.0 ? -2.0 : 2.0;
    }

    public void setJawPosition(double position){
        desiredPosition = position;
        motionMagicRequest.Position = desiredPosition;
    }

    public void setPivotAngle(double pivotAngle) {
        this.pivotAngle = pivotAngle;
    }

    public void setWristAngle(double wristAngle) {
        this.wristAngle = wristAngle;
    }

    // ****************************** COMMAND METHODS ****************************** //

    public Command setJawPositionCommand(double position) {
        return Commands.runOnce(() -> setJawPosition(position));
    }

    public Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public Command setEnabledCommand(boolean enable) {
        return Commands.runOnce(() -> setEnabled(enable));
    }

    public Command stopJawCommand() { //TODO
        return Commands.sequence(
            Commands.runOnce(() -> positionMotor.setControl(brakeRequest))
        );
    }

    public Command stopRollerCommand() { //TODO
        return Commands.sequence(
            Commands.runOnce(() -> rollerMotor.setControl(brakeRequest))
        );
    }

     // ****************************** NAMED COMMANDS ****************************** //
     
    public Command intakeAlgae() {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.parallel(
                setJawPositionCommand(ClawConstants.kAlgaePosition),
                setVelocityCommand(RollerConstants.kIntakePower)
            )
        );
    }
    
    public Command intakeCoral() {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.parallel(
                setJawPositionCommand(ClawConstants.kCoralPosition),
                setVelocityCommand(RollerConstants.kIntakePower)
            )
        );
    }

    public Command outtakeAlgae() {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.parallel(
                setJawPositionCommand(ClawConstants.kAlgaePosition),
                setVelocityCommand(RollerConstants.kOuttakePower)
            )
        );
    }
    
    public Command outtakeCoral() {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.parallel(
                setJawPositionCommand(ClawConstants.kCoralPosition),
                setVelocityCommand(RollerConstants.kOuttakePower)
            )
        );
    }

    public Command goToStow() {
        return Commands.sequence(
            setEnabledCommand(true),
            setJawPositionCommand(ClawConstants.kStowPosition),
            setVelocityCommand(0)  
        );
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reportToSmartDashboard'");
    }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        ShuffleboardTab tab = Shuffleboard.getTab("Claw");
        tab.addNumber("Desired Position", () -> desiredPosition);
        tab.addNumber("Current Position", () -> positionMotor.getPosition().getValueAsDouble());
        tab.addNumber("Positional Voltage", () -> positionMotor.getMotorVoltage().getValueAsDouble());
    }
}