package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ClawConstants;

public class IntakeV2 extends SubsystemBase implements Reportable{
    private final TalonFX rollerMotor;
    private final TalonFX clawMotor;
    private final TalonFXConfigurator rollerConfigurator;
    private final TalonFXConfigurator clawConfigurator;

    // private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private final MotionMagicVoltage motionMagicRequestClaw = new MotionMagicVoltage(0);
    private final NeutralOut brakeRequest = new NeutralOut();

    private double desiredPositionClaw;
    private double desiredVelocity;
    private boolean enabled = false;
    private double pivotAngle = 0;
    private double wristAngle = 0.0; //need to find

    private double voltage = 0.0;
    double ff = 0;

    public IntakeV2() {
        rollerMotor = new TalonFX(RollerConstants.kMotorID);
        rollerConfigurator = rollerMotor.getConfigurator();


        clawMotor = new TalonFX(ClawConstants.kMotorID);
        clawConfigurator = clawMotor.getConfigurator();

        TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
        TalonFXConfiguration clawConfigs = new TalonFXConfiguration();

        configurePID(rollerConfigs, clawConfigs);
        zeroEncoder();
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    //****************************** SETUP METHODS ******************************//

    private void configurePID(TalonFXConfiguration rollerConfigs, TalonFXConfiguration clawConfigs) {
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
        clawConfigurator.refresh(clawConfigs);
        clawConfigs.Slot0.kP = ClawConstants.kPMotor;
        clawConfigs.Slot0.kI = ClawConstants.kIMotor;
        clawConfigs.Slot0.kD = ClawConstants.kDMotor;
        clawConfigs.Slot0.kV = ClawConstants.kVMotor;
        clawConfigs.Slot0.kS = ClawConstants.kSMotor;
        clawConfigs.Slot0.kG = ClawConstants.kGMotor;

        clawConfigs.Feedback.SensorToMechanismRatio = 42.0 / 18.0 * 5.0;

        clawConfigs.MotionMagic.MotionMagicCruiseVelocity =  ClawConstants.kCruiseVelocity;
        clawConfigs.MotionMagic.MotionMagicAcceleration = ClawConstants.kAcceleration;
        clawConfigs.MotionMagic.MotionMagicJerk = ClawConstants.kJerk;
    
        StatusCode clawResponse = clawConfigurator.apply(clawConfigs);
        if (!clawResponse.isOK()){
            DriverStation.reportError("Could not apply claw PID configs, error code:" + clawResponse.toString(), new Error().getStackTrace());
        } 
    }

    public void zeroEncoder() {
        clawMotor.setPosition(0);    
        desiredPositionClaw = 0;
    }

    @Override
    public void periodic() {
        ff = 0.4; // 0.4 * Math.cos((positionMotor.getPosition().getValueAsDouble() + pivotAngle + wristAngle) * 2 * Math.PI);// 0.788

        if (!enabled){
            rollerMotor.setControl(brakeRequest);
            clawMotor.setControl(brakeRequest);
        } else {
            // rollerMotor.setControl(velocityRequest);
            rollerMotor.setVoltage(voltage);
            clawMotor.setControl(motionMagicRequestClaw.withFeedForward(ff));
        }
    }

    // ****************************** STATE METHODS ***************************** //

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    // TODO use voltage directly??
    private void setVelocity(double velocity) {
        // desiredVelocity = velocity;
        // velocityRequest.Velocity = velocity;
        // if (velocity == 0.0) voltage = 0.0;
        // else if (velocity < 0.0) voltage = -2.0;
        // else voltage = 2.0;
        voltage = velocity;
        // voltage = velocity == 0.0 ? 0.0 : (velocity < 0.0 ? -2.0 : 2.0);
    }

    public void setClawPosition(double position){
        desiredPositionClaw = position;
        motionMagicRequestClaw.Position = desiredPositionClaw;
    }

    public void setPivotAngle(double pivotAngle) {
        this.pivotAngle = pivotAngle;
    }

    public void setWristAngle(double wristAngle) {
        this.wristAngle = wristAngle;
    }

    // ****************************** COMMAND METHODS ****************************** //

    public Command setClawPositionCommand(double position) {
        return Commands.runOnce(() -> setClawPosition(position));
    }

    public Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public Command setEnabledCommand(boolean enable) {
        return Commands.runOnce(() -> setEnabled(enable));
    }

    public Command stopClawCommand() { //TODO
        return Commands.sequence(
            Commands.runOnce(() -> clawMotor.setControl(brakeRequest))
        );
    }

    public Command stopRollerCommand() { //TODO
        return Commands.sequence(
            Commands.runOnce(() -> rollerMotor.setVoltage(0))
        );
    }

     // ****************************** NAMED COMMANDS ****************************** //
     
    /*public Command intakeAlgae() {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.parallel(
                setClawPositionCommand(ClawConstants.kAlgaePosition),
                setVelocityCommand(RollerConstants.kIntakePower)
            )
        );
    }
    
    public Command intakeCoral() {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.parallel(
                setClawPositionCommand(ClawConstants.kCoralPosition),
                setVelocityCommand(RollerConstants.kIntakePower)
            )
        );
    }

    public Command outtakeAlgae() {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.parallel(
                setClawPositionCommand(ClawConstants.kAlgaePosition),
                setVelocityCommand(RollerConstants.kOuttakePower)
            )
        );
    }
    
    public Command outtakeCoral() {
        return Commands.sequence(
            setEnabledCommand(true),
            Commands.parallel(
                setClawPositionCommand(ClawConstants.kCoralPosition),
                setVelocityCommand(RollerConstants.kOuttakePower)
            )
        );
    }

    public Command goToStow() {
        return Commands.sequence(
            setEnabledCommand(true),
            setClawPositionCommand(ClawConstants.kStowPosition),
            setVelocityCommand(0)  
        );
    }*/

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {       

        SmartDashboard.putNumber("Roller Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Roller Current Rotations", rollerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Roller Desired Velocity", desiredVelocity);
        SmartDashboard.putNumber("Roller Current Velocity", rollerMotor.getVelocity().getValueAsDouble());
        
        SmartDashboard.putNumber("Claw Wrist Voltage", clawMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Claw Current Rotations", clawMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Claw Desired Rotations", desiredPositionClaw);
        SmartDashboard.putNumber("Claw ff", ff);
    }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        ShuffleboardTab tab = Shuffleboard.getTab("Claw");
        tab.addNumber("Desired Position", () -> desiredPositionClaw);
        tab.addNumber("Current Position", () -> clawMotor.getPosition().getValueAsDouble());
        tab.addNumber("Positional Voltage", () -> clawMotor.getMotorVoltage().getValueAsDouble());
    }
}