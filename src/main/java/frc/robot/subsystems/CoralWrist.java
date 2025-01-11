package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralWristConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.NerdyMath;

public class CoralWrist extends SubsystemBase implements Reportable{
    private TalonFX wristMotor;
    private TalonFXConfigurator wristConfigurator;

    private final PIDController wristPID;
    private double desiredPosition;
    private double desiredVelocity;
    public boolean enabled = true;


    public CoralWrist() {
        wristMotor = new TalonFX(ElevatorConstants.kElevatorMotorID);
        wristConfigurator = wristMotor.getConfigurator();
        wristPID = new PIDController(1, 0, 0);
    }

   

    public void init() {
        desiredPosition = CoralWristConstants.kCoralWristStationPosition;
        desiredVelocity = 0;
        stop();
    }

    public void periodic() {
        if (!enabled){
            stop();
        }
        else {
            goToPosition(desiredPosition);
        }
    }

    public void stop() {
        wristMotor.set(0);
    }

     public Command goToPosition(double targetPosition) {
        return Commands.sequence(
            Commands.runOnce(() -> desiredPosition = targetPosition),
            Commands.runOnce(() -> wristConfigurator.setPosition(targetPosition)),
            Commands.run(() -> setVelocity(wristPID.calculate(wristMotor.getPosition().getValueAsDouble()))),
            Commands.waitUntil(()-> NerdyMath.inRange(
                wristMotor.getPosition().getValueAsDouble(),
                targetPosition - CoralWristConstants.kCoralWristDeadband,
                targetPosition + CoralWristConstants.kCoralWristDeadband
            )),
            Commands.runOnce(() -> stop())
        );
    }

    public Command setVelocity(double velocity) {
        return Commands.sequence(
            Commands.runOnce(() -> desiredVelocity = velocity),
            Commands.run(() -> wristMotor.set(velocity))
        );
    }

    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
            case MINIMAL:
                SmartDashboard.putNumber("CoralWrist Desired Position", desiredPosition);
                SmartDashboard.putNumber("CoralWrist Current Position", wristMotor.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("CoralWrist Desired Velocity", desiredVelocity);
                SmartDashboard.putNumber("CoralWrist Current Velocity", wristMotor.getVelocity().getValueAsDouble());
                
        }
    }

    public void initShuffleboard(LOG_LEVEL level) {
        ShuffleboardTab tab;
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
            tab = Shuffleboard.getTab("Coral Wrist");
                tab.addNumber("CoralWrist Desired Velocity", () -> desiredVelocity);
                tab.addNumber("CoralWrist Current Velocity", () -> wristMotor.getVelocity().getValueAsDouble());
            case MINIMAL:
                tab = Shuffleboard.getTab("Main");
                tab.addNumber("CoralWrist Desired Position", () -> desiredPosition);
                tab.addNumber("CoralWrist Current Position", () -> wristMotor.getPosition().getValueAsDouble());
                break;
        }
    }
}
