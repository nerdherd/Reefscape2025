package frc.robot.subsystems;


import java.lang.annotation.Target;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.NerdyMath;



public class Elevator extends SubsystemBase implements Reportable {
    private final TalonFX elevatorMotor;

    private final PIDController elevatorPID;
    private double desiredPosition;
    private double desiredVelocity;
    public boolean enabled = false;

    public Elevator() {
        elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorID, "rio");
        elevatorPID = new PIDController(0.2, 0, 0); // 1, 0, 0

        elevatorMotor.setPosition(0.0);
    }
    
    @Override
    public void periodic() {
        if (enabled) {
            setVelocity((elevatorPID.calculate(elevatorMotor.getPosition().getValueAsDouble(), desiredPosition))/2);
            if (desiredPosition == ElevatorConstants.kElevatorStowPosition) {
                setVelocity((elevatorPID.calculate(elevatorMotor.getPosition().getValueAsDouble(), desiredPosition))/2);
            }
        }
        else {
            setVelocity((elevatorPID.calculate(elevatorMotor.getPosition().getValueAsDouble(), ElevatorConstants.kElevatorStowPosition))/2);
        }
    }

    // STATE METHODS //

    public Command setDisabledCommand() {
        return Commands.runOnce(() -> this.setEnabled(false));
    }
    
    public void setEnabled(boolean e) {
        this.enabled = e;
        if (!e) desiredPosition = ElevatorConstants.kElevatorStowPosition;
    }

    public Command setEnabledCommand() {
        return Commands.runOnce(() -> this.setEnabled(true));
    }
    
    public Command goToPosition(double targetPosition) {
        return Commands.sequence(
            setEnabledCommand(),
            Commands.runOnce(() -> { desiredPosition = targetPosition; })
        );
    }

    public void setVelocity(double velocity) {
        desiredVelocity = Math.min(Math.max(velocity, -ElevatorConstants.kElevatorSpeed), ElevatorConstants.kElevatorSpeed);
        elevatorMotor.set(desiredVelocity);
    }
    
    public Command setVelocityCommand(double velocity) {
        Command command = Commands.runOnce(() -> setVelocity(velocity));
        command.addRequirements(this);
        return command;
    }

    // NAMED COMMANDS //

    public Command moveToReefL1() {
        return goToPosition(ElevatorConstants.kElevatorL1Position);
    }

    public Command moveToReefL2() {
        return goToPosition(ElevatorConstants.kElevatorL2Position);
    }

    public Command moveToReefL3() {
        return goToPosition(ElevatorConstants.kElevatorL3Position);
    }

    public Command moveToReefL4() {
        return goToPosition(ElevatorConstants.kElevatorL4Position);
    }

    // LOG METHODS //

    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
            case MINIMAL:
                SmartDashboard.putNumber("Elevator Desired Position", desiredPosition);
                SmartDashboard.putNumber("Elevator Current Position", elevatorMotor.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("Elevator Desired Velocity", desiredVelocity);
                SmartDashboard.putNumber("Elevator Current Velocity", elevatorMotor.getVelocity().getValueAsDouble());
                SmartDashboard.putBoolean("Elevator Enabled", this.enabled);
        }
    }

    public void initShuffleboard(LOG_LEVEL level) {
        ShuffleboardTab tab;
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
            case MINIMAL:
                tab = Shuffleboard.getTab("Elevator");
                tab.addNumber("Elevator Desired Velocity", () -> desiredVelocity);
                tab.addNumber("Elevator Current Velocity", () -> elevatorMotor.getVelocity().getValueAsDouble());
                tab.addNumber("Elevator Desired Position", () -> desiredPosition);
                tab.addNumber("Elevator Current Position", () -> elevatorMotor.getPosition().getValueAsDouble());
                tab.addBoolean("Elevator Enabled", () -> enabled);
                break;
        }
    }

}
