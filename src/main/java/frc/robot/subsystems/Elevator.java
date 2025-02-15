package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.NerdyMath;

public class Elevator extends SubsystemBase implements Reportable {
    private final TalonFX elevatorMotor;

    private final PIDController elevatorPID;
    private double desiredPosition;
    private double desiredVelocity;
    private boolean enabled = false;

    public Elevator(Orchestra orchestra) {
        elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorID, "rio");
        elevatorPID = new PIDController(0.2, 0, 0); // 1, 0, 0
        elevatorMotor.setPosition(0.0);
        orchestra.addInstrument(elevatorMotor);
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

    // ****************************** STATE METHODS ****************************** //

    private void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) desiredPosition = ElevatorConstants.kElevatorStowPosition;
    }
    
    private void setPosition(double position) {
        desiredPosition = position;
    }

    private void setVelocity(double velocity) {
        desiredVelocity = NerdyMath.clamp(velocity, -ElevatorConstants.kElevatorSpeed, ElevatorConstants.kElevatorSpeed);
        elevatorMotor.set(desiredVelocity);
    }

    // ****************************** COMMAND METHODS ***************************** //

    private Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> this.setEnabled(enabled));
    }

    private Command setPositionCommand(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    private Command goToPosition(double position) {
        return Commands.sequence(
            setEnabledCommand(true),
            setPositionCommand(position)
        );
    }

    private Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    private Command stopCommand() {
        return Commands.sequence(
            setVelocityCommand(0),
            setEnabledCommand(false)
        );
    }

    // ****************************** NAMED COMMANDS ****************************** //

    public Command stow() {
        return goToPosition(ElevatorConstants.kElevatorStowPosition);
    }

    public Command moveToStation() {
        return goToPosition(ElevatorConstants.kElevatorStationPosition);
    }

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
            case MEDIUM:
            case MINIMAL:
                SmartDashboard.putNumber("Elevator Desired Position", desiredPosition);
                SmartDashboard.putNumber("Elevator Current Position", elevatorMotor.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("Elevator Desired Velocity", desiredVelocity);
                SmartDashboard.putNumber("Elevator Current Velocity", elevatorMotor.getVelocity().getValueAsDouble());
                SmartDashboard.putBoolean("Elevator Enabled", this.enabled);
        }
    }

    @Override
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
