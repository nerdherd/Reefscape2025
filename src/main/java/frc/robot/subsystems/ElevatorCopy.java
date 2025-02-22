package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorCopy extends SubsystemBase {
    private final TalonFX elevatorMotor = new TalonFX(2);
    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(
        0.15, 0.0, 0.01, new TrapezoidProfile.Constraints(200, 100)
    );
    private final double elevatorFF = 0.15;
    private final DutyCycleOut elevatorOutput = new DutyCycleOut(0.0);
    private double targetHeightTicks = 0.0;
    private double pivotAngleDegrees = 0.0; // To adjust feedforward
    private static final double TICKS_PER_METER = 4096.0 / 1.0;
    private static final double MAX_HEIGHT_TICKS = 4096.0;

    public ElevatorCopy() {
        //elevatorMotor.setInverted(InvertedValue.CounterClockwise_Positive);
        elevatorMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
        elevatorMotor.setPosition(0.0);
        elevatorPID.setTolerance(5.0);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Elevator Pos", elevatorMotor.getPosition().getValueAsDouble() * 2048);
        SmartDashboard.putNumber("Elevator Target", targetHeightTicks);
    }

    public void runPID()
    {        
        double elevatorPos = elevatorMotor.getPosition().getValueAsDouble() * 2048;
        double pivotAngleCos = Math.cos(Math.toRadians(pivotAngleDegrees));
        double ffOut = elevatorFF * pivotAngleCos; 
        double pidOut = elevatorPID.calculate(elevatorPos, targetHeightTicks);
        double control = Math.max(-1.0, Math.min(1.0, pidOut + ffOut));

        elevatorMotor.setControl(elevatorOutput.withOutput(control));
    }

    public void setTargetHeight(double heightMeters) {
        targetHeightTicks = heightMeters * TICKS_PER_METER;
        targetHeightTicks = Math.max(0, Math.min(MAX_HEIGHT_TICKS, targetHeightTicks));
    }

    public double getPosition() {
        return elevatorMotor.getPosition().getValueAsDouble() * 2048;
    }

    public void setPivotAngle(double angleDegrees) {
        this.pivotAngleDegrees = angleDegrees;
    }

    public void stop() {
        elevatorMotor.setControl(elevatorOutput.withOutput(0));
        targetHeightTicks = getPosition();
    }

    public boolean atSetpoint() {
        return elevatorPID.atGoal();
    }
}
