
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorPivotCopy extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(1);
    private final ProfiledPIDController pivotPID = new ProfiledPIDController(
        0.1, 0.0, 0.01, new TrapezoidProfile.Constraints(100, 50)
    );
    private final double pivotFF = 0.1;
    private final DutyCycleOut pivotOutput = new DutyCycleOut(0.0);
    private double targetAngleTicks = 0.0;
    private double elevatorPos = 0.0;
    private static final double TICKS_PER_DEGREE = 2048.0 / 360.0;
    private static final double ELEVATOR_MAX_TICKS = 4096.0;

    public ElevatorPivotCopy() {
        // Configure Talon FX
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // JD?
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive voltage moves arm up
        config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = 20;//GEAR_RATIO;
        // config.Feedback.RotorToSensorRatio = 1.0; //JD
        pivotMotor.getConfigurator().apply(config);

        pivotMotor.setPosition(0.0);
        pivotPID.setTolerance(5.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Pos", pivotMotor.getPosition().getValueAsDouble() * 2048);
        SmartDashboard.putNumber("Pivot Target", targetAngleTicks);
    }

    public void runPID()
    {        
        double pivotPos = pivotMotor.getPosition().getValueAsDouble() * 2048;
        double elevatorExtensionFactor = elevatorPos / ELEVATOR_MAX_TICKS;
        double pivotAngleCos = Math.cos(Math.toRadians(pivotPos / 2048.0 * 360.0));
        double ffOut = pivotFF + (0.2 * elevatorExtensionFactor * pivotAngleCos);

        double pidOut = pivotPID.calculate(pivotPos, targetAngleTicks);
        double control = Math.max(-1.0, Math.min(1.0, pidOut + ffOut));

        pivotMotor.setControl(pivotOutput.withOutput(control));
    }

    public void setTargetAngle(double angleDegrees) {
        targetAngleTicks = angleDegrees * TICKS_PER_DEGREE;
        targetAngleTicks = Math.max(-2048, Math.min(2048, targetAngleTicks));
    }

    public void setElevatorPosition(double elevatorTicks) {
        this.elevatorPos = elevatorTicks;
    }

    public double getAngleDegrees() {
        return (pivotMotor.getPosition().getValueAsDouble() * 2048) / TICKS_PER_DEGREE;
    }

    public void stop() {
        pivotMotor.setControl(pivotOutput.withOutput(0));
        targetAngleTicks = getAngleDegrees();
    }

    public boolean atSetpoint() {
        return pivotPID.atGoal();
    }
}
