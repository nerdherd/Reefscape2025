package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.V1IntakeConstants;
import frc.robot.subsystems.Reportable.LOG_LEVEL;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
Use correct Controller
Reset the encoder with resetEncoder() when the arm is NEUTRAL down.
Move the arm to 90° manually and measure the voltage needed to hold it there.
Set kF to that voltage (e.g., if 1.5V holds it, kF = 1.5).
Command 90° and check if the arm moves to horizontal and holds steady.
Adjust kF if it droops at 90°.
*/

public class IntakeWristCopy extends SubsystemBase {
    // Hardware declarations
    private final TalonFX motor = new TalonFX(V1IntakeConstants.kMotorID); 

    // Profiled PID Controller
    //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html
    private final ProfiledPIDController pidController;
    
    // Constants (tune these values)
    private static final double kP = 0.25;    // Proportional gain (volts/degree)
    private static final double kI = 0.0;   // Integral gain (volts/degree-second)
    private static final double kD = 0.0;   // Derivative gain (volts/degree-per-second)
    private static final double kF = 0.6;    // Feedforward gain (volts) Set to the voltage needed to hold the arm at 90° (e.g., 1.0–2.0V).
    private static final double TOLERANCE = 2.0; // Degrees tolerance
    private static final double MAX_VOLTAGE = 3.0;// 2 initially to avoid damage.//8.0; // Max voltage (volts)
    private static final double MAX_VELOCITY = V1IntakeConstants.kCruiseVelocity; // Degrees per second
    private static final double MAX_ACCELERATION = 90.0; // Degrees per second squared
    
    // Position constants (in degrees)
    private static final double DOWN_POSITION = 0.0;   // Arm fully down
    private static final double NEUTRAL_POSITION = 90.0; // Arm horizontal
    private static final double MAX_POSITION = 210.0;  // Max position
    private double setpoint = DOWN_POSITION; // Target position in degrees

    // Encoder configuration
    private static final double GEAR_RATIO = 13.89; //it means 6 motor rotations equal 1 arm rotation.
    private static final double DEGREES_PER_ARM_ROTATION = 360.0;

    // Voltage control request
    private final VoltageOut voltageControl = new VoltageOut(0);

    public IntakeWristCopy() {
        // Configure Talon FX
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // JD?
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive voltage moves arm up
        config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        // config.Feedback.RotorToSensorRatio = 1.0; //JD
        motor.getConfigurator().apply(config);
        
        // Configure Profiled PID controller
        pidController = new ProfiledPIDController(
            kP, kI, kD,
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION)
        );
        pidController.setTolerance(TOLERANCE);
        
        // Initial setpoint
        setpoint = getArmPosition();
    }

    @Override
    public void periodic() {
        runPID();
    }

    // Get current arm position in degrees
    public double getArmPosition() {
        double rotorPosition = motor.getPosition().getValueAsDouble(); // Rotations
        return rotorPosition * DEGREES_PER_ARM_ROTATION; // Degrees
    }

    // Set desired arm position
    public void setArmPosition(double position) {
        setpoint = Math.max(DOWN_POSITION, Math.min(MAX_POSITION, position));
        pidController.setGoal(setpoint);
    }

    // Manual voltage control
    public void setArmVoltage(double voltage) {
        double limitedVoltage = Math.max(-MAX_VOLTAGE, Math.min(MAX_VOLTAGE, voltage));
        motor.setControl(voltageControl.withOutput(limitedVoltage));
        pidController.reset(getArmPosition());
    }

    // PID voltage control with profiling
    public void runPID() {
        double currentPosition = getArmPosition();
        double pidOutput = pidController.calculate(currentPosition); // Output in volts
        
        // Feedforward: proportional to sin(angle), max at 90°
        double feedForward = Math.sin(Math.toRadians(currentPosition)) * kF;
        double totalVoltage = pidOutput + feedForward;
        
        // Limit voltage
        totalVoltage = Math.max(-MAX_VOLTAGE, Math.min(MAX_VOLTAGE, totalVoltage));
        
        // Apply voltage
        motor.setControl(voltageControl.withOutput(totalVoltage));
    }

    // Check if arm is at setpoint
    public boolean atSetpoint() {
        return pidController.atGoal();
    }

    // Stop the arm
    public void stop() {
        motor.setControl(voltageControl.withOutput(0));
        setpoint = getArmPosition();
    }

    public void setPivotAngle(double baseAngle)
    {

    }

    // Reset encoder to zero (call when arm is fully down)
    public void resetEncoder() {
        motor.setPosition(0);
        setpoint = DOWN_POSITION;
        pidController.reset(getArmPosition());
    }

    
    // @Override
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
                // tab.addDouble("MM Position", () -> motionMagicRequest.Position);
                // tab.addDouble("Desired Position", () -> desiredPosition);
            case MINIMAL:
            
            tab.addNumber("Arm Position", ()->getArmPosition());
            tab.addNumber("Arm Setpoint", ()->setpoint);
            tab.addNumber("Rotor Position", ()->motor.getPosition().getValueAsDouble());
            tab.addNumber("Arm Voltage", ()->motor.getMotorVoltage().getValueAsDouble());
        
                // tab.addBoolean("Enabled", () -> enabled);
                tab.addNumber("Current Coral Wrist Angle", () -> motor.getPosition().getValueAsDouble());
                // tab.addNumber("Coral Wrist pigeon angle", () -> pigeon.getRoll().getValueAsDouble());
                tab.addNumber("Wrist Voltage", () -> motor.getMotorVoltage().getValueAsDouble());
                // tab.addNumber("Wrist FF", () -> motionMagicRequest.FeedForward);
                break;
        }
    }
}