package frc.robot.commands;

import static frc.robot.Constants.SwerveDriveConstants.kDriveAlpha;
import static frc.robot.Constants.SwerveDriveConstants.kDriveKinematics;
import static frc.robot.Constants.SwerveDriveConstants.kMinimumMotorOutput;
import static frc.robot.Constants.SwerveDriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
import static frc.robot.Constants.SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
import static frc.robot.Constants.SwerveDriveConstants.kTeleMaxAcceleration;
import static frc.robot.Constants.SwerveDriveConstants.kTeleMaxDeceleration;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.util.filters.OldDriverFilter2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.util.filters.DeadbandFilter;
import frc.robot.util.filters.Filter;
import frc.robot.util.filters.FilterSeries;
import frc.robot.util.filters.ScaleFilter;
public class SwerveJoystickCommand extends Command {
    private final SwerveDrivetrain swerveDrive;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> towSupplier, precisionSupplier;
    private final Supplier<Boolean> moveLeft, moveRight;
    private final Supplier<Integer> zoneId;
    private final Supplier<Double> desiredAngle;
    private final Supplier<Boolean> turnToAngleSupplier;
    private final PIDController turnToAngleController;
    private Filter xFilter, yFilter, turningFilter;

    public static double targetAngle = 0;

    public enum DodgeDirection {
        LEFT,
        RIGHT,
        NONE
    }

    private boolean wasLeftPressed = false; 
    private boolean wasRightPressed = false;

    /**
     * Construct a new SwerveJoystickCommand
     * 
     * @param swerveDrive           The Swerve Drive subsystem
     * @param xSpdFunction          A supplier returning the desired x speed
     * @param ySpdFunction          A supplier returning the desired y speed
     * @param turningSpdFunction    A supplier returning the desired turning speed
     * @param fieldOrientedFunction A boolean supplier that toggles field oriented/robot oriented mode.
     * @param towSupplier           A boolean supplier that toggles the tow mode.
     * @param precisionSupplier     A boolean supplier that toggles the precision mode.
     */
    public SwerveJoystickCommand(SwerveDrivetrain swerveDrive,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> towSupplier, 
            Supplier<Boolean> moveLeftSupplier, Supplier<Boolean> moveRightSupplier,
            Supplier<Boolean> precisionSupplier,
            Supplier<Integer> insideZoneId,
            Supplier<Boolean> turnToAngleSupplier,
            Supplier<Double> desiredAngleSupplier
        ) {
        this.swerveDrive = swerveDrive;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.towSupplier = towSupplier;
        this.precisionSupplier = precisionSupplier;

        this.zoneId = insideZoneId;
        
        this.turnToAngleSupplier = turnToAngleSupplier;
        this.desiredAngle = desiredAngleSupplier;

        this.moveLeft = moveLeftSupplier;
        this.moveRight = moveRightSupplier;

        this.xFilter = new OldDriverFilter2(
            ControllerConstants.kDeadband, 
            kMinimumMotorOutput,
            kTeleDriveMaxSpeedMetersPerSecond, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
        this.yFilter = new OldDriverFilter2(
            ControllerConstants.kDeadband, 
            kMinimumMotorOutput,
            kTeleDriveMaxSpeedMetersPerSecond, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
        this.turningFilter = new FilterSeries(
            new DeadbandFilter(ControllerConstants.kRotationDeadband),
            new ScaleFilter(kTeleDriveMaxAngularSpeedRadiansPerSecond)
            );
        

        this.turnToAngleController = new PIDController(
            SwerveDriveConstants.kPThetaTeleop,
            SwerveDriveConstants.kIThetaTeleop,
            SwerveDriveConstants.kDThetaTeleop
            );

        // this.turnToAngleController = new PIDController(
        //     SwerveAutoConstants.kPTurnToAngle, 
        //     SwerveAutoConstants.kITurnToAngle, 
        //     SwerveAutoConstants.kDTurnToAngle, 
        //     0.02);
        
        this.turnToAngleController.setTolerance(
            SwerveDriveConstants.kTurnToAnglePositionToleranceAngle, 
            SwerveDriveConstants.kTurnToAngleVelocityToleranceAnglesPerSec * 0.02);
    

        this.turnToAngleController.enableContinuousInput(0, 360);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        if (towSupplier.get()) {
            swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates);
            return;
        }

        if(zoneId.get() != 0)
        {
            // Check moveLeft states
            checkButtonStates(moveLeft.get(), wasLeftPressed, -1);
            wasLeftPressed = moveLeft.get(); // Update previous state

            // Check moveRight states
            checkButtonStates(moveRight.get(), wasRightPressed, 1);
            wasRightPressed = moveRight.get(); // Update previous state

            if(moveLeft.get() || moveRight.get())
            {
                return; // just move it by autopath
            }
        }

        // get speeds
        double turningSpeed;
        double xSpeed = xSpdFunction.get();
        double ySpeed = -ySpdFunction.get();

        double filteredTurningSpeed;
        double filteredXSpeed = xFilter.calculate(xSpeed);
        double filteredYSpeed = yFilter.calculate(ySpeed);

        if (turnToAngleSupplier.get()) {
            double tempAngle = desiredAngle.get();
            if ((Math.abs(tempAngle - 1000.0) > 0.01)) {
                targetAngle = tempAngle;
            } else {
                targetAngle = ((targetAngle + turningSpdFunction.get() % 360) + 360) % 360;
                // SwerveDriveConstants.kPThetaTeleop.loadPreferences();
                // SwerveDriveConstants.kIThetaTeleop.loadPreferences();
                // SwerveDriveConstants.kDThetaTeleop.loadPreferences();
                // turnToAngleController.setP(SwerveDriveConstants.kPThetaTeleop.get());
                // turnToAngleController.setI(SwerveDriveConstants.kIThetaTeleop.get());
                // turnToAngleController.setD(SwerveDriveConstants.kDThetaTeleop.get());
            }
            turningSpeed = turnToAngleController.calculate(swerveDrive.getImu().getHeading(), targetAngle);
            SmartDashboard.putNumber("Turning Speed Initial", turningSpeed);
            // turningSpeed += Math.signum(turningSpeed) * SwerveAutoConstants.kTurnToAngleFeedForwardDegreesPerSecond;
            turningSpeed = Math.toRadians(turningSpeed);
            turningSpeed = MathUtil.clamp(
                turningSpeed, 
                -SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond, 
                SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond);
            SmartDashboard.putNumber("Turning Speed", turningSpeed);
            SmartDashboard.putNumber("Target Angle", targetAngle);
            
            filteredTurningSpeed = turningSpeed;
        }
        else {
            // Manual turning
            turningSpeed = turningSpdFunction.get();
            turningSpeed *= -0.5;
            filteredTurningSpeed = turningFilter.calculate(turningSpeed);
        }


        if (precisionSupplier.get()) {
            filteredXSpeed /= 4;
            filteredYSpeed /= 4;
            // filteredTurningSpeed /= 4; // Also slows down the turn to angle speed
        }
        
        ChassisSpeeds chassisSpeeds;
        // Check if in field oriented mode
        if (!fieldOrientedFunction.get()) {
            swerveDrive.setDriveMode(DRIVE_MODE.FIELD_ORIENTED);
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                filteredXSpeed, filteredYSpeed, filteredTurningSpeed, 
                swerveDrive.getImu().getRotation2d());
        } else {
            swerveDrive.setDriveMode(DRIVE_MODE.ROBOT_ORIENTED);
            chassisSpeeds = new ChassisSpeeds(
                filteredXSpeed, filteredYSpeed, filteredTurningSpeed);
        }

        SwerveModuleState[] moduleStates;

        moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Calculate swerve module states
        swerveDrive.setModuleStates(moduleStates);
    }

    private void checkButtonStates(boolean isPressed, boolean wasPressed, int direction) {
        // Pressed: Transition from not pressed to pressed
        if (isPressed && !wasPressed) {
            swerveDrive.setAutoPathRun( zoneId.get(), direction);
        }

        // Held: Button is currently pressed
        if (isPressed) {
            // do nothing for now
        }

        // Released: Transition from pressed to not pressed
        if (!isPressed && wasPressed) {
            swerveDrive.stopAutoPath();
        }
    }
    

    public double getTargetAngle() {
        return targetAngle;
    }
}