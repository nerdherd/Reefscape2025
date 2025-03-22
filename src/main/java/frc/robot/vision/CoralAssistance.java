package frc.robot.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class CoralAssistance {

    public double tx;
    public double ty;
    public double ta;
    public String name;

    private PIDController areaController;
    private PIDController txController;
    private PIDController rotationController;

    public CoralAssistance(String name) {
        this.name = name;

        tx = LimelightHelpers.getTX(name);  // Horizontal offset from crosshair to target in degrees
        ty = LimelightHelpers.getTY(name);  // Vertical offset from crosshair to target in degrees
        ta = LimelightHelpers.getTA(name);  // Target area (0% to 100% of image)

        // IMPORTANT: These values were copied from the old code, so they are probably wrong
        areaController = new PIDController(0.18, 0, 0.004);     // TODO: tune
        txController = new PIDController(0.05, 0, 0.001);       // TODO: tune
        rotationController = new PIDController(0.08, 0, 0.006); // TODO: tune
    }

    public void driveToCoral(SwerveDrivetrain swerveDrivetrain,double targetArea){
        if (LimelightHelpers.getTV(name)){
            tx = LimelightHelpers.getTX(name);  // Horizontal offset from crosshair to target in degrees
            ty = LimelightHelpers.getTY(name);  // Vertical offset from crosshair to target in degrees
            ta = LimelightHelpers.getTA(name);  // Target area (0% to 100% of image)

            double forwardSpeed = areaController.calculate(ta,targetArea);
            double sidewaySpeed = txController.calculate(tx,0);

            swerveDrivetrain.drive(forwardSpeed,sidewaySpeed);
        }
    }

    public Command driveToCoralCommand(SwerveDrivetrain swerveDrivetrain, double targetArea) {
        return Commands.runOnce(() -> driveToCoral(swerveDrivetrain, targetArea));
    }


}