package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWrist;

public class SetArmPosition extends Command {
    private final IntakeWrist arm;
    private final double targetPosition;

    public SetArmPosition(IntakeWrist arm, double targetPosition) {
        this.arm = arm;
        this.targetPosition = targetPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmPosition(targetPosition);
    }

    @Override
    public void execute() {
        //arm.runPID();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return arm.atSetpoint();
    }
}