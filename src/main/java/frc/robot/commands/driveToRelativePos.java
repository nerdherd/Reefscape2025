// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.CryptoPrimitive;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveToRelativePos extends Command {
  private final Transform2d relativePos;
  private final TriFunction<Transform2d, Double, Double, Command> driveToCommandSupplier;
  private final double maxV, maxA;
  private Command currentDriveTo = Commands.none();
  /** Creates a new driveToRelativePos. */
  public driveToRelativePos(TriFunction<Transform2d, Double, Double, Command> driveToCommandSupplier, Transform2d relativePos, double maxV, double maxA) {
    this.relativePos = relativePos;
    this.driveToCommandSupplier = driveToCommandSupplier;
    this.maxV = maxV;
    this.maxA = maxA;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentDriveTo = driveToCommandSupplier.apply(relativePos, maxV, maxA);
    currentDriveTo.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDriveTo.execute();
  }

  @Override
  public void end(boolean interrupted) {
    currentDriveTo.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return currentDriveTo.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return currentDriveTo.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    if (currentDriveTo.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
      return InterruptionBehavior.kCancelSelf;
    } else {
      return InterruptionBehavior.kCancelIncoming;
    }
  }
}
