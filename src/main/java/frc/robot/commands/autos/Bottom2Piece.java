package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class Bottom2Piece extends SequentialCommandGroup {
    private static AlgaeRoller intakeRoller;
    private static Elevator elevator;
    private static List<PathPlannerPath> pathGroup;
    private static Pose2d startingPose;

    public Bottom2Piece(SwerveDrivetrain swerve, AlgaeRoller intakeRoller, Elevator elevator, String autoPath) 
    throws IOException, ParseException {
        Bottom2Piece.intakeRoller = intakeRoller;
        Bottom2Piece.elevator = elevator;

        Bottom2Piece.pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        Bottom2Piece.startingPose = pathGroup.get(0).getStartingDifferentialPose();
        
        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(swerve.getImu()::zeroAll),
            runAuto()
        );
    }

    public Command runAuto() {
        return Commands.sequence(
            AutoBuilder.followPath(pathGroup.get(0)),
            elevator.moveToReefL4(),
            intakeRoller.outtake(),
            Commands.waitSeconds(1.5),
            intakeRoller.stop(),

            Commands.parallel(
                elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(1))
            ),
            elevator.moveToStation(),
            intakeRoller.intake(),
            Commands.waitSeconds(2.5),
            intakeRoller.stop(),

            Commands.parallel(
                elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(2))
            ),
            elevator.moveToReefL3(),
            intakeRoller.outtake(),
            Commands.waitSeconds(1.5),
            intakeRoller.stop(),

            elevator.stow()
        );
    }

    public Command stopAuto() {
        return Commands.sequence(
            intakeRoller.stop(),
            elevator.stow()
        );
    }
}
