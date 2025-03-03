package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import frc.robot.subsystems.IntakeV2;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class PathOnlyBottom2Piece extends SequentialCommandGroup {
    // private IntakeRoller intakeRoller;
    // private Elevator elevator;
    private List<PathPlannerPath> pathGroup;
    private Pose2d startingPose;

    public PathOnlyBottom2Piece(SwerveDrivetrain swerve, String autoPath) 
    throws IOException, ParseException {
        // this.intakeRoller = intakeRoller;
        // this.elevator = elevator;

        this.pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        this.startingPose = pathGroup.get(0).getStartingDifferentialPose();
        
        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            runAuto()
        );
    }

    public Command runAuto() {
        return Commands.sequence(
            AutoBuilder.followPath(pathGroup.get(0)),
            // elevator.moveToReefL3(),
            // intakeRoller.outtake(),
            Commands.waitSeconds(1.5),
            // intakeRoller.stop(),

            Commands.parallel(
                // elevator.stow(),
                Commands.none(),
                AutoBuilder.followPath(pathGroup.get(1))
            ),
            // elevator.moveToStation(),
            // intakeRoller.intake(),
            Commands.waitSeconds(2.5),
            // intakeRoller.stop(),

            Commands.parallel(
                // elevator.stow(),
                Commands.none(),
                AutoBuilder.followPath(pathGroup.get(2))
            ),
            // elevator.moveToReefL3(),
            // intakeRoller.outtake(),
            Commands.waitSeconds(1.5)
            // ,
            // intakeRoller.stop(),

            // elevator.stow()
        );
    }

    public Command stopAuto() {
        return Commands.sequence(
            Commands.none()
            // intakeRoller.stop(),
            // elevator.stow()
        );
    }
}
