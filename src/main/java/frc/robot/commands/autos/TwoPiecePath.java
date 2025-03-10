package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TwoPiecePath extends SequentialCommandGroup {
    public TwoPiecePath(SwerveDrivetrain swerve, String autoname, SuperSystem superSystem) throws IOException, ParseException {
        
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);

        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();

        addCommands(
            // Commands.runOnce(swerve.getImu()::zeroAll), //Check if needed
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            

            Commands.sequence(
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(0))
                    // Commands.sequence(
                    //     Commands.waitSeconds(0.4),
                    //     superSystem.moveToAuto(NamedPositions.L2)
                    // )
                ),
                // Commands.sequence(
                //     superSystem.outtake(),
                //     Commands.waitSeconds(0.2)
                // ),
                Commands.parallel(
                    // superSystem.moveTo(NamedPositions.SemiStow),
                    AutoBuilder.followPath(pathGroup.get(1))
                    // Commands.sequence(
                    //     Commands.waitSeconds(1),
                    //     superSystem.moveToAuto(NamedPositions.Station)
                    // )
                ),
                // Commands.sequence(
                //     superSystem.intakeUntilSensed(),
                //     superSystem.holdPiece()
                // ),
                Commands.parallel(
                    // superSystem.moveTo(NamedPositions.SemiStow),
                    AutoBuilder.followPath(pathGroup.get(2))
                    // Commands.sequence(
                    //     Commands.waitSeconds(1),
                    //     superSystem.moveToAuto(NamedPositions.L2)
                    // )
                )
                // Commands.sequence(
                //     superSystem.outtake(),
                //     Commands.waitSeconds(0.2),
                //     superSystem.stopRoller()
                // )

            )
            );
    }
}
