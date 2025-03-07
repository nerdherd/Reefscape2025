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
import frc.robot.Constants.SuperSystemConstants.NamedPositions;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(SwerveDrivetrain swerve, String autoname, SuperSystem superSystem) throws IOException, ParseException {
        
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);

        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            
            Commands.sequence(
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(0)),
                    Commands.sequence(
                        Commands.waitSeconds(0.4),
                        superSystem.moveTo(NamedPositions.L2)
                    )
                ),
                Commands.sequence(
                    superSystem.outtake(),
                    Commands.waitSeconds(0.125),
                    superSystem.stopRoller()
                ),
                Commands.parallel(
                    superSystem.moveTo(NamedPositions.Stow),
                    AutoBuilder.followPath(pathGroup.get(1)),
                    Commands.sequence(
                        Commands.waitSeconds(1),
                        superSystem.moveTo(NamedPositions.Station)
                    )
                ),
                Commands.sequence(
                    superSystem.intake(),
                    Commands.waitSeconds(1),
                    superSystem.stopRoller()
                ),
                Commands.parallel(
                    superSystem.moveTo(NamedPositions.Stow),
                    AutoBuilder.followPath(pathGroup.get(1)),
                    Commands.sequence(
                        Commands.waitSeconds(1),
                        superSystem.moveTo(NamedPositions.L2)
                    )
                ),
                Commands.sequence(
                    superSystem.outtake(),
                    Commands.waitSeconds(0.2),
                    superSystem.stopRoller()
                )

            )
            );
    }
}
