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
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(SwerveDrivetrain swerve, String autoname, SuperSystem superSystem) throws IOException, ParseException {
        
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll), //Check if needed
            // Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            // Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            
            Commands.sequence(
                Commands.sequence(
                    superSystem.holdPiece(),
                    AutoBuilder.followPath(pathGroup.get(0)),
                    Commands.sequence(
                        superSystem.moveToAuto(NamedPositions.L4)
                    )
                ),
                Commands.sequence(
                    superSystem.outtake(),
                    Commands.waitSeconds(2.0),
                    superSystem.stopRoller()
                ),
                Commands.sequence(
                    superSystem.moveTo(NamedPositions.L5),
                    Commands.parallel(
                        AutoBuilder.followPath(pathGroup.get(1)),
                        superSystem.moveToAuto(NamedPositions.Station)
                    )
                ),
                Commands.sequence(
                    superSystem.intake(),
                    Commands.waitSeconds(2),
                    superSystem.holdPiece()
                ),
                Commands.sequence(
                        AutoBuilder.followPath(pathGroup.get(2))
                    ),
                    Commands.sequence(
                        Commands.waitSeconds(1),
                        superSystem.moveToAuto(NamedPositions.L4)
                    )
                ),
                Commands.sequence(
                    superSystem.outtake(),
                    Commands.waitSeconds(2),
                    superSystem.stopRoller()
                ),

                Commands.sequence(
                    superSystem.moveTo(NamedPositions.L5),
                    superSystem.moveTo(NamedPositions.SemiStow)
                )
                );
    }
}
