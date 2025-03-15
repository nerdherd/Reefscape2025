package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SuperSystemConstants.NamedPositions;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TwoPieceOffset extends SequentialCommandGroup {
    public TwoPieceOffset(SwerveDrivetrain swerve, String autoname, SuperSystem superSystem) throws IOException, ParseException {
        
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);

        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();
        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll), //Check if needed
            // Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose))//,
            // Commands.runOnce(() ),
            
            /*Commands.sequence(
                Commands.sequence(
                    superSystem.holdPiece(),
                    Commands.parallel(
                        AutoBuilder.followPath(pathGroup.get(0)), // Go to ready
                        superSystem.moveToAuto(NamedPositions.L4AutoPre)
                        ),
                    Commands.parallel(
                        superSystem.moveToAuto(NamedPositions.L4Auto),
                        AutoBuilder.followPath(pathGroup.get(1)) // Move to reef
                    )
                ),
                Commands.sequence(
                    Commands.waitSeconds(0.3),
                    superSystem.outtake(),
                    Commands.waitSeconds(0.3)
                    ),
                    Commands.sequence(
                        Commands.parallel(
                            superSystem.moveTo(NamedPositions.L5),
                            superSystem.stopRoller()
                        ),
                        Commands.parallel(
                            AutoBuilder.followPath(pathGroup.get(2)), // To station
                            superSystem.moveToAuto(NamedPositions.Station)
                    )
                ),
                Commands.sequence(
                    // superSystem.intake(),
                    Commands.deadline(
                        superSystem.intakeUntilSensed(),
                        Commands.waitSeconds(10)),
                    // Commands.waitSeconds(2),
                    superSystem.holdPiece()
                ),
                Commands.sequence(
                    Commands.parallel(
                        Commands.sequence(
                            Commands.waitSeconds(0.3)
                        ),
                        AutoBuilder.followPath(pathGroup.get(3))
                    ))
                //     Commands.parallel(
                //         superSystem.moveToAuto(NamedPositions.L4Auto),
                //         AutoBuilder.followPath(pathGroup.get(4))
                //     )
                // ),
                // Commands.sequence(
                //     superSystem.outtake(),
                //     Commands.waitSeconds(0.5),
                //     superSystem.stopRoller()
                // ),

                // Commands.sequence(
                //     superSystem.moveTo(NamedPositions.L5),
                //     superSystem.moveTo(NamedPositions.SemiStow)
                // )
            )*/
        );
    }
}
