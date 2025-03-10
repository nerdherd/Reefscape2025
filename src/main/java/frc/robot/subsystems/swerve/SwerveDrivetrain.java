package frc.robot.subsystems.swerve;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SwerveDriveConstants.CANCoderConstants;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.util.NerdyLine;
import frc.robot.util.NerdyMath;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.VisionSys;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Reportable;

import static frc.robot.Constants.SwerveDriveConstants.*;
import static frc.robot.Constants.PathPlannerConstants.kPPRotationPIDConstants;
import static frc.robot.Constants.PathPlannerConstants.kPPTranslationPIDConstants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;

public class SwerveDrivetrain extends SubsystemBase implements Reportable {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final Gyro gyro;
    // private final SwerveDriveOdometry odometer;
    private boolean isTest = false;
    private final SwerveDrivePoseEstimator poseEstimator;
    private DRIVE_MODE driveMode = DRIVE_MODE.FIELD_ORIENTED;

    //Vision
    private int counter = 0;
    private int visionFrequency = 1;
    private AprilTagFieldLayout layout;
    private double lastDistance;
    private double[] distances = new double[] {0, 1, 1.72, 2, 3, 3.5, 4, 5};
    private double[] tolerances = new double[] {12, 11, 10, 9, 5, 4, 2, 2};
    private NerdyLine toleranceSpline = new NerdyLine(distances, tolerances);
    private double[] angles          = new double[] {0, 10, 30, 45, 90};
    private double[] toleranceScales = new double[] {1, 0.95, 0.75, 0.4, 0};
    private NerdyLine angleToleranceSpline = new NerdyLine(angles, toleranceScales);
    
    private Field2d field;
    private VisionSys vision = new VisionSys();

    public enum DRIVE_MODE {
        FIELD_ORIENTED, // always use it
        ROBOT_ORIENTED, // most likely it's for testing
        // AUTONOMOUS not used
    }

    /**
     * Construct a new {@link SwerveDrivetrain}
     */
    public SwerveDrivetrain(Gyro gyro) throws IllegalArgumentException {
        
        // LimelightHelpers.setPipelineIndex(VisionConstants.kLimelightBackLeftName, 1);
        LimelightHelpers.setPipelineIndex(VisionConstants.kLimelightBackRightName, 1);
        LimelightHelpers.setPipelineIndex(VisionConstants.kLimelightFrontLeftName, 1);
        LimelightHelpers.setPipelineIndex(VisionConstants.kLimelightFrontRightName, 1);
        
        frontLeft = new SwerveModule(
            kFLDriveID,
            kFLTurningID,
            kFLDriveReversed,
            kFLTurningReversed,
            CANCoderConstants.kFLCANCoderID,
            CANCoderConstants.kFLCANCoderReversed);
        frontRight = new SwerveModule(
            kFRDriveID,
            kFRTurningID,
            kFRDriveReversed,
            kFRTurningReversed,
            CANCoderConstants.kFRCANCoderID,
            CANCoderConstants.kFRCANCoderReversed);
        backLeft = new SwerveModule(
            kBLDriveID,
            kBLTurningID,
            kBLDriveReversed,
            kBLTurningReversed,
            CANCoderConstants.kBLCANCoderID,
            CANCoderConstants.kBLCANCoderReversed);
        backRight = new SwerveModule(
            kBRDriveID,
            kBRTurningID,
            kBRDriveReversed,
            kBRTurningReversed,
            CANCoderConstants.kBRCANCoderID,
            CANCoderConstants.kBRCANCoderReversed);

        this.gyro = gyro;

        /** @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
         *     in meters, and heading in radians). Increase these numbers to trust your state estimate
         *     less.
         * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
         *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
         *     the vision pose measurement less.
        */
        this.poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // TODO: Set pose estimator weights
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); 
        

        //Vision
        layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);      

        field = new Field2d();
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        //DCMotor dcMotor = new DCMotor(kDriveOneMinusAlpha, kDriveAlpha, kBRTurningID, kBRDriveID, kBLTurningID, kBLDriveID);
        //ModuleConfig moduleConfig = new ModuleConfig(kBRTurningID, kBRDriveID, kWheelBase, dcMotor, kBLTurningID, kBLDriveID);
        RobotConfig robotConfig = null;
        try {
             robotConfig = RobotConfig.fromGUISettings();
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> setChassisSpeeds(speeds),
            new PPHolonomicDriveController(
                kPPTranslationPIDConstants, 
                kPPRotationPIDConstants), 
                // kPPMaxVelocity,
                // kTrackWidth,
                // new ReplanningConfig()), 
            robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    boolean initPoseByVisionDone = false;

    /**
     * Have modules move towards states and update odometry
     */
    @Override
    public void periodic() {
	
	if (!isTest) {
            runModules();
        }
        
        poseEstimator.update(gyro.getRotation2d(), getModulePositions());

        field.setRobotPose(poseEstimator.getEstimatedPosition());
            
            double robotRotation = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    
            SmartDashboard.putNumber("Robot Rotation", robotRotation);
    
            visionupdateOdometry(VisionConstants.kLimelightBackLeftName); // TODO Do we need to pass in robotRotation?
            visionupdateOdometry(VisionConstants.kLimelightBackRightName);
            visionupdateOdometry(VisionConstants.kLimelightFrontLeftName);
            visionupdateOdometry(VisionConstants.kLimelightFrontRightName);

    }

    //******************************  Vision ******************************/
	private void visionupdateOdometry(String limelightName) {
        boolean doRejectUpdate = false;

        LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName); //TODO: test if we need to account for alliance
        double xyStds = 0.5; //Tune 
        double degStds = 999999; //

        boolean receivedValidData = LimelightHelpers.getTV(limelightName);
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName); //MegatTag1
        
        if(estimate == null) {
            return;
        }
        Pose2d botPose1 = estimate.pose;
        
        if(!receivedValidData)
            doRejectUpdate = true;
        // else if(botPose1.getZ() > 0.3 || botPose1.getZ() < -0.3)
        //     doRejectUpdate = true;
        else if(megaTag2.tagCount == 1 && megaTag2.rawFiducials.length == 1)
        {
            if(megaTag2.rawFiducials[0].ambiguity > .7)
            {
                doRejectUpdate = true;
            }
            if(megaTag2.rawFiducials[0].distToCamera > 3)
            {
                doRejectUpdate = true;
            }
    
            SmartDashboard.putNumber(limelightName + " X Position", botPose1.getX());
            SmartDashboard.putNumber(limelightName + " Y Position", botPose1.getY());
            
            // 1 target with large area and close to estimated pose
            if (megaTag2.avgTagArea > 0.8 && megaTag2.rawFiducials[0].distToCamera < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (megaTag2.avgTagArea > 0.1 && megaTag2.rawFiducials[0].distToCamera < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }
        }
        else if (megaTag2.tagCount >= 2) {
            xyStds = 0.5;
            degStds = 6;
        }

        if(!doRejectUpdate)
        {
            poseEstimator.setVisionMeasurementStdDevs(
              VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

            //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            poseEstimator.addVisionMeasurement(
                megaTag2.pose,
                megaTag2.timestampSeconds);
        }
    }
	

    private void visionupdateOdometry(String limelightName,double robotRotation) {

        // Needs to be tested on field to see if accurate, because it uses the robots rotation
        // to help figure out where it is

        LimelightHelpers.SetRobotOrientation(limelightName, robotRotation, 0, 0, 0, 0, 0);

        String log = limelightName;

        // LimelightHelpers.SetRobotOrientation("limelightName")
        boolean doRejectUpdate = false;

        LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName); //TODO: test if we need to account for alliance
        double xyStds = 0.5; //Tune 
        double degStds = 999999; //

        boolean receivedValidData = LimelightHelpers.getTV(limelightName);
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (estimate == null){
            doRejectUpdate = true;
        }
        
        if(!receivedValidData)
            doRejectUpdate = true;
        // else if(botPose1.getZ() > 0.3 || botPose1.getZ() < -0.3)
        //     doRejectUpdate = true;
        // else if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        // {
        //     doRejectUpdate = true;
        // }
        else if(megaTag2.tagCount == 0)
        {
            doRejectUpdate = true;
        }

        // SmartDashboard.putBoolean(limelightName+" Valid Data", !doRejectUpdate);
        log+="Valid Data: "+ Boolean.toString(!doRejectUpdate)+"\n";
        log+="Is Null: "+Boolean.toString(estimate == null)+"\n";
        log+="Megatag Count"+Integer.toString(megaTag2.tagCount)+"\n";
        // SmartDashboard.putBoolean(limelightName+" Is Null", estimate == null);
        // SmartDashboard.putNumber(limelightName+" Megatag Count", megaTag2.tagCount);

        SmartDashboard.putString(limelightName + "Info", log);

        if(!doRejectUpdate)
        {
            Pose2d botPose1 = estimate.pose;

            SmartDashboard.putNumber(limelightName + " X Position", botPose1.getX());
            SmartDashboard.putNumber(limelightName + " Y Position", botPose1.getY());
            SmartDashboard.putNumber(limelightName + " Rotation"  , botPose1.getRotation().getDegrees());
;
            if (megaTag2.tagCount >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (megaTag2.avgTagArea > 0.8 && megaTag2.rawFiducials[0].distToCamera < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (megaTag2.avgTagArea > 0.1 && megaTag2.rawFiducials[0].distToCamera < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }

            poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

            //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            poseEstimator.addVisionMeasurement(
                megaTag2.pose,
                megaTag2.timestampSeconds);
        }
    }
    
    //****************************** RESETTERS ******************************/

    /**
     * Resets the odometry to given pose 
     * @param pose  A Pose2D representing the pose of the robot
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public void resetOdometryWithAlliance(Pose2d pose){
        if (RobotContainer.IsRedSide()) {
            resetOdometry(FlippingUtil.flipFieldPose(pose));
        } else {
            resetOdometry(pose);
        }
    }

    public void zeroGyroAndPoseAngle() {
        gyro.zeroHeading();
        gyro.setOffset(0);
        Pose2d pose = getPose();
        Pose2d newPose = new Pose2d(pose.getX(), pose.getY(), RobotContainer.IsRedSide() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), newPose);
    }

    public void resetGyroFromPoseWithAlliance(Pose2d pose) {
        if (RobotContainer.IsRedSide()) {
            double angle = FlippingUtil.flipFieldPose(pose).getRotation().getDegrees() - 180;
            angle = NerdyMath.posMod(angle, 360);
            gyro.resetHeading(angle);
        } else {
            gyro.resetHeading(NerdyMath.posMod(pose.getRotation().getDegrees(), 360));
        }
    }

    public void refreshModulePID() {
        frontLeft.refreshPID();
        backLeft.refreshPID();
        frontRight.refreshPID();  
        backRight.refreshPID();
    }

    /**
     * Stops all modules. See {@link SwerveModule#stop()} for more info.
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Have modules move to their desired states. See {@link SwerveModule#run()} for more info.
     */
    public void runModules() {
        frontLeft.run();
        frontRight.run();
        backLeft.run();
        backRight.run();
    }

    
            
    private int getMostClosedApriltagIdInZone(int zoneId) {
        return 19; // todo add cameras' reading 
    }

    private Map<Integer, ArrayList<Pose2d>> myMap = new HashMap<>();

    private Pose2d calcuTargetPoseByReq(int zoneId, int poseId)
    {
        if(myMap.isEmpty()) // todo move it to constructor
        {
            ArrayList<Pose2d> list17 = new ArrayList<>();
            list17.add(new Pose2d(3.204,2.591,new Rotation2d(-120*3.1416/180.0))); // must be left
            list17.add(new Pose2d(3.878,2.187,new Rotation2d(-120*3.1416/180.0))); // must be right
            myMap.put(17, list17);

            ArrayList<Pose2d> list18 = new ArrayList<>();
            list18.add(new Pose2d(2.646,4.391,new Rotation2d(180*3.1416/180.0))); // must be left
            list18.add(new Pose2d(2.646,3.707,new Rotation2d(180*3.1416/180.0))); // must be right
            myMap.put(18, list18);

            ArrayList<Pose2d> list19 = new ArrayList<>();
            list19.add(new Pose2d(3.897,5.805,new Rotation2d(2.094))); // must be left
            list19.add(new Pose2d(3.262,5.555,new Rotation2d(2.094))); // must be right
            myMap.put(19, list19);

            ArrayList<Pose2d> list20 = new ArrayList<>();
            list20.add(new Pose2d(5.831,5.401,new Rotation2d(60*3.1416/180.0)));
            list20.add(new Pose2d(5.109,5.776,new Rotation2d(60*3.1416/180.0)));
            myMap.put(20, list20);

            ArrayList<Pose2d> list21 = new ArrayList<>();
            list21.add(new Pose2d(6.321,3.535,new Rotation2d(0)));
            list21.add(new Pose2d(6.321,4.343,new Rotation2d(0)));
            myMap.put(21, list21);

            ArrayList<Pose2d> list22 = new ArrayList<>();
            list22.add(new Pose2d(5.008,2.2,new Rotation2d(154.638*3.1416/180.0)));
            list22.add(new Pose2d(5.85,2.649,new Rotation2d(154.638*3.1416/180.0)));
            myMap.put(22, list22);

            ArrayList<Pose2d> list6 = new ArrayList<>();
            list6.add(new Pose2d(13.672,2.187,new Rotation2d(-60*3.1416/180.0))); // must be left
            list6.add(new Pose2d(14.307,2.591,new Rotation2d(-60*3.1416/180.0))); // must be right
            myMap.put(6, list6);

            ArrayList<Pose2d> list7 = new ArrayList<>();
            list7.add(new Pose2d(14.914,3.602,new Rotation2d(0))); // must be left
            list7.add(new Pose2d(14.914,4.333,new Rotation2d(0))); // must be right
            myMap.put(7, list7);

            ArrayList<Pose2d> list8 = new ArrayList<>();
            list8.add(new Pose2d(14.375,5.411,new Rotation2d(60*3.1416/180.0))); // must be left
            list8.add(new Pose2d(13.672,5.5776,new Rotation2d(60*3.1416/180.0))); // must be right
            myMap.put(8, list8);

            ArrayList<Pose2d> list9 = new ArrayList<>();
            list9.add(new Pose2d(12.460,5.805,new Rotation2d(120*3.1416/180.0)));
            list9.add(new Pose2d(11.835,5.487,new Rotation2d(120*3.1416/180.0)));
            myMap.put(9, list9);

            ArrayList<Pose2d> list10 = new ArrayList<>();
            list10.add(new Pose2d(11.219,4.400,new Rotation2d(180*3.1416/180.0)));
            list10.add(new Pose2d(11.219,3.717,new Rotation2d(180*3.1416/180.0)));
            myMap.put(10, list10);

            ArrayList<Pose2d> list11 = new ArrayList<>();
            list11.add(new Pose2d(11.748,2.639,new Rotation2d(-120*3.1416/180.0)));
            list11.add(new Pose2d(12.383,2.264,new Rotation2d(-120*3.1416/180.0)));
            myMap.put(11, list11);
        }

        Pose2d targetPose = poseEstimator.getEstimatedPosition();
        if(zoneId == 1) // own reef
        {
            // obtain the closed apriltag id from two low-back cameras.
            int targetApriltagId = getMostClosedApriltagIdInZone(zoneId); 
            if(myMap.containsKey(targetApriltagId))
            {
                if(poseId == -1)
                {
                    return myMap.get(targetApriltagId).get(0); // the left side of one apriltag on reef
                }
                else if(poseId == 1)
                {
                    return myMap.get(targetApriltagId).get(1);// the right side of one apriltag on reef
                }
                else 
                {

                }
            }
        }
        return targetPose;
    }
            
    double maxVelocityMps = 1;
    double maxAccelerationMpsSq = 1;
    private Command pathfindingCommand; // Store the command reference
    public void setAutoPathRun(int zoneId, int poseId)
    {
        Pose2d destPoseInBlue = calcuTargetPoseByReq(zoneId, poseId); // base on (poseid and zoneid and apriltag id)
        PathConstraints pathcons = new PathConstraints(
            maxVelocityMps, maxAccelerationMpsSq, 
            Units.degreesToRadians(360), Units.degreesToRadians(720)
        );
        if(RobotContainer.IsRedSide())
            pathfindingCommand = AutoBuilder.pathfindToPose(FlippingUtil.flipFieldPose(destPoseInBlue), pathcons);
        else
            pathfindingCommand = AutoBuilder.pathfindToPose(destPoseInBlue, pathcons);;
        
        pathfindingCommand.schedule();
    }

    public void stopAutoPath() {
        if (pathfindingCommand != null && !pathfindingCommand.isFinished()) {
            pathfindingCommand.cancel();
        }
    }

    //****************************** GETTERS ******************************/

    public Gyro getImu() {
        return this.gyro;
    }

    /**
     * Gets a pose2d representing the position of the drivetrain
     * @return A pose2d representing the position of the drivetrain
     */
    public Pose2d getPose() {
        // return odometer.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getTagPose2D(int tagID)
    {
        return getTagPose3D(tagID).toPose2d();
    }

    public Pose3d getTagPose3D(int tagID)
    {
        Optional<Pose3d> tagPose = layout.getTagPose(tagID);
        if(tagPose.isEmpty()) return null;
        return tagPose.get();
    }

    public double getDistanceFromTag(boolean preserveOldValue, int tagID)
    {
        Pose2d tagPose = getTagPose2D(tagID);
        if(tagPose == null) return (preserveOldValue ? lastDistance : 0.01);

        Pose2d robotPose = getPose();
        lastDistance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
        lastDistance = Math.sqrt(Math.pow(robotPose.getX()-tagPose.getX(), 2) + Math.pow(robotPose.getY()-tagPose.getY(), 2));

        return lastDistance;
    }
    public double getSpeakerTurnToAngleTolerance ()
    {
        double distance = getDistanceFromTag(true, RobotContainer.IsRedSide() ? 4 : 7);
        if(distance > 5) {
            return 0;
        }
        if(distance > 4) {
            return 1;
        }
        double tolerance = toleranceSpline.getOutput(distance);
        if(tolerance < 0) {
            return 0;
        }
        return tolerance;
    }

    public double getTurnToSpecificTagAngle(int tagID)
    {
        Pose2d tagPose = getTagPose2D(tagID);
        Pose2d robotPose = getPose();
        double xOffset = tagPose.getX() - robotPose.getX();
        double yOffset = tagPose.getY() - robotPose.getY();

        double allianceOffset = 90;
        double angle = NerdyMath.posMod(-Math.toDegrees(Math.atan2(xOffset, yOffset)) + allianceOffset, 360);
        if(RobotContainer.IsRedSide()) {
            return angle; //TODO: test if works since this is a bit different than original code
        }
        return (180 + angle) % 360;
    }

    public double getTurnToAngleToleranceScale(double targetAngle)
    {
        double angleToSpeaker = 10000;
        targetAngle = NerdyMath.posMod(targetAngle, 360);
        if (targetAngle > 180) {
            angleToSpeaker = Math.abs(360 - targetAngle);
        }
        else if (targetAngle < 180) {
            angleToSpeaker = targetAngle;
        }
        return angleToleranceSpline.getOutput(angleToSpeaker);
    }

    public boolean turnToAngleMode = true;

    public Command toggleTurnToAngleMode() {
      return Commands.runOnce(() -> turnToAngleMode = !turnToAngleMode);
    }

    public boolean getTurnToAngleMode() {
        return turnToAngleMode;
    }

    /**
     * Get the position of each swerve module
     * @return An array of swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(), 
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    private ChassisSpeeds getChassisSpeeds() {
        return SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public int getCurrentZoneByPose()
    {
        Pose2d cPose2d = poseEstimator.getEstimatedPosition();
        double xp = cPose2d.getX();
        double yp = cPose2d.getY();

        //1 is in reef zone, 2 is in left station, 3 is in right station, 4 is in proc zone
        if(RobotContainer.IsRedSide())
        {
            if(NerdyMath.isPoseInsideCircleZone(13, 4, 9, xp, yp))
            {
                return 1;
            }
            // todo zone 2,3,4
        }
        else
        {
            if(NerdyMath.isPoseInsideCircleZone(4.5, 4, 9, xp, yp))
            //if( xp > 1.1 && xp < 6.8 && yp > 1.5 && yp < 7.5) // a box area around reef
            {
                return 1;
            }
            // todo zone 2,3,4
        }
        return 0; // 0 is default for disable.  
    }

    public void drive(double xSpeed, double ySpeed, double turnSpeed) {
        setModuleStates(
            SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, turnSpeed)
            )
        );
    }

    public void drive(double xSpeed, double ySpeed) {
        drive(xSpeed, ySpeed, 0);
    }

    public void driveFieldOriented(double xSpeed, double ySpeed, double turnSpeed) {
        setModuleStates(
            SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, gyro.getRotation2d())
            )
        );
    }

    public void driveFieldOriented(double xSpeed, double ySpeed) {
        driveFieldOriented(xSpeed, ySpeed, 0);
    }

    public Command driveToPose(SwerveDriveConstants.FieldPositions fieldPos, double maxVelocityMps, double maxAccelerationMpsSq) {
        return driveToPose(fieldPos.pos, maxVelocityMps, maxAccelerationMpsSq);
    }

    public Command driveToPose(Pose2d destPoseInBlue, double maxVelocityMps, double maxAccelerationMpsSq) {
        PathConstraints pathcons = new PathConstraints(
            maxVelocityMps, maxAccelerationMpsSq, 
            Units.degreesToRadians(180), Units.degreesToRadians(360)
        );
        return Commands.either(
            AutoBuilder.pathfindToPose(FlippingUtil.flipFieldPose(destPoseInBlue), pathcons),
            AutoBuilder.pathfindToPose(destPoseInBlue, pathcons),
            RobotContainer::IsRedSide  
        );
    }

    //Equation used found by Zachary Martinez
    //https://www.desmos.com/calculator/q70q2ekunm

    public Command moveLeftOf(int tagID) {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        Rotation2d tagRotation = layout.getTagPose(tagID).get().toPose2d().getRotation();
        Rotation2d tagRotationInverse = new Rotation2d(-tagRotation.getRadians());
        Double theta_0 = tagRotationInverse.getRadians();
        Double moveBy = 0.5; //TODO: Change Later
        // D_x = R_x + M cos (theta_0)
        // D_y = R_y + M sin (theta_0)
        Transform2d transformer = new Transform2d((moveBy * Math.cos(theta_0)), (moveBy * Math.sin(theta_0)), tagRotation);
        // return driveToRelativePose(PathPlannerConstants.kPPMaxVelocity, PathPlannerConstants.kPPMaxAcceleration, transformer);
        return driveToRelativePose(1,1, transformer); //only for testing
    }

    public Command moveRightOf(int tagID) {
        Rotation2d tagRotation = layout.getTagPose(tagID).get().toPose2d().getRotation();
        Rotation2d tagRotationInverse = new Rotation2d(-tagRotation.getRadians());
        Double theta_0 = tagRotationInverse.getRadians();
        Double moveBy = -0.5; //TODO: Change Later
        // D_x = R_x + M cos (theta_0)
        // D_y = R_y + M sin (theta_0)
        Transform2d transformer = new Transform2d((moveBy * Math.cos(theta_0)), (moveBy * Math.sin(theta_0)), tagRotation);
        // return driveToRelativePose(PathPlannerConstants.kPPMaxVelocity, PathPlannerConstants.kPPMaxAcceleration, transformer);
        return driveToRelativePose(1,1, transformer); //only for testing

    }

    public Command driveToRelativePose(double maxVelocityMps, double maxAccelerationMpsSq, Transform2d translation) {
        Pose2d targetPose = getPose().plus(translation);
        return driveToPose(targetPose, maxVelocityMps, maxAccelerationMpsSq);
    }

     public int getReefTagID(String limelightName) {
        long id = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tid").getInteger(-1);
        return (int) id;
    }
    
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] targetStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(targetStates);
    }

    public void setChassisSpeedsBi(ChassisSpeeds speeds, DriveFeedforwards drives) {
        SwerveModuleState[] targetStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(targetStates);
    }

    //****************************** SETTERS ******************************/

    /**
     * Set the drive mode (only for telemetry purposes)
     * @param driveMode
     */
    public void setDriveMode(DRIVE_MODE driveMode) {
        this.driveMode = driveMode;
    }

    public void setVelocityControl(boolean withVelocityControl) {
        frontLeft.toggleVelocityControl(withVelocityControl);
        frontRight.toggleVelocityControl(withVelocityControl);
        backLeft.toggleVelocityControl(withVelocityControl);
        backRight.toggleVelocityControl(withVelocityControl);
    }

    /**
     * Set the neutral modes of all modules.
     * <p>
     * true sets break mode, false sets coast mode
     * 
     * @param breaking  Whether or not the modules should be in break
     */
    public void setBreak(boolean breaking) {
        frontLeft.setBreak(breaking);
        frontRight.setBreak(breaking);
        backLeft.setBreak(breaking);
        backRight.setBreak(breaking);
    }

    /**
     * Sets module desired states
     * @param desiredStates desired states of the four modules (FL, FR, BL, BR)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void towModules() {
        frontLeft.setDesiredState(towModuleStates[0], false);
        frontRight.setDesiredState(towModuleStates[1], false);
        backLeft.setDesiredState(towModuleStates[2], false);
        backRight.setDesiredState(towModuleStates[3], false);
    }

    public Command towCommand() {
        return Commands.runOnce(this::towModules, this);
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab;
        if (level == LOG_LEVEL.MINIMAL) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Swerve");
        }

        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.add("Field Position", field).withSize(6, 3);
                // tab.addString(("Current Command"), () -> {
                //     Command currCommand = this.getCurrentCommand();
                //     if (currCommand == null) {
                //         return "null";
                //     } else {
                //         return currCommand.getName();
                //     }
                // }
                // );
                tab.add("Toggle Test", Commands.runOnce(() -> isTest = !isTest));
                tab.addBoolean("Test Mode", () -> isTest);
                // Might be negative because our swerveDriveKinematics is flipped across the Y axis
            case MEDIUM:
            case MINIMAL:
                tab.addNumber("X Position (m)", () -> poseEstimator.getEstimatedPosition().getX());
                tab.addNumber("Y Position (m)", () -> poseEstimator.getEstimatedPosition().getY());
                tab.addNumber("Odometry Angle", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees());
                tab.addString("Drive Mode", () -> this.driveMode.toString());
                break;
        }
    }

    public void initModuleShuffleboard(LOG_LEVEL level) {
        frontRight.initShuffleboard(level);
        frontLeft.initShuffleboard(level);
        backLeft.initShuffleboard(level);
        backRight.initShuffleboard(level);
    }

    /**
     * Report values to smartdashboard.
     */
     public void reportToSmartDashboard(LOG_LEVEL level) {
    //     switch (level) {
    //         case OFF:
    //             break;
    //         case ALL:
    //         case MEDIUM:
    //             SmartDashboard.putData("Zero Modules", Commands.runOnce(this::zeroModules));
    //         case MINIMAL:
    //             SmartDashboard.putNumber("Odometer X Meters", poseEstimator.getEstimatedPosition().getX());
    //             SmartDashboard.putNumber("Odometer Y Meters", poseEstimator.getEstimatedPosition().getY());
    //             SmartDashboard.putString("Drive Mode", this.driveMode.toString());
    //             break;
    //     }
     }

    // public void reportModulesToSmartDashboard(LOG_LEVEL level) {
    //     frontRight.reportToSmartDashboard(level);
    //     frontLeft.reportToSmartDashboard(level);
    //     backLeft.reportToSmartDashboard(level);
    //     backRight.reportToSmartDashboard(level);
    // }
}