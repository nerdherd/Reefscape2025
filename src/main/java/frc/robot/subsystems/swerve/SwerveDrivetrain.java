package frc.robot.subsystems.swerve;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SwerveDriveConstants.CANCoderConstants;
import frc.robot.Constants.SwerveDriveConstants.ReefOffsets;
import frc.robot.Constants.SwerveDriveConstants.StationOffsets;
import frc.robot.subsystems.imu.Gyro;
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

import com.pathplanner.lib.auto.AutoBuilder;
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
    
    ArrayList<Pose2d> list1 = new ArrayList<>();
    ArrayList<Pose2d> list2 = new ArrayList<>();
    ArrayList<Pose2d> list3 = new ArrayList<>();
    ArrayList<Pose2d> list4 = new ArrayList<>();
    ArrayList<Pose2d> list6 = new ArrayList<>();
    ArrayList<Pose2d> list7 = new ArrayList<>();
    ArrayList<Pose2d> list8 = new ArrayList<>();
    ArrayList<Pose2d> list9 = new ArrayList<>();
    ArrayList<Pose2d> list10 = new ArrayList<>();
    ArrayList<Pose2d> list11 = new ArrayList<>();
    ArrayList<Pose2d> list12 = new ArrayList<>();
    ArrayList<Pose2d> list13 = new ArrayList<>();
    ArrayList<Pose2d> list15 = new ArrayList<>();
    ArrayList<Pose2d> list16 = new ArrayList<>();
    ArrayList<Pose2d> list17 = new ArrayList<>();
    ArrayList<Pose2d> list18 = new ArrayList<>();
    ArrayList<Pose2d> list19 = new ArrayList<>();
    ArrayList<Pose2d> list20 = new ArrayList<>();
    ArrayList<Pose2d> list21 = new ArrayList<>();
    ArrayList<Pose2d> list22 = new ArrayList<>();

    private Field2d field;
    private VisionSys vision = new VisionSys();
    public boolean useVision = true;

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
        initReefSidePoses();
        initCagePoses();
        initProcesPoses();
        initStationsPoses();

        
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

    private void initCagePoses() {
        list4.add(SwerveDriveConstants.MapPoses.tag4LeftBlueOnRed);
        list4.add(SwerveDriveConstants.MapPoses.tag4RightBlueOnRed); // they are for blue side
        list4.add(SwerveDriveConstants.MapPoses.tag4MidBlueOnRed);
        myCageMap.put(4, list4);

        list15.add(SwerveDriveConstants.MapPoses.tag15LeftRedOnBlue); // they are for red side
        list15.add(SwerveDriveConstants.MapPoses.tag15RightRedOnBlue);
        list15.add(SwerveDriveConstants.MapPoses.tag15MidRedOnBlue);
        myCageMap.put(15, list15);
    }

    private void initProcesPoses(){
        list3.add(SwerveDriveConstants.MapPoses.tag3Mid);
        myProsMap.put(3, list3);

        list16.add(SwerveDriveConstants.MapPoses.tag16Mid);
        myProsMap.put(16, list16);
    } 

    private void initStationsPoses() {
        list12.add(SwerveDriveConstants.MapPoses.tag12Left);
        list12.add(SwerveDriveConstants.MapPoses.tag12Right);
        list12.add(SwerveDriveConstants.MapPoses.tag12Mid);
        myStationMap.put(12, list12);

        list13.add(SwerveDriveConstants.MapPoses.tag13Left);
        list13.add(SwerveDriveConstants.MapPoses.tag13Right);
        list13.add(SwerveDriveConstants.MapPoses.tag13Mid);
        myStationMap.put(13, list13);

        list1.add(SwerveDriveConstants.MapPoses.tag1Left);
        list1.add(SwerveDriveConstants.MapPoses.tag1Right);
        list1.add(SwerveDriveConstants.MapPoses.tag1Mid);
        myStationMap.put(1, list1);

        list2.add(SwerveDriveConstants.MapPoses.tag2Left);
        list2.add(SwerveDriveConstants.MapPoses.tag2Right);
        list2.add(SwerveDriveConstants.MapPoses.tag2Mid);
        myStationMap.put(2, list2);
    } 

    private void initReefSidePoses() {
        list17.add(SwerveDriveConstants.MapPoses.tag17Left);
        list17.add(SwerveDriveConstants.MapPoses.tag17Right);
        list17.add(SwerveDriveConstants.MapPoses.tag17Mid);
        myReefMap.put(17, list17);

        list18.add(SwerveDriveConstants.MapPoses.tag18Left);
        list18.add(SwerveDriveConstants.MapPoses.tag18Right);
        list18.add(SwerveDriveConstants.MapPoses.tag18Mid);
        myReefMap.put(18, list18);

        list19.add(SwerveDriveConstants.MapPoses.tag19Left);
        list19.add(SwerveDriveConstants.MapPoses.tag19Right);
        list19.add(SwerveDriveConstants.MapPoses.tag19Mid);
        myReefMap.put(19, list19);

        list20.add(SwerveDriveConstants.MapPoses.tag20Left);
        list20.add(SwerveDriveConstants.MapPoses.tag20Right);
        list20.add(SwerveDriveConstants.MapPoses.tag20Mid);
        myReefMap.put(20, list20);

        list21.add(SwerveDriveConstants.MapPoses.tag21Left);
        list21.add(SwerveDriveConstants.MapPoses.tag21Right);
        list21.add(SwerveDriveConstants.MapPoses.tag21Mid);
        myReefMap.put(21, list21);

        list22.add(SwerveDriveConstants.MapPoses.tag22Left);
        list22.add(SwerveDriveConstants.MapPoses.tag22Right);
        list22.add(SwerveDriveConstants.MapPoses.tag22Mid);
        myReefMap.put(22, list22);

        list6.add(SwerveDriveConstants.MapPoses.tag6Left);
        list6.add(SwerveDriveConstants.MapPoses.tag6Right);
        list6.add(SwerveDriveConstants.MapPoses.tag6Mid);
        myReefMap.put(6, list6);

        list7.add(SwerveDriveConstants.MapPoses.tag7Left);
        list7.add(SwerveDriveConstants.MapPoses.tag7Right);
        list7.add(SwerveDriveConstants.MapPoses.tag7Mid);
        myReefMap.put(7, list7);

        list8.add(SwerveDriveConstants.MapPoses.tag8Left);
        list8.add(SwerveDriveConstants.MapPoses.tag8Right);
        list8.add(SwerveDriveConstants.MapPoses.tag8Mid);
        myReefMap.put(8, list8);

        list9.add(SwerveDriveConstants.MapPoses.tag9Left);
        list9.add(SwerveDriveConstants.MapPoses.tag9Right);
        list9.add(SwerveDriveConstants.MapPoses.tag9Mid);
        myReefMap.put(9, list9);

        list10.add(SwerveDriveConstants.MapPoses.tag10Left);
        list10.add(SwerveDriveConstants.MapPoses.tag10Right);
        list10.add(SwerveDriveConstants.MapPoses.tag10Mid);
        myReefMap.put(10, list10);

        list11.add(SwerveDriveConstants.MapPoses.tag11Left);
        list11.add(SwerveDriveConstants.MapPoses.tag11Right);
        list11.add(SwerveDriveConstants.MapPoses.tag11Mid);
        myReefMap.put(11, list11);
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
    
            if (useVision){
                visionupdateOdometry(VisionConstants.kLimelightBackLeftName); 
                visionupdateOdometry(VisionConstants.kLimelightBackRightName);
                visionupdateOdometry(VisionConstants.kLimelightFrontLeftName);
                visionupdateOdometry(VisionConstants.kLimelightFrontRightName);
            }
        
            //todo try MegaTag2
    }

    //******************************  Vision ******************************/
	private void visionupdateOdometry(String limelightName) {
        boolean useMegaTag2 = true; //set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if(useMegaTag2 == false)
        {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        
        if (mt1 == null){
            return;
        }
        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
            doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
            doRejectUpdate = true;
            }
        }
        if(mt1.tagCount == 0)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            poseEstimator.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
        }
        }
        else if (useMegaTag2 == true)
        {
        double currentPoseYaw = RobotContainer.IsRedSide() ? poseEstimator.getEstimatedPosition().getRotation().getDegrees() + 180 : poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(limelightName, currentPoseYaw, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2 == null){
            return;
        }
        // if(Math.abs(gyro.) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        // {
        //     doRejectUpdate = true;
        // }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
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

        // SmartDashboard.putString(limelightName + "Info", log);

        if(!doRejectUpdate)
        {
            Pose2d botPose1 = estimate.pose;

            // SmartDashboard.putNumber(limelightName + " X Position", botPose1.getX());
            // SmartDashboard.putNumber(limelightName + " Y Position", botPose1.getY());
            // SmartDashboard.putNumber(limelightName + " Rotation"  , botPose1.getRotation().getDegrees());
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

    private int getFrontCameraApriltagID(int zoneId)
    {
        if(zoneId == 2 || zoneId == 4) {
            //todo
            // return vision.getLargerApriltagByTa(VisionConstants.??);
        }
        if(zoneId == 2)
        {
            if(poseEstimator.getEstimatedPosition().getY() > 4) {
                return RobotContainer.IsRedSide() ? 2 : 13;
            } else if (poseEstimator.getEstimatedPosition().getY() < 4) {
                return RobotContainer.IsRedSide() ? 1 : 12;
            }
        } else if(zoneId == 4) {
            return RobotContainer.IsRedSide() ? 3 : 16;
        }
        return -1;
    }
            
    private int getMostClosedApriltagIdInReefZone(int zoneId) {
        if(zoneId == 1) {
            int startIndex = RobotContainer.IsRedSide() ? 6 : 17;
            int indexToGet = -1;
            double distance = getDistanceFromTag(false, startIndex);
            for (int index = startIndex; index <= startIndex + 5; index++) {
                double distance2 = getDistanceFromTag(false, index);
                if(distance2 < distance) {
                    distance = distance2;
                    indexToGet = index;
                }
            }
            return indexToGet;
        }
            // return vision.getLargerApriltagByTa(VisionConstants.kLimelightBackLeftName, 
            //     VisionConstants.kLimelightBackRightName);
        return -1; 
    }

    private Map<Integer, ArrayList<Pose2d>> myReefMap = new HashMap<>();
    private Map<Integer, ArrayList<Pose2d>> myProsMap = new HashMap<>();
    private Map<Integer, ArrayList<Pose2d>> myStationMap = new HashMap<>();
    private Map<Integer, ArrayList<Pose2d>> myCageMap = new HashMap<>();

    private Pose2d calcuTargetPoseByReq(int zoneId, int poseId)
    {
        Pose2d targetPose = poseEstimator.getEstimatedPosition();
        if(zoneId == 1) // own reef
        {
            // obtain the closed apriltag id from two low-back cameras.
            int targetApriltagId = getMostClosedApriltagIdInReefZone(zoneId); 
            if(myReefMap.containsKey(targetApriltagId))
            {
                if(poseId == -1)
                {
                    return myReefMap.get(targetApriltagId).get(0); // the left side of one apriltag on reef
                }
                else if(poseId == 1)
                {
                    return myReefMap.get(targetApriltagId).get(1);// the right side of one apriltag on reef
                }
                else if(poseId == 0) 
                {
                    return myReefMap.get(targetApriltagId).get(2); // Middle of apriltag 
                }
            }
        }
        // todo other zone
        else if (zoneId == 2)
        {
            // stations
            int aid = getFrontCameraApriltagID(zoneId);
            if(myStationMap.containsKey(aid))
            {
                if(poseId == -1 )
                {
                    return myStationMap.get(aid).get(0);
                }
                else if(poseId == 1)
                {
                    return myStationMap.get(aid).get(1);
                }
                else if(poseId == 0)
                {
                    return myStationMap.get(aid).get(2);
                }
            }

        }else if (zoneId == 3)
        {
            // cage: need to consider the pose is too close to reef!!
            // only enable it druing last 25 seconds
            // todo
            int aid = RobotContainer.IsRedSide() ? 15 : 4;;
            
            if(myCageMap.containsKey(aid))
            {
                if(poseId == -1 )
                {
                    return myCageMap.get(aid).get(0); //  cage left
                }
                else if( poseId == 1)
                {
                    return myCageMap.get(aid).get(1); //  cage right
                }
                else if( poseId == 0)
                {
                    return myCageMap.get(aid).get(2); //  cage center
                }
            }
            
        }else if (zoneId == 4)
        {
            // processor
            int aid = getFrontCameraApriltagID(zoneId);
            if(myProsMap.containsKey(aid))
            {
                if(poseId == -1 || poseId == 0 || poseId == 1)
                {
                    return myProsMap.get(aid).get(0);
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
        
        if(destPoseInBlue == null) return;

        PathConstraints pathcons = new PathConstraints(
            maxVelocityMps, maxAccelerationMpsSq, 
            Units.degreesToRadians(360), Units.degreesToRadians(720)
        );

        pathfindingCommand = AutoBuilder.pathfindToPose(destPoseInBlue, pathcons);

        pathfindingCommand.schedule();
    }

    public void stopAutoPath() {
        if (pathfindingCommand != null && !pathfindingCommand.isFinished()) {
            pathfindingCommand.cancel();
            stopModules();
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
            // Reef
            if(NerdyMath.isPoseInsideCircleZone(13, 4, 7.5, xp, yp)) {
                return 1;
            }
            // Station
            else if(NerdyMath.isPoseInsideCircleZone(16.7, 0.66, 4, xp, yp) || NerdyMath.isPoseInsideCircleZone(16.7, 7.4, 4, xp, yp)) {
                return 2;
            }
            // Cage
            else if(7.4 < xp && xp < 10)  {
            // cage: need to consider the pose is too close to reef!!
            // only enable it druing last 25 seconds
            // todo
                return 3;
            }
            // Processor
            else if(NerdyMath.isPoseInsideCircleZone(11.56, 8.06, 2.12, xp, yp)) {
                return 4;
            }
        }
        else
        {
            // Reef
            if(NerdyMath.isPoseInsideCircleZone(4.5, 4, 5.06, xp, yp)) {
                return 1;
            }
            // Bot and Top Station
            else if(NerdyMath.isPoseInsideCircleZone(1, 0.5, 4, xp, yp) || NerdyMath.isPoseInsideCircleZone(1, 7.5, 4, xp, yp)) {
                return 2;
            }
            // cage
            else if (7.4 < xp && xp < 10)  {
                // cage: need to consider the pose is too close to reef!!
                // only enable it druing last 25 seconds
                // todo
                    return 3;
                }
            else if(NerdyMath.isPoseInsideCircleZone(6.35, 0, 2.12, xp, yp)) {
                return 4;
            }
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
        Pose2d tagPose = layout.getTagPose(tagID).get().toPose2d();
        Rotation2d tagRotation = tagPose.getRotation();
        Rotation2d tagRotationInverse = new Rotation2d(-tagRotation.getRadians());
        Double theta_0 = tagRotationInverse.getRadians();
        Double moveBy = 0.5; //TODO: Change Later
        // D_x = R_x + M cos (theta_0)
        // D_y = R_y + M sin (theta_0)
        Transform2d transformer = new Transform2d((tagPose.getX() + moveBy * Math.cos(theta_0)), (tagPose.getY() + moveBy * Math.sin(theta_0)), tagRotation);
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

    /**
     * Calculate position to move to based on Reef side AprilTags
     * @param tagID ID of tag to move to
     * @param side -1 for Left, 0 for Middle, 1 for Right
     * @return Pose2d of position to drive to
     */
    public Pose2d calcReefSidePose(int tagID, int side) {
        // get tag info
        Pose2d tagPose = layout.getTagPose(tagID).get().toPose2d();
        double tagAngle = tagPose.getRotation().getRadians();

        // add vert offset and find bot rotation
        double xPos = tagPose.getX() + ReefOffsets.frontOffset*Math.cos(tagAngle);
        double yPos = tagPose.getY() + ReefOffsets.frontOffset*Math.sin(tagAngle);
        Rotation2d botRotation = new Rotation2d(-tagAngle);

        if (side == 0) return new Pose2d(xPos, yPos, botRotation); // mid pose has vert offset

        // use side rotation to add side offset
        Rotation2d sideRotation;
        if (side == -1) sideRotation = new Rotation2d(botRotation.getRadians() + Math.PI/2); // left: 90 deg ccw
        else sideRotation = new Rotation2d(botRotation.getRadians() - Math.PI/2); // right: 90 deg cw

        xPos += ReefOffsets.sideOffset*Math.cos(sideRotation.getRadians());
        yPos += ReefOffsets.sideOffset*Math.sin(sideRotation.getRadians());

        return new Pose2d(xPos, yPos, botRotation); // side poses have vert and side offsets
    }

    /**
     * Calculate position to move to based on Station side AprilTags
     * @param tagID ID of tag to move to
     * @param side -1 for Left, 0 for Middle, 1 for Right
     * @return Pose2d of position to drive to
     */
    public Pose2d calcStationSidePose(int tagID, int side) {
        // get tag info
        Pose2d tagPose = layout.getTagPose(tagID).get().toPose2d();
        double tagAngle = tagPose.getRotation().getRadians();

        // add vert offset and find bot rotation
        double xPos = tagPose.getX() + StationOffsets.frontOffset*Math.cos(tagAngle);
        double yPos = tagPose.getY() + StationOffsets.frontOffset*Math.sin(tagAngle);
        Rotation2d botRotation = new Rotation2d(-tagAngle);

        if (side == 0) return new Pose2d(xPos, yPos, botRotation); // mid pose has vert offset

        // use side rotation to add side offset
        Rotation2d sideRotation;
        if (side == -1) sideRotation = new Rotation2d(botRotation.getRadians() + Math.PI/2); // left: 90 deg ccw
        else sideRotation = new Rotation2d(botRotation.getRadians() - Math.PI/2); // right: 90 deg cw

        xPos += StationOffsets.sideOffset*Math.cos(sideRotation.getRadians());
        yPos += StationOffsets.sideOffset*Math.sin(sideRotation.getRadians());

        return new Pose2d(xPos, yPos, botRotation); // side poses have vert and side offsets
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

    public Command setVisionEnabledCommand(boolean useVision) {
        return Commands.runOnce(() -> this.useVision = useVision);
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
                tab.add("Field Position", field).withSize(6, 3);
            case MINIMAL:
                tab.addNumber("X Position (m)", () -> poseEstimator.getEstimatedPosition().getX());
                tab.addNumber("Y Position (m)", () -> poseEstimator.getEstimatedPosition().getY());
                tab.addNumber("Odometry Angle", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees());
                // tab.add("Pose Estimator Pose", poseEstimator.getEstimatedPosition());
                tab.addString("Pose Estimator Pose Str", () -> poseEstimator.getEstimatedPosition().toString());
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