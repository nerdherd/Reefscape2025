package frc.robot.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.Constants;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.vision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class VisionSys {

    private ArrayList<String> vision = new ArrayList<>();

    private static int DefaultPipelineNumber = 0;

    public VisionSys()
    {
        vision.add(Constants.VisionConstants.kLimelightBackLeftName);
        vision.add(Constants.VisionConstants.kLimelightBackRightName);
        vision.add(Constants.VisionConstants.kLimelightFrontLeftName);
        vision.add(Constants.VisionConstants.kLimelightFrontRightName);
    }

    public void setEscapeApriltagID()
    {
        int[] validIDs = {3,4}; // update them based on the field situation
        for (String c : vision) {
            LimelightHelpers.SetFiducialIDFiltersOverride(c, validIDs);
        }
    }

    public void setPipelineIndex(int pipelineNumber)
    {
        DefaultPipelineNumber = pipelineNumber;
        for (String c : vision) {
            LimelightHelpers.setPipelineIndex(c, pipelineNumber);
        }
    }

    public boolean validCheck()
    {
        for (String c : vision) {
            if(LimelightHelpers.getCurrentPipelineIndex(c) != DefaultPipelineNumber)
                return false;
        }
        return true;
    }

    public void applyPoseEst(SwerveDrivePoseEstimator m_poseEstimator, PigeonV2 m_gyro)
    {
        boolean doRejectUpdate = false;
        double poseRotationDegree = m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation("limelight", poseRotationDegree, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
        // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        if(Math.abs(m_gyro.getRate()) > 360)
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            m_poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
    }

    /*To use the LL4 IMU, currently your LL must be mounted in "landscape" mode.
The flow looks like this:
Call SetRobotOrientation() with your "external" IMU such as a Pigeon or a NavX. You can continue to call this method as often as you would like.
Call SetIMUMode() to configure how your Limelight utilizes IMU data from internal and external IMUs.
In general, use mode 1 while your robot is waiting for the autonomous period to begin. Switch to mode 2 while enabled or while enabled and turning. */
    public void setSeeding(int mode)
    {
        for (String c : vision) 
            LimelightHelpers.SetIMUMode(c, mode);
    }

    public int getLargerApriltagByTa(String camera1, String camera2) // todo testing pls
    {
        double ta1 = LimelightHelpers.getTA(camera1);
        double ta2 = LimelightHelpers.getTA(camera2);
        int id1 = (int) LimelightHelpers.getFiducialID(camera1);
        int id2 = (int) LimelightHelpers.getFiducialID(camera2);
        
        if (ta1 <= 0 && ta2 <= 0) {
            return -1; // No valid targets
        }            
        // If only camera1 has a valid target
        if (ta1 > 0 && ta2 <= 0) {
            return id1;
        }            
        // If only camera2 has a valid target
        if (ta2 > 0 && ta1 <= 0) {
            return id2;
        }
        
        // Both cameras have targets, return the ID of the larger one
        return (ta1 > ta2) ? id1 : id2;
    }

    public int getLargerApriltagByTa(String camera1) // todo testing pls
    {
        double ta1 = LimelightHelpers.getTA(camera1);

        // If only camera1 has a valid target
        if (ta1 > 0.2 ) {
            return (int) LimelightHelpers.getFiducialID(camera1);
        }
        else{
            return -1;
        }
    }
    
}
