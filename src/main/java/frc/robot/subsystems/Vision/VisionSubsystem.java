package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LimelightHelpers;
import frc.Util.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;


public class VisionSubsystem extends SubsystemBase {
    
    // Vision settings
    public boolean useVision = true;
    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

    // Subsystem & Limelight
    private final CommandSwerveDrivetrain m_drive;
    private final String limelightNames = "limelight-tags";

    // Pose estimation
    private double averageTagDistance = 0.0;
    private static PoseEstimate mt2;
    private static PoseEstimate oldMt = new PoseEstimate();

    //Logging
    List<Pose3d> allTagPoses = new ArrayList<>();

    public VisionSubsystem(CommandSwerveDrivetrain m_drive) {
        this.m_drive = m_drive;
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames) == null ? new PoseEstimate():LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames) ;
    }

    @Override
    public void periodic() {
        // Update robot orientation in Limelight
        LimelightHelpers.SetRobotOrientation(
            limelightNames, 
            m_drive.getState().Pose.getRotation().getDegrees(), 
            0, 0, 0, 0, 0
        );

        averageTagDistance = (mt2 == null) ? 0 : mt2.avgTagDist;

        // Calculate standard deviations
        double xyStdDev = VisionConstants.xyStdDevCoefficient 
                        * Math.pow(averageTagDistance, 2) 
        / ((LimelightHelpers.getTargetCount(limelightNames) == 0) ? 100 : LimelightHelpers.getTargetCount(limelightNames));

        lastTagDetectionTimes.put((int)LimelightHelpers.getFiducialID(limelightNames), Timer.getTimestamp());

        // Apply measurement standard deviations
        m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));

        // Process odometry and log vision data
        m_drive.filterOutOfFieldData();
        odometryWithVision(limelightNames, xyStdDev,0);

        Logger.recordOutput("Vision/Detected tag",LimelightHelpers.getTV(limelightNames));
        Logger.recordOutput("Vision/Distance from tag",averageTagDistance);

    }

    public void odometryWithVision(String limelightName, double xySTD, double thetaSTD) {
        if (LimelightHelpers.getTV(limelightNames) && LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames) != null) {
            
            if (mt2 != null) {
                oldMt = mt2;
            }

            // Fetch latest bot pose
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames) == null ? new PoseEstimate():LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames) ;
            boolean doRejectUpdate = false;

            // Reject updates if robot is spinning too fast
            if (Math.abs(m_drive.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) {
                doRejectUpdate = true;
            }

            // Reject updates if pose is out of field bounds
            boolean outOfBounds = 
                   (mt2.pose.getX() < -Constants.FieldConstants.fieldBorderMargin)
                || (mt2.pose.getX() > Constants.FieldConstants.fieldLength + Constants.FieldConstants.fieldBorderMargin)
                || (mt2.pose.getY() < -Constants.FieldConstants.fieldBorderMargin)
                || (mt2.pose.getY() > Constants.FieldConstants.fieldWidth + Constants.FieldConstants.fieldBorderMargin);
                
            if (outOfBounds || mt2.tagCount == 0) {
                doRejectUpdate = true;
            }

            // Reject updates if there's a sudden position jump
            boolean suddenJump = 
                   (Math.abs(oldMt.pose.getX() - mt2.pose.getX()) > 0.2)
                || (Math.abs(oldMt.pose.getY() - mt2.pose.getY()) > 0.2);

            if (suddenJump) {
                doRejectUpdate = true;
            }

            // Apply the update if valid
            if (!doRejectUpdate) {
                m_drive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
            Logger.recordOutput("MegatagPose", mt2.pose);
        }
    }
  }