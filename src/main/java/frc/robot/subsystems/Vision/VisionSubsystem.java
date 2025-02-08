package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LimelightHelpers;
import frc.Util.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;

import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    
    // Vision settings
    public boolean useVision = true;

    // Subsystem & Limelight
    private final CommandSwerveDrivetrain m_drive;
    private final String limelightNames;

    // Pose estimation
    private double averageTagDistance = 0.0;
    private static PoseEstimate mt2;
    private static PoseEstimate oldMt = new PoseEstimate();

    public VisionSubsystem(CommandSwerveDrivetrain m_drive, String limelightNames) {
        this.m_drive = m_drive;
        this.limelightNames = limelightNames;
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames);
    }

    @Override
    public void periodic() {
        // Update robot orientation in Limelight
        LimelightHelpers.SetRobotOrientation(
            limelightNames, 
            m_drive.getState().Pose.getRotation().getDegrees(), 
            0, 0, 0, 0, 0
        );

        // Count detected tags
        int tagsDetected = LimelightHelpers.getTargetCount(limelightNames);
        averageTagDistance = (mt2 == null) ? 0 : mt2.avgTagDist;

        // Calculate standard deviations
        double xyStdDev = VisionConstants.xyStdDevCoefficient 
                        * Math.pow(averageTagDistance, 2) 
                        / ((tagsDetected == 0) ? 100 : tagsDetected);

        double thetaStdDev = VisionConstants.thetaStdDevCoefficient 
                           * Math.pow(averageTagDistance, 2) 
                           / ((tagsDetected == 0) ? 100 : tagsDetected);

        // Apply measurement standard deviations
        m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));

        // Process odometry and log vision data
        m_drive.filterOutOfFieldData();
        odometryWithVision(limelightNames, xyStdDev, thetaStdDev);

        Logger.recordOutput("Vision/Distance from tag", averageTagDistance);
        Logger.recordOutput("Vision/XY STD", xyStdDev);
        Logger.recordOutput("Vision/Theta STD", thetaStdDev);
        Logger.recordOutput("Vision/Using Vision", useVision);
    }

    public void odometryWithVision(String limelightName, double xySTD, double thetaSTD) {
        if (LimelightHelpers.getTV(limelightName)) {
            
            if (mt2 != null) {
                oldMt = mt2;
            }

            // Fetch latest bot pose
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
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

            SmartDashboard.putBoolean("Rejected Update", doRejectUpdate);
        }
    }

    public Command shouldUseVision(boolean vision) {
        return Commands.runOnce(() -> useVision = vision, this);
    }
  }