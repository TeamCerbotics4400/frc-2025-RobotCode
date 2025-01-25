package frc.robot.subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.VisionConstants;

import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

  public boolean useVision = true;
  private final CommandSwerveDrivetrain m_drive;
  private String limelightNames;
  private double averageTagDistance = 0.0;
  private static LimelightHelpers.PoseEstimate mt2;

  public VisionSubsystem(CommandSwerveDrivetrain m_drive, String limelightNames) {
    this.m_drive = m_drive;
    this.limelightNames = limelightNames;
    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames);
  }

  @Override
  public void periodic() {

    int tagsDetected = LimelightHelpers.getTargetCount(limelightNames);

    averageTagDistance = mt2.avgTagDist;

    double xyStdDev =
        VisionConstants.xyStdDevCoefficient
            * Math.pow(averageTagDistance, 2)
            / (tagsDetected == 0 ? 100 : tagsDetected);

    double thetaStdDev =
        VisionConstants.thetaStdDevCoefficient
            * Math.pow(averageTagDistance, 2)
            / (tagsDetected == 0 ? 100 : tagsDetected);

    m_drive.filterOutOfFieldData();
    odometryWithVision(VisionConstants.tagLimelightName, xyStdDev, thetaStdDev);

    Logger.recordOutput("Vision/Distance from tag", averageTagDistance);
    Logger.recordOutput("Vision/XY STD", xyStdDev);
    Logger.recordOutput("Vision/Theta STD", thetaStdDev);
    Logger.recordOutput("Vision/Using Vision", useVision);
  }

  public void odometryWithVision(String limelightName, double xySTD, double thetaSTD) {
    if (LimelightHelpers.getTV(limelightName)) {
      LimelightHelpers.PoseEstimate oldMt = mt2;
      boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(
          limelightName, m_drive.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
      if (Math.abs(m_drive.getPigeon2().getAngularVelocityZWorld().getValueAsDouble())
          > 720)
      {
        doRejectUpdate = true;
      }

      if (mt2.pose.getX() < -Constants.FieldConstants.fieldBorderMargin
          || mt2.pose.getX()
              > Constants.FieldConstants.fieldLength
                  + Constants.FieldConstants.fieldBorderMargin
          || mt2.pose.getY() < -Constants.FieldConstants.fieldBorderMargin
          || mt2.pose.getY()
              > Constants.FieldConstants.fieldWidth
                  + Constants.FieldConstants.fieldBorderMargin) {
        doRejectUpdate = true;
      }

      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (Math.abs(oldMt.pose.getX() - mt2.pose.getX()) > 0.5
          || Math.abs(oldMt.pose.getY() - mt2.pose.getY()) > 0.5) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {

        m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(xySTD, xySTD, thetaSTD));
        m_drive.addVisionMeasurement(
            mt2.pose.div(Constants.DriveConstants.driveOdometryRatio), mt2.timestampSeconds);
      }
      SmartDashboard.putBoolean("Rejected Update", doRejectUpdate);
    }
  }

  public Command shouldUseVision(boolean vision) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
              useVision = vision;
            },
            this);
    return ejecutable;
  }
}