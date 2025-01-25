// Copyright (c) 2025 FRC 4400//
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.CustomDashboardUtil;
import frc.Util.LocalADStarAK;
import frc.robot.Commands.FieldCentricDrive;
import frc.robot.Commands.AutoCommands.AutoCommand;
import frc.robot.Commands.AutoCommands.Paths.NoneAuto;
import frc.robot.Commands.AutoCommands.Paths.WorkShopPaths.TestAuto;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Swerve.TunerConstants;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  private final CommandXboxController chassisDriver = new CommandXboxController(0);

  public static final CommandSwerveDrivetrain m_drive = TunerConstants.createDrivetrain();

  private final SendableChooser<AutoCommand> autoChooser = new SendableChooser<>();

  public static Field2d autoFieldPreview = new Field2d();
  public CustomDashboardUtil m_dashboard = new CustomDashboardUtil();
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  public RobotContainer() {
  /* Path follower */

    autoChooser.setDefaultOption("Nothing Path", new NoneAuto());
    autoChooser.addOption("Test Auto", new TestAuto());

    autoChooser.onChange(auto->{
        autoFieldPreview.getObject("path").setPoses(auto.getAllPathPoses());
    });

    PathPlannerLogging.setLogActivePathCallback(
            (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));
    PathPlannerLogging.setLogTargetPoseCallback(
            pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));

    Pathfinding.setPathfinder(new LocalADStarAK());
    /*This code warms up the library to avoid delay on the path */
    PathfindingCommand.warmupCommand().schedule();
  
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("Auto Preview", autoFieldPreview);

    configureBindings();
  }

  private void configureBindings() {

    m_drive.setDefaultCommand(
        new FieldCentricDrive(
            m_drive,
             ()-> chassisDriver.getLeftY(),
             ()-> chassisDriver.getLeftX(), 
             ()-> chassisDriver.getRightX()));

    chassisDriver.y().onTrue(
        new ParallelRaceGroup(
            pathFindAndAlignCommand(m_dashboard.getReefSelected()),
             new SequentialCommandGroup(new WaitCommand(1),
              new WaitCommand(10000).until(()->isJoystickActive()))));

    chassisDriver.a().onTrue(m_drive.runOnce(() -> m_drive.seedFieldCentric()));

    m_drive.registerTelemetry(logger::telemeterize);
  }

  public static Command pathFindAndAlignCommand(int val) {
    return Commands.sequence(
        Commands.either(
                m_drive
                    .goToPose(FieldConstants.redSidePositions[val])
                    .until(
                        () ->
                            m_drive
                                    .getState()
                                    .Pose
                                    .getTranslation()
                                    .getDistance(FieldConstants.redSidePositions[val].getTranslation())
                                <= 3),
                m_drive
                    .goToPose(FieldConstants.blueSidePositions[val])
                    .until(
                        () ->
                            m_drive
                                    .getState()
                                    .Pose
                                    .getTranslation()
                                    .getDistance(FieldConstants.blueSidePositions[val].getTranslation())
                                <= 3),
                Robot::isRedAlliance)
            .andThen(
                new DeferredCommand(
                    () -> {
                      Pose2d currentPose = m_drive.getState().Pose;
                      ChassisSpeeds currentSpeeds =
                          ChassisSpeeds.fromRobotRelativeSpeeds(
                              m_drive.getCurrentChassisSpeeds(), currentPose.getRotation());

                      Rotation2d heading =
                          new Rotation2d(
                              currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

                      Pose2d targetPose =
                          Robot.isRedAlliance()
                              ? FieldConstants.redSidePositions[val]
                              : FieldConstants.blueSidePositions[val];
                      var bezierPoints =
                          PathPlannerPath.waypointsFromPoses(
                              new Pose2d(currentPose.getTranslation(), heading), targetPose);
                      PathPlannerPath path =
                          new PathPlannerPath(
                              bezierPoints,
                              new PathConstraints(
                                  2.0,
                                  3.0,
                                  Units.degreesToRadians(360),
                                  Units.degreesToRadians(360)),
                              null,
                              new GoalEndState(0.0, targetPose.getRotation()));
                      path.preventFlipping = true;

                      return AutoBuilder.followPath(path);
                    },
                    Set.of(m_drive))));
  }

  private boolean isJoystickActive() {
    double deadband = 0.2; // Threshold for joystick movement
    return Math.abs(chassisDriver.getLeftX()) > deadband ||
           Math.abs(chassisDriver.getLeftY()) > deadband ||
           Math.abs(chassisDriver.getRightX()) > deadband;
}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
