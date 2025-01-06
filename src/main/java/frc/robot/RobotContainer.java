// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public static final CommandSwerveDrivetrain m_drive = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drive.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_drive.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(m_drive.applyRequest(() -> brake));
        joystick.b().whileTrue(m_drive.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(m_drive.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(m_drive.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*joystick.back().and(joystick.y()).whileTrue(m_drive.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(m_drive.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(m_drive.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(m_drive.sysIdQuasistatic(Direction.kReverse));*/

        joystick.y().onTrue(pathfindAndAlignAmp());

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(m_drive.runOnce(() -> m_drive.seedFieldCentric()));

        m_drive.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public static Command pathfindAndAlignAmp() {
        return Commands.sequence(
            Commands.either(
                m_drive
                        .goToPose(new Pose2d(5,5,new Rotation2d()))
                        .until(
                            () ->
                                m_drive
                                        .getState()
                                        .Pose
                                        .getTranslation()
                                        .getDistance(new Translation2d(5,5))
                                    <= 3),
                    m_drive
                        .goToPose(new Pose2d(5,5,new Rotation2d()))
                        .until(
                            () ->
                                m_drive
                                        .getState()
                                        .Pose
                                        .getTranslation()
                                        .getDistance(new Translation2d(5,5))
                                    <= 3),
                    Robot::isRedAlliance)
                .andThen(
                    new RunCommand(
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
                                  ? new Pose2d(5,5,new Rotation2d()) //red
                                  : new Pose2d(5,5,new Rotation2d());
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
    
                    Command followPathCommand = AutoBuilder.followPath(path);
                    followPathCommand.schedule();
                                        })));
      }
}
