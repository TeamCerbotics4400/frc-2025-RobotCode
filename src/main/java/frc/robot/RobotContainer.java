// Copyright (c) 2025 FRC 4400//
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.CustomDashboardUtil;
import frc.Util.LocalADStarAK;
import frc.robot.Commands.FieldCentricDrive;
import frc.robot.Commands.IntakeSequenceCommand;
import frc.robot.Commands.AutoCommands.AutoCommand;
import frc.robot.Commands.AutoCommands.Paths.NoneAuto;
import frc.robot.Commands.AutoCommands.Paths.WorkShopPaths.TestAuto;
import frc.robot.Commands.AutoCommands.SubsystemCommands.LeaveReefCommand;
import frc.robot.Commands.IntakeCommand.IntakeSequence1;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.Climber.ClimberIO;
import frc.robot.Subsystems.Climber.ClimberIOKraken;
import frc.robot.Subsystems.Climber.ClimberSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOKraken;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOKraken;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Swerve.TunerConstants;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  /* Driver controllers*/
  private final CommandXboxController chassisDriver = new CommandXboxController(0);
  private final CommandXboxController subsystemsDriver = new CommandXboxController(1);

  /* Subsystems with their respective IO's */
  public static final CommandSwerveDrivetrain m_drive = TunerConstants.createDrivetrain();

  /* Elevator */
  public static final ElevatorIO elevatorIO = new ElevatorIOKraken();
  public static ElevatorSubsystem m_elevator;

  /* Intake */
  public static final IntakeIO intakeIO = new IntakeIOKraken();
  public static IntakeSubsystem m_intake;

  /* Climber */
  public static final ClimberIO climberIO = new ClimberIOKraken();
  public static ClimberSubsystem m_climber;

  /* Chooser for autonomous */
  private final SendableChooser<AutoCommand> autoChooser = new SendableChooser<>();

  /* Set up for utils */
  public static Field2d autoFieldPreview = new Field2d();
  public CustomDashboardUtil m_dashboard = new CustomDashboardUtil();
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  public RobotContainer() {

    /* IO options for replay */
    switch (Constants.currentMode) {
      /* Configs for the REAL robot */
      case REAL:
        m_elevator = new ElevatorSubsystem(elevatorIO);
        m_intake = new IntakeSubsystem(intakeIO);
      m_climber = new ClimberSubsystem(climberIO);
        break;
      /* Configs to replay a log */
      case REPLAY:
        m_elevator = new ElevatorSubsystem(new ElevatorIO(){});
        m_intake = new IntakeSubsystem(new IntakeIO(){});
        m_climber = new ClimberSubsystem(new ClimberIO(){});
      break;
      /* Default to just in case it somehow fails, lol */
      default:
      m_elevator = new ElevatorSubsystem(elevatorIO);
      m_intake = new IntakeSubsystem(intakeIO);
      m_climber = new ClimberSubsystem(climberIO);
        break;
    }
    
    enableNamedCommands();
  /* Path follower */
    autoChooser.setDefaultOption("Nothing Path", new NoneAuto());
    autoChooser.addOption("Test Auto", new TestAuto());

    autoChooser.onChange(auto->{
        autoFieldPreview.getObject("path").setPoses(auto.getAllPathPoses());
    });

    /* Record path poses and targets for logging */
    PathPlannerLogging.setLogActivePathCallback(
            (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));
    PathPlannerLogging.setLogTargetPoseCallback(
            pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));

    Pathfinding.setPathfinder(new LocalADStarAK());
    /*This code warms up the library to avoid delay on the path */
    PathfindingCommand.warmupCommand().schedule();
  
    /* Shows the preview before match setup */
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("Auto Preview", autoFieldPreview);

    configureBindings();
  }

  private void configureBindings() {

    //To leade at level 1   0.35, 0.1 on setvoltageCommadn
    /*chassisDriver.a().onTrue(m_elevator.goToPosition(0.0));

    chassisDriver.b().onTrue(m_elevator.goToPosition(0.5));

    chassisDriver.x().onTrue(m_elevator.goToPosition(0.2));

    chassisDriver.y().onTrue(m_elevator.goToPosition(1.67));

    chassisDriver.rightBumper().whileTrue(m_intake.setVoltageCommand(0.3, 0.3)).whileFalse(m_intake.setVoltageCommand(0,0));*/


    //elevador pos
    chassisDriver.a().onTrue(m_elevator.goToPosition(0.0));

    chassisDriver.povLeft().onTrue(m_elevator.goToPosition(.15));

    chassisDriver.b().onTrue(m_elevator.goToPosition(0.4));

    chassisDriver.x().onTrue(m_elevator.goToPosition(.91));

    chassisDriver.y().onTrue(m_elevator.goToPosition(1.67));


    chassisDriver.povUp().whileTrue(m_climber.setClimberVoltage(0.3)).whileFalse(m_climber.setClimberVoltage(0));

    chassisDriver.povDown().whileTrue(m_climber.setClimberVoltage(-0.5)).whileFalse(m_climber.setClimberVoltage(0));


    //climber power
    chassisDriver.rightBumper().onTrue(new IntakeSequenceCommand(m_intake));
    chassisDriver.leftBumper().whileTrue(m_intake.setVoltageCommand(0.3, 0.3)).whileFalse(m_intake.setVoltageCommand(0,0));

    chassisDriver.povRight().whileTrue(m_intake.setVoltageCommand(.15,.35)).whileFalse(m_intake.setVoltageCommand(0, 0));

     m_drive.setDefaultCommand(
        new FieldCentricDrive(
            m_drive,
             ()-> -chassisDriver.getLeftY(),
             ()-> -chassisDriver.getLeftX(), 
             ()-> -chassisDriver.getRightX()));

    /*chassisDriver.leftBumper().onTrue(
        new ParallelRaceGroup(
            pathFindAndAlignCommand(()->m_dashboard.getReefSelected()),
             new SequentialCommandGroup(new WaitCommand(1),
              new WaitCommand(10000).until(()->isJoystickActive()))));*/

    chassisDriver.button(7).onTrue(m_drive.runOnce(() -> m_drive.seedFieldCentric()));

    m_drive.registerTelemetry(logger::telemeterize);
  }

  public static Command pathFindAndAlignCommand(Supplier<Integer> val) {
    return Commands.sequence(
        Commands.either(
                m_drive
                    .goToPose(FieldConstants.redSidePositions[val.get()])
                    .until(
                        () ->
                            m_drive
                                    .getState()
                                    .Pose
                                    .getTranslation()
                                    .getDistance(FieldConstants.redSidePositions[val.get()].getTranslation())
                                <= 2),
                m_drive
                    .goToPose(FieldConstants.blueSidePositions[val.get()])
                    .until(
                        () ->
                            m_drive
                                    .getState()
                                    .Pose
                                    .getTranslation()
                                    .getDistance(FieldConstants.blueSidePositions[val.get()].getTranslation())
                                <= 2),
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
                              ? FieldConstants.redSidePositions[val.get()]
                              : FieldConstants.blueSidePositions[val.get()];
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

  private void enableNamedCommands(){
    NamedCommands.registerCommand("ElevatorL4", m_elevator.goToPosition(1.67));
    NamedCommands.registerCommand("OutakeReef", new LeaveReefCommand(m_intake, m_elevator));
  } 

  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          chassisDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          chassisDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
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
