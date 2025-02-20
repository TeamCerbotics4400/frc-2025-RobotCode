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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.Commands.AutoCommands.SubsystemCommands.LeaveReefCommand;
import frc.robot.Commands.ElevatorCommands.ElevatorAutoCommand;
import frc.robot.Commands.IntakeCommand.IntakeSequenceCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Climber.ClimberIO;
import frc.robot.Subsystems.Climber.ClimberIOKraken;
import frc.robot.Subsystems.Climber.ClimberIOSparkMax;
import frc.robot.Subsystems.Climber.ClimberSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOKraken;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOKraken;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeIO;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeIOKraken;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Swerve.TunerConstants;
import frc.robot.Subsystems.Vision.VisionSubsystem;

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
  public static final ClimberIO climberIO = new ClimberIOSparkMax();
  public static ClimberSubsystem m_climber;

  /*IntakeAlgae */
  public static final IntakeAlgaeIO intakeAlgaeIO = new IntakeAlgaeIOKraken();
  public static IntakeAlgaeSubsystem m_algae;

  /* Vision */
  public static VisionSubsystem m_vision = new VisionSubsystem(m_drive, VisionConstants.tagLimelightName);

  /* Chooser for autonomous */
  private final SendableChooser<AutoCommand> autoChooser = new SendableChooser<>();

  /* Set up for utils */
  public static Field2d autoFieldPreview = new Field2d();
  public static CustomDashboardUtil m_dashboard = new CustomDashboardUtil();
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  public RobotContainer() {

    /* IO options for replay */
    switch (Constants.currentMode) {
      /* Configs for the REAL robot */
      case REAL:
        m_elevator = new ElevatorSubsystem(elevatorIO);
        m_intake = new IntakeSubsystem(intakeIO);
        m_climber = new ClimberSubsystem(climberIO);
        m_algae = new IntakeAlgaeSubsystem(intakeAlgaeIO);
        break;
      /* Configs to replay a log */
      case REPLAY:
        m_elevator = new ElevatorSubsystem(new ElevatorIO(){});
        m_intake = new IntakeSubsystem(new IntakeIO(){});
        m_climber = new ClimberSubsystem(new ClimberIO(){});
        m_algae = new IntakeAlgaeSubsystem(new IntakeAlgaeIO(){});
      break;
      /* Default to just in case it somehow fails, lol */
      default:
        m_elevator = new ElevatorSubsystem(elevatorIO);
        m_intake = new IntakeSubsystem(intakeIO);
        m_climber = new ClimberSubsystem(climberIO);       
        m_algae = new IntakeAlgaeSubsystem(intakeAlgaeIO);
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

  /* Chassis commands */
    m_drive.setDefaultCommand(
      new FieldCentricDrive(
          m_drive,
           ()-> -chassisDriver.getLeftY(),
           ()-> -chassisDriver.getLeftX(), 
           ()-> chassisDriver.getRightX()));


  chassisDriver.start().onTrue(
      new ParallelRaceGroup(
          pathFindAndAlignCommand(()->m_dashboard.getReefSelected()),
           new SequentialCommandGroup(new WaitCommand(1),
            new WaitCommand(10000).until(()->isJoystickActive()))));


  chassisDriver.back().onTrue(m_drive.runOnce(() -> m_drive.seedFieldCentric()).ignoringDisable(true));

  m_drive.registerTelemetry(logger::telemeterize);

    /* Elevator Commands */
  
  //subsystemsDriver.a().onTrue(m_elevator.goToPosition(.15));
  chassisDriver.b().onTrue(m_elevator.goToPosition(.4));
  chassisDriver.x().onTrue(m_elevator.goToPosition(.91));
  chassisDriver.y().onTrue(m_elevator.goToPosition(1.74));
  
  chassisDriver.a().onTrue(m_elevator.goToPosition(0.0));


    /* Climber Commands */
   chassisDriver.povUp().whileTrue(m_climber.setClimberVoltage(0.3)).whileFalse(m_climber.setClimberVoltage(0));

  chassisDriver.povDown().whileTrue(m_climber.setClimberVoltage(-0.5)).whileFalse(m_climber.setClimberVoltage(0));

  /* Intake Commands */
  chassisDriver.rightBumper().onTrue(new IntakeSequenceCommand(m_intake));

    /*IntakeAlgae Commands */
    //va asia abajo
  /*   chassisDriver.povUp().onTrue(m_algae.setVoltageCommandPiv(.1)).whileFalse(m_algae.setVoltageCommandPiv(0));

    //va acia el escalador
    chassisDriver.povDown().onTrue(m_algae.setVoltageCommandPiv(-0.1)).whileFalse(m_algae.setVoltageCommandPiv(0));

    //mete
    chassisDriver.povLeft().onTrue(m_algae.setVoltageCommandRoll(.3)).whileFalse(m_algae.setVoltageCommandRoll(0));
    //saca
    chassisDriver.povRight().onTrue(m_algae.setVoltageCommandRoll(-.3)).whileFalse(m_algae.setVoltageCommandRoll(0));

*/
    chassisDriver.leftBumper().whileTrue(
      new ConditionalCommand(
        m_intake.setVoltageCommand(0.15,0.35),
        m_intake.setVoltageCommand(0.3, 0.3), 
      ()-> m_elevator.getPosition() < 0.3))
      .whileFalse(m_intake.setVoltageCommand(0, 0));

  }

    public static Command pathFindAndAlignCommand(Supplier<Integer> val) {
      return Commands.sequence(
          new DeferredCommand(
              () -> Commands.either(
                  m_drive
                      .goToPose(() -> FieldConstants.blueCenterPosition[val.get()])   // RED VAL
                      .until(() -> 
                          m_drive.getState().Pose.getTranslation()
                              .getDistance(FieldConstants.redSidePositions[val.get()].getTranslation()) <= 0.05
                      ),
                  m_drive
                      .goToPose(() -> FieldConstants.blueCenterPosition[val.get()])
                      .until(() -> 
                          m_drive.getState().Pose.getTranslation()
                              .getDistance(FieldConstants.blueSidePositions[val.get()].getTranslation()) <= 0.05
                      ),
                  Robot::isRedAlliance
              ),
              Set.of(m_drive)
          )
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
                                  2.0,
                                  Units.degreesToRadians(360),
                                  Units.degreesToRadians(360)),
                              null,
                              new GoalEndState(0.0, targetPose.getRotation()));
                      path.preventFlipping = true;
  
                      return AutoBuilder.followPath(path);
                  },
                  Set.of(m_drive)
              )
          )
      );
    
  }

  private void enableNamedCommands(){
    NamedCommands.registerCommand("ElevatorL4", new ElevatorAutoCommand(m_elevator, 1.67, m_intake));
    NamedCommands.registerCommand("ElevatorL0", m_elevator.goToPosition( 0.0));
    NamedCommands.registerCommand("OutakeReef", new LeaveReefCommand(m_intake, m_elevator));
    NamedCommands.registerCommand("IntakeCoral", new IntakeSequenceCommand(m_intake));
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

  public static ElevatorSubsystem getElevatorSubsystem(){
    return m_elevator;
  }

  public static CustomDashboardUtil getDashboardUtil(){
    return m_dashboard;
  }

  public static CommandSwerveDrivetrain getSwerve(){
    return m_drive;
  }
}
