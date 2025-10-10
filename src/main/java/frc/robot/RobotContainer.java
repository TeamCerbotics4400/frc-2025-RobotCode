// Copyright (c) 2025 FRC 4400//
package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.CustomDashboardUtil;
import frc.robot.Commands.DoNothingCommandCommand;
import frc.robot.Commands.ElevatorCommands.Level1CycleCommand;
import frc.robot.Commands.IntakeCommand.IntakeSequence3;
import frc.robot.Commands.SwerveCommands.FieldCentricDrive;
import frc.robot.Constants.OuttakeState;
import frc.robot.Subsystems.Climber.ClimberIO;
import frc.robot.Subsystems.Climber.ClimberIOkraken;
import frc.robot.Subsystems.Climber.ClimberSubsystem;
import frc.robot.Subsystems.Climber.ClimberSubsystem.ClimbingState;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOKraken;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOKraken;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeIO;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeIOKraken;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeSubsystem.AlgaeState;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Swerve.TunerConstants;
import frc.robot.Subsystems.Vision.VisionSubsystem;

import java.util.function.Supplier;

public class RobotContainer {

  /* Driver controllers */
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
  public static final ClimberIO climberIO = new ClimberIOkraken();
  public static ClimberSubsystem m_climber;

  /* IntakeAlgae */
  public static final IntakeAlgaeIO intakeAlgaeIO = new IntakeAlgaeIOKraken();
  public static IntakeAlgaeSubsystem m_algae;

  /* Vision */
  public static VisionSubsystem m_vision = new VisionSubsystem(m_drive);

  /* Set up for utils */
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
        m_elevator = new ElevatorSubsystem(new ElevatorIO() {
        });
        m_intake = new IntakeSubsystem(new IntakeIO() {
        });
        m_climber = new ClimberSubsystem(new ClimberIO() {
        });
        m_algae = new IntakeAlgaeSubsystem(new IntakeAlgaeIO() {
        });
        break;
      /* Default to just in case it somehow fails, lol */
      default:
        m_elevator = new ElevatorSubsystem(elevatorIO);
        m_intake = new IntakeSubsystem(intakeIO);
        m_climber = new ClimberSubsystem(climberIO);
        m_algae = new IntakeAlgaeSubsystem(intakeAlgaeIO);
        break;
    }

    configureBindings();
  }

  private void configureBindings() {

    /* __________________ Chassis commands __________________ */

    // Drive Swerve Command
    m_drive.setDefaultCommand(
        new FieldCentricDrive(
            m_drive,
            () -> -chassisDriver.getLeftY(),
            () -> -chassisDriver.getLeftX(),
            () -> chassisDriver.getRightX()));

    // Reset Field Centric (usable while disabled)
    chassisDriver.back().onTrue(
        m_drive.runOnce(() -> m_drive.resetRotation(new Rotation2d(
            Robot.isRedAlliance() ? Math.PI : 0))).ignoringDisable(true));

    // Logging telemetry
    m_drive.registerTelemetry(logger::telemeterize);

    /* __________________ Elevator Commands __________________ */

    // Level 1
    chassisDriver.povDown().onTrue(new Level1CycleCommand(m_elevator, m_intake));

    // Level 2
    chassisDriver.b().onTrue(
        new ConditionalCommand(
            m_elevator.goToPosition(0.48)
                .onlyIf(() -> m_intake.finishedIntakeSequence),
            m_elevator.goToPosition(0.57),
            () -> m_algae.getState() != AlgaeState.FLOORPOSITION));

    // Level 3
    chassisDriver.x().onTrue(
        new ConditionalCommand(
            m_elevator.goToPosition(0.94)
                .onlyIf(() -> m_intake.finishedIntakeSequence),
            m_elevator.goToPosition(0.57),
            () -> m_algae.getState() != AlgaeState.FLOORPOSITION));

    // Level 4
    chassisDriver.y().onTrue(
        m_elevator.goToPosition(1.73)
            .onlyIf(() -> m_intake.finishedIntakeSequence));

    // Reset Elevator
    chassisDriver.a().onTrue(
        m_elevator.goToPosition(0.0));

    /* __________________ Climber Commands __________________ */

    m_climber.setDefaultCommand(
        climberIpadCommand(() -> m_dashboard.getLevelEntry()));

    /* __________________ End Effector Commands __________________ */

    // Intake in and out sequence
    chassisDriver.rightBumper().onTrue(
        new IntakeSequence3(m_intake));

    // Outtake coral depending on elevator level
    /*
     * chassisDriver.leftBumper()
     * .onTrue(
     * new ConditionalCommand(
     * new IntakeSequence2(m_intake),
     * m_intake.setVoltageCommand(0.4, 0.4),
     * () -> m_elevator.getPosition() < 0.36
     * )
     * )
     * .whileFalse(
     * new InstantCommand(() ->
     * m_intake.changeState(IntakeState.FINISHED)
     * ).andThen(
     * m_intake.setVoltageCommand(0, 0)
     * )
     * );
     */

    /* __________________ Algae Commands __________________ */

    // Left Trigger - Algae to position 10
    chassisDriver.leftTrigger()
        .whileTrue(
            m_algae.goToPosition(10, AlgaeState.FLOORPOSITION)
                .andThen(m_algae.setVoltageCommandRoll(0.83)))
        .whileFalse(
            m_algae.goToPosition(0.0, AlgaeState.BACKPOSITION)
                .andThen(m_algae.setVoltageCommandRoll(0.83)));

    // Right Trigger - Algae to position 2
    chassisDriver.rightTrigger()
        .whileTrue(
            m_algae.goToPosition(3.3, AlgaeState.REEFPOSITION)
                .andThen(m_algae.setVoltageCommandRoll(0.83)))
        .whileFalse(
            m_algae.goToPosition(0.0, AlgaeState.BACKPOSITION)
                .andThen(m_algae.setVoltageCommandRoll(0.83)));

    /* __________________ Climber Manual Commands __________________ */

    /* Mover el escalador por voltage */

    subsystemsDriver.y()
        .onTrue(m_climber.goToPosition(195.0));

    subsystemsDriver.x()
        .onTrue(m_climber.goToPosition(1));

    subsystemsDriver.povUp()
        .whileTrue(m_climber.setKrakenVoltage(8))
        .whileFalse(m_climber.setKrakenVoltage(0));

    subsystemsDriver.povDown()
        .whileTrue(m_climber.setKrakenVoltage(-8))
        .whileFalse(m_climber.setKrakenVoltage(0));

    // POV Right - Climber set position

    chassisDriver.povRight().whileTrue(m_algae.goToPosition(7.0, AlgaeState.FLOORPOSITION));

    /* __________________ BACKUP CONTROLLER __________________ */

    chassisDriver.leftBumper().onTrue(
        new ConditionalCommand(

            new ConditionalCommand(
                m_intake.setVoltageCommand(0.4, 0.4),
                m_intake.setVoltageCommand(0.35, 0.35),
                () -> m_elevator.getPosition() > 1.70),

            m_algae.setVoltageCommandRoll(-1),

            () -> (Constants.outtakeState == OuttakeState.CORAL_PRIORITY
                && m_intake.hasGamePieceInside()) || !m_algae.isGamePieceInside()))
        .whileFalse(m_intake.setVoltageCommand(0, 0).alongWith(
            m_algae.goToPosition(0.1, AlgaeState.BACKPOSITION).andThen(
                m_algae.setVoltageCommandRoll(0))
                .onlyIf(() -> !m_algae.isGamePieceInside())));

  }

  public static Command climberIpadCommand(Supplier<Integer> val) {
    return new InstantCommand(() -> {
      Command selectedCommand;

      switch (val.get()) {

        case 1:
          selectedCommand = m_climber.goToPosition(-196.0)
              .unless(() -> m_climber.climbState == ClimbingState.CLIMBING); // Step 2
          break;

        default:
          selectedCommand = new DoNothingCommandCommand(); // End
          break;
      }
      if (val.get() == 3) {
        // Constants.outtakeState = OuttakeState.CORAL_PRIORITY;
      }
      if (val.get() == 2) {
        Constants.outtakeState = OuttakeState.ALGAE_PRIORITY;
      }
      selectedCommand.schedule();
    }, m_climber);
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
    return null;
    // return new FieldCentricDrive(m_drive,()->0.4, ()->0.0, ()->0.0);
  }

  public static IntakeSubsystem getIntakeSubsystem() {
    return m_intake;
  }

  public static ElevatorSubsystem getElevatorSubsystem() {
    return m_elevator;
  }

  public static IntakeAlgaeSubsystem getAlgaeSubsystem() {
    return m_algae;
  }

  public static CustomDashboardUtil getDashboardUtil() {
    return m_dashboard;
  }

  public static ClimberSubsystem getClimberSubsystem() {
    return m_climber;
  }

  public static CommandSwerveDrivetrain getSwerve() {
    return m_drive;
  }
}
