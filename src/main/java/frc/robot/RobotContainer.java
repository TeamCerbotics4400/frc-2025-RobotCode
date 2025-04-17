// Copyright (c) 2025 FRC 4400//
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.CustomDashboardUtil;
import frc.Util.LocalADStarAK;
import frc.robot.Commands.DoNothingCommandCommand;
import frc.robot.Commands.AlgaeIntakeCommand.PriorityOutakeCommand;
import frc.robot.Commands.AutoCommands.AutoCommand;
import frc.robot.Commands.AutoCommands.Paths.NoneAuto;
import frc.robot.Commands.AutoCommands.Paths.WorkShopPaths.LeaveAuto;
import frc.robot.Commands.AutoCommands.Paths.WorkShopPaths.Left1CoralAuto;
import frc.robot.Commands.AutoCommands.Paths.WorkShopPaths.Left3CoralAuto;
import frc.robot.Commands.AutoCommands.Paths.WorkShopPaths.Left4CoralAuto;
import frc.robot.Commands.AutoCommands.Paths.WorkShopPaths.Right1CoralAuto;
import frc.robot.Commands.AutoCommands.Paths.WorkShopPaths.Right3CoralAuto;
import frc.robot.Commands.AutoCommands.SubsystemCommands.LeaveReefCommand;
import frc.robot.Commands.ClimberCommand.ClimberSequence;
import frc.robot.Commands.ElevatorCommands.ElevatorAutoCommand;
import frc.robot.Commands.ElevatorCommands.Level1CycleCommand;
import frc.robot.Commands.IntakeCommand.IntakeSequence2;
import frc.robot.Commands.IntakeCommand.IntakeSequence3;
import frc.robot.Commands.SwerveCommands.FieldCentricDrive;
import frc.robot.Commands.SwerveCommands.SwerveAutoAlignPose;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OuttakeState;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Climber.ClimberIO;
import frc.robot.Subsystems.Climber.ClimberIOSparkMax;
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
  public static VisionSubsystem m_vision = new VisionSubsystem(m_drive);

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
    autoChooser.addOption("Left Side 4 Coral", new Left4CoralAuto());
    autoChooser.addOption("Left Side 3 Coral", new Left3CoralAuto());
    autoChooser.addOption("Left 1 Coral + 2 Algae", new Left1CoralAuto());
    autoChooser.addOption("Right Side 3 Coral", new Right3CoralAuto());
    autoChooser.addOption("Right Side 1 Coral", new Right1CoralAuto());
    autoChooser.addOption("LEAVE NOTHING ELSE", new LeaveAuto());

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
    FollowPathCommand.warmupCommand().schedule();
  
    /* Shows the preview before match setup */
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("Auto Preview", autoFieldPreview);

    configureBindings();
  }


  private void configureBindings() {

    /*__________________ Chassis commands __________________*/
  
    // Drive Swerve Command
    m_drive.setDefaultCommand(
      new FieldCentricDrive(
        m_drive,
        () -> -chassisDriver.getLeftY(),
        () -> -chassisDriver.getLeftX(), 
        () -> chassisDriver.getRightX()
      )
    );
  
    // Auto Align Command
    chassisDriver.start().onTrue(
      new ParallelRaceGroup(
        pathFindAndAlignCommand(() -> m_dashboard.getReefSelected()),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new WaitCommand(10000).until(() -> isJoystickActive())
        )
      )
    );
  
    // Reset Field Centric (usable while disabled)
    chassisDriver.back().onTrue(
      m_drive.runOnce(() -> 
        m_drive.resetRotation(new Rotation2d(
          Robot.isRedAlliance() ? Math.PI : 0
        ))
      ).ignoringDisable(true)
    );
  
    // Logging telemetry
    m_drive.registerTelemetry(logger::telemeterize);
  
    /*__________________ Elevator Commands __________________*/
  
    // Level 1
    chassisDriver.povDown().onTrue(new Level1CycleCommand(m_elevator, m_intake));
  
    // Level 2
    chassisDriver.b().onTrue(
      new ConditionalCommand(
        m_elevator.goToPosition(0.48)
          .onlyIf(() -> m_intake.finishedIntakeSequence),
        m_elevator.goToPosition(0.57),
        () -> m_algae.getState() != AlgaeState.ACTIVEPOSITION
      )
    );
  
    // Level 3
    chassisDriver.x().onTrue(
      new ConditionalCommand(
        m_elevator.goToPosition(0.94)
          .onlyIf(() -> m_intake.finishedIntakeSequence),
        m_elevator.goToPosition(0.57),
        () -> m_algae.getState() != AlgaeState.ACTIVEPOSITION
      )
    );
  
    // Level 4
    chassisDriver.y().onTrue(
      m_elevator.goToPosition(1.73)
        .onlyIf(() -> m_intake.finishedIntakeSequence)
    );
  
    // Reset Elevator
    chassisDriver.a().onTrue(
      m_elevator.goToPosition(0.0)
    );
  
    /*__________________ Climber Commands __________________*/
  
    m_climber.setDefaultCommand(
      climberIpadCommand(() -> m_dashboard.getLevelEntry())
    );
  
    /*__________________ End Effector Commands __________________*/
  
    // Intake in and out sequence
    chassisDriver.rightBumper().onTrue(
      new IntakeSequence3(m_intake)
    );
  
    // Outtake coral depending on elevator level
    /*chassisDriver.leftBumper()
      .onTrue(
        new ConditionalCommand(
          new IntakeSequence2(m_intake),
          m_intake.setVoltageCommand(0.4, 0.4),
          () -> m_elevator.getPosition() < 0.36
        )
      )
      .whileFalse(
        new InstantCommand(() -> 
          m_intake.changeState(IntakeState.FINISHED)
        ).andThen(
          m_intake.setVoltageCommand(0, 0)
        )
      );*/
  
    /*__________________ Algae Commands __________________*/
  
    // Left Trigger - Algae to position 10
    chassisDriver.leftTrigger()
      .whileTrue(
        m_algae.goToPosition(9.5, AlgaeState.ACTIVEPOSITION)
          .andThen(m_algae.setVoltageCommandRoll(0.83))
      )
      .whileFalse(
        m_algae.goToPosition(0.0, AlgaeState.BACKPOSITION)
          .andThen(m_algae.setVoltageCommandRoll(0.83))
      );
  
    // Right Trigger - Algae to position 2
    chassisDriver.rightTrigger()
      .whileTrue(
        m_algae.goToPosition(2.5, AlgaeState.ACTIVEPOSITION)
          .andThen(m_algae.setVoltageCommandRoll(0.83))
      )
      .whileFalse(
        m_algae.goToPosition(0.0, AlgaeState.BACKPOSITION)
          .andThen(m_algae.setVoltageCommandRoll(0.83))
      );

  
    /*__________________ Climber Manual Commands __________________*/
  
    // POV Up - Climber up
    chassisDriver.povUp()
      .whileTrue(new ClimberSequence(m_climber, m_algae));
  
    // POV Left - Climber down
    chassisDriver.povLeft()
      .whileTrue(m_climber.setNeoVoltage(-1))
      .whileFalse(m_climber.setNeoVoltage(0));
  
    // POV Right - Climber set position
    subsystemsDriver.povRight().onTrue(
      m_climber.setNeoPosition(-196)
    );

    chassisDriver.povRight().whileTrue(m_algae.goToPosition(7.0, AlgaeState.ACTIVEPOSITION));
  
    /*__________________ BACKUP CONTROLLER __________________*/
  
      chassisDriver.leftBumper().onTrue(
        new ConditionalCommand(

          new ConditionalCommand( 
          m_intake.setVoltageCommand(0.4, 0.4),
          m_intake.setVoltageCommand(0.35, 0.35),
          ()-> m_elevator.getPosition() < 1.70
          ), 

          m_algae.setVoltageCommandRoll(-1), 

          ()-> (Constants.outtakeState == OuttakeState.CORAL_PRIORITY
          && m_intake.hasGamePieceInside()) || !m_algae.isGamePieceInside())
      ).whileFalse(m_intake.setVoltageCommand(0, 0).alongWith(
        m_algae.goToPosition(0.1, AlgaeState.BACKPOSITION).andThen(
        m_algae.setVoltageCommandRoll(0))
      .onlyIf(()-> !m_algae.isGamePieceInside())));


      subsystemsDriver.a().whileTrue(
        m_elevator.safeReset(-0.1))
        .whileFalse(m_elevator.setManualVoltage(0));

  }
  

public static Command climberIpadCommand(Supplier<Integer> val) {
    return new InstantCommand(() -> {
        Command selectedCommand;
        
        switch (val.get()) {
          
            case 1:
                selectedCommand = m_climber.setNeoPosition(-196.0).unless(()-> m_climber.climbState == ClimbingState.CLIMBING); //Step 2
                break;
            
            default:
                selectedCommand = new DoNothingCommandCommand(); //End
                break;
        }
        if(val.get() == 3){
       //   Constants.outtakeState = OuttakeState.CORAL_PRIORITY;
        }
        if(val.get() == 2){
          Constants.outtakeState = OuttakeState.ALGAE_PRIORITY;
        }
        selectedCommand.schedule();
    }, m_climber);
}


    public static Command pathFindAndAlignCommand(Supplier<Integer> val) {
      return Commands.sequence(
          new DeferredCommand(
              () -> Commands.either(
                  m_drive
                      .goToPose(() -> FieldConstants.alignRedPose[val.get()])   // RED VAL
                      .until(() -> 
                          m_drive.getState().Pose.getTranslation()
                              .getDistance(FieldConstants.alignRedPose[val.get()].getTranslation()) <= 0.2
                      ),
                  m_drive
                      .goToPose(() -> FieldConstants.alignBluePose[val.get()])
                      .until(() -> 
                          m_drive.getState().Pose.getTranslation()
                              .getDistance(FieldConstants.alignBluePose[val.get()].getTranslation()) <= 0.2
                      ),
                  Robot::isRedAlliance
              ),
              Set.of(m_drive)
          )
          .andThen(
            Commands.defer(
              () -> new SwerveAutoAlignPose(
                  () -> FieldConstants.redSidePositions[val.get()],
                  () -> FieldConstants.blueSidePositions[val.get()],
                  m_drive
              ),
              Set.of(m_drive)
            )
          )
      );
  }

  private void enableNamedCommands(){
  NamedCommands.registerCommand("ElevatorL4", 
    new ElevatorAutoCommand(m_elevator, 1.73, m_intake, 1.72));

  NamedCommands.registerCommand("ElevatorL4Backup", 
    new ElevatorAutoCommand(m_elevator, 1.71, m_intake, 1.70));

  NamedCommands.registerCommand("ElevatorL0", 
    m_elevator.goToPosition(0.0));

  NamedCommands.registerCommand("OutakeReef", 
    new LeaveReefCommand(m_intake, m_elevator));

  NamedCommands.registerCommand("IntakeCoral", 
    new IntakeSequence3(m_intake));

  NamedCommands.registerCommand("SafeFailElevator", 
    new ElevatorAutoCommand(m_elevator, 1.73, m_intake, 1.72)); 

  NamedCommands.registerCommand("SafeFailElevatorBackup", 
    new ElevatorAutoCommand(m_elevator, 1.71, m_intake, 1.70)); 

  NamedCommands.registerCommand("AlgaeElevatorPos", 
    m_elevator.goToPosition(0.20));

  NamedCommands.registerCommand("AlgaeIntake", 
    m_algae.goToPosition(2, AlgaeState.BACKPOSITION)
           .andThen(m_algae.setVoltageCommandRoll(0.83)));

  NamedCommands.registerCommand("PrepareAlgae", 
    m_algae.goToPosition(0, AlgaeState.BACKPOSITION));

  NamedCommands.registerCommand("OutakeAlgae", 
    m_algae.setVoltageCommandRoll(-0.83)
           .until(() -> m_algae.getRollerCurrent() < 30));

  NamedCommands.registerCommand("ElevatorL4NoSafe", 
    m_elevator.goToPosition(1.73));

  NamedCommands.registerCommand("HighAlgaeElevator", 
    m_elevator.goToPosition(0.76));

    NamedCommands.registerCommand("ElevatorAlgaePose", 
    m_elevator.goToPosition(1.758));   
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
   //return new FieldCentricDrive(m_drive,()->0.4, ()->0.0, ()->0.0);
  }

  public static IntakeSubsystem getIntakeSubsystem(){
    return m_intake;
  }

  public static ElevatorSubsystem getElevatorSubsystem(){
    return m_elevator;
  }

  public static IntakeAlgaeSubsystem getAlgaeSubsystem(){
    return m_algae;
  }

  public static CustomDashboardUtil getDashboardUtil(){
    return m_dashboard;
  }

  public static ClimberSubsystem getClimberSubsystem(){
    return m_climber;
  }

  public static CommandSwerveDrivetrain getSwerve(){
    return m_drive;
  }
}
