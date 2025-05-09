// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.IntakeCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeSequence3 extends Command {
  /** Creates a new IntakeCommand. */
  IntakeSubsystem m_intake;
  Timer timer = new Timer();
  boolean safePiece = false;
  public IntakeSequence3(IntakeSubsystem m_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intake = m_intake;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.finishedIntakeSequence = false;
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //1100
    m_intake.setVoltageVoid(0.1, 0.1);

    if(m_intake.hasGamePieceInside()){
      timer.start();
    }

    if(timer.hasElapsed(0.38) && timer.isRunning()){
      safePiece = true;
      timer.stop();
    }
    else{
      safePiece = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setVoltageVoid(0,0);
    if(m_intake.hasGamePieceInside()){
    m_intake.finishedIntakeSequence = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return safePiece;
  }
}
