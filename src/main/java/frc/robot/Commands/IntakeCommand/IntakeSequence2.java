// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.IntakeCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeSequence2 extends Command {
  /** Creates a new IntakeCommand. */
  IntakeSubsystem m_intake;
  Timer timer = new Timer();
  public IntakeSequence2(IntakeSubsystem m_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intake = m_intake;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //1100

    //if(timer.hasElapsed(0.1) && timer.isRunning()){    0.3    0.7    0.1     0.5
      m_intake.setVoltageVoid(0.1, 0.5);
    //}
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setVoltageVoid(0,0);
    timer.stop();
  }
}
