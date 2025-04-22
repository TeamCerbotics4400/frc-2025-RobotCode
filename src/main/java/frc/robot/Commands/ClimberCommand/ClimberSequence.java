// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ClimberCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber.ClimberSubsystem;
import frc.robot.Subsystems.Climber.ClimberSubsystem.ClimbingState;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeSubsystem.AlgaeState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberSequence extends Command {
  /** Creates a new ClimberSequence. */
  private ClimberSubsystem m_climber;
  private IntakeAlgaeSubsystem m_algae;
  public ClimberSequence(ClimberSubsystem m_climber, IntakeAlgaeSubsystem m_algae) {
    this.m_algae = m_algae;
    this.m_climber = m_climber;

    addRequirements(m_algae, m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_climber.getSparkMaxPosition() < -190){
      m_climber.climbState = ClimbingState.CLIMBING;
    }

    m_climber.setNeoVoidVoltage(1);
    if(m_climber.getSparkMaxPosition() > -180 && m_climber.climbState == ClimbingState.CLIMBING){
    m_algae.goToPositionVoid(10);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setNeoVoidVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
