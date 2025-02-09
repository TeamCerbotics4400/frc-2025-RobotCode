// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Util.CustomDashboardUtil;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorAutoCommand extends Command {

  private ElevatorSubsystem m_elevator;
  private IntakeSubsystem m_intake;
  private double position = 0.0;
  public ElevatorAutoCommand(ElevatorSubsystem m_elevator, double position, IntakeSubsystem m_intake) {
    this.m_elevator = m_elevator;
    this.m_intake = m_intake;
    this.position = position;

    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.resetController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intake.finishedIntakeSequence){
    m_elevator.goToPositionVoid(position);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
