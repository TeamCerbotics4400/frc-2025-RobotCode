// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands.SubsystemCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class LeaveReefCommand extends Command {

  IntakeSubsystem m_intake;
  ElevatorSubsystem m_elevator;
  Timer timer = new Timer();

  public LeaveReefCommand(IntakeSubsystem m_intake, ElevatorSubsystem m_elevator) {

    this.m_intake = m_intake;
    this.m_elevator = m_elevator;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //1100
    if(m_elevator.isInPosition()){
      timer.start();
    }

    if(timer.hasElapsed(0.3) && timer.isRunning()){
      m_intake.setVoltageVoid(0.3, 0.3);
      timer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setVoltageVoid(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_intake.hasGamePieceInside() || m_elevator.getPosition() < 0.1;
  }
}
