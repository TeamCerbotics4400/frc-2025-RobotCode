// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AlgaeIntakeCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeSubsystem;

public class AlgaeIntakeIntakeCommand extends Command {

  private IntakeAlgaeSubsystem m_algae;
  private Timer timer = new Timer();

  public AlgaeIntakeIntakeCommand(IntakeAlgaeSubsystem m_algae) {

    this.m_algae = m_algae;

    addRequirements(m_algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //1100
    m_algae.setVoltageRollerVoid(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algae.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
