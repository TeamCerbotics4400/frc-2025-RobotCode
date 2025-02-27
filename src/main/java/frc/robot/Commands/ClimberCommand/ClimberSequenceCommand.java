// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ClimberCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Climber.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberSequenceCommand extends SequentialCommandGroup {
  ClimberSubsystem m_climber;
  public ClimberSequenceCommand(ClimberSubsystem m_climber) {
    this.m_climber = m_climber;

    addCommands(
      m_climber.setKrakenPosition(-3.67),
      new WaitCommand(0.3),
      m_climber.setNeoVoltage(1)
    );
  }
}
