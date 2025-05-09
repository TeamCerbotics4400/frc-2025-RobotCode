// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.IntakeCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSequenceCommand extends SequentialCommandGroup {
  IntakeSubsystem m_intake;
  public IntakeSequenceCommand(IntakeSubsystem m_intake) {
    this.m_intake = m_intake;

    addCommands(
      new IntakeSequence1(m_intake)
     // new IntakeSequence2(m_intake)
      //new IntakeSequence3(m_intake)
    );
  }
}
