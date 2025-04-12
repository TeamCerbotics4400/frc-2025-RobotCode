// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AlgaeIntakeCommand;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeSubsystem;

public class PriorityOutakeCommand extends Command {

  private IntakeAlgaeSubsystem m_algae;
  private IntakeSubsystem m_intake;
  private Supplier<Integer> val;
  private boolean shouldFinish = false;

  private enum OuttakeState {
    CORAL_PRIORITY,
    ALGAE_PRIORITY
}

private OuttakeState outtakeState = OuttakeState.ALGAE_PRIORITY;

  public PriorityOutakeCommand(IntakeAlgaeSubsystem m_algae, IntakeSubsystem m_intake, Supplier<Integer> val) {
    this.m_algae = m_algae;
    this.m_intake = m_intake;
    this.val = val;

    addRequirements(m_algae,m_intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(val.get() == 3){
      outtakeState = OuttakeState.CORAL_PRIORITY;
    }
    if(val.get() == 2){
      outtakeState = OuttakeState.ALGAE_PRIORITY;
    }

    if(outtakeState == OuttakeState.CORAL_PRIORITY && m_intake.hasGamePieceInside()){
      m_intake.setVoltageVoid(0.4, 0.4);
      outtakeState = OuttakeState.ALGAE_PRIORITY;
      shouldFinish = true;
    }else if(m_algae.isGamePieceInside()){
      m_algae.setVoltageRollerVoid(-1);
      outtakeState = OuttakeState.CORAL_PRIORITY;
      shouldFinish = true;
    }

  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setVoltageVoid(0.0, 0.0);
    m_algae.setVoltageRollerVoid(0);
  }

  @Override
  public boolean isFinished() {
    return shouldFinish;
  }
}
