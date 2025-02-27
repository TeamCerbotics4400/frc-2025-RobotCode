// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AlgaeIntakeCommand;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Util.CustomDashboardUtil;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgaeCommand extends Command {

  private IntakeAlgaeSubsystem m_algae;
  private CustomDashboardUtil m_util;
  private int val = 0;

  public IntakeAlgaeCommand(IntakeAlgaeSubsystem m_algae, CustomDashboardUtil m_util) {
    this.m_algae = m_algae;
    this.m_util = m_util;

    val = m_util.getLevelEntry();

    addRequirements(m_algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_algae.resetController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(val != m_util.getLevelEntry()){
      m_algae.resetController();
      val = m_util.getLevelEntry();
    }

    if(val == 3){
      m_algae.setVoltageRollerVoid(-1);
    }
    else if(val == 2){
      m_algae.setVoltageRollerVoid(1);
    }
    else if(val == 0 || val == 1){
      m_algae.setVoltageRollerVoid(0);
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