// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
  private boolean enablePID = false;

    /*Set the Maximun velocity and acceleration, needs to be tuned according to your robot*/
    private final TrapezoidProfile.Constraints m_profile = new TrapezoidProfile.Constraints(1.75, 0.75);

    /*Main PID Controller using the constrains as reference */
    private final ProfiledPIDController m_controller = new ProfiledPIDController(0, 0, 0, m_profile);

    /*FeedForward Model for extra presicion */
    private final ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(0, 0, 0);


  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if(enablePID){
        io.setVoltage(
          m_controller.calculate(inputs.elevatorPosition),
           m_ElevatorFeedforward.calculate(m_controller.getSetpoint().position, m_controller.getSetpoint().velocity));
    }
  }

  public ProfiledPIDController getController(){
    return m_controller;
  }

  public Command goToPosition(double position) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
              getController().reset(inputs.elevatorPosition);
              m_controller.setGoal(position);
              enablePID = true;
            },
            this);
    return ejecutable;
  }

  public Command setVoltage(double volts){
    return run (()-> io.setVoltage(volts, 0));
  }
}
