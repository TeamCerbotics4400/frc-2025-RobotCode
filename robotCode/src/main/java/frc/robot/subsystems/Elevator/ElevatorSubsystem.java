// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

/* Clase para el simulador de un elevador */
  private final ElevatorSim elevatorSimulator =
    new ElevatorSim(
      0.1, 
      0.1,
         DCMotor.getKrakenX60Foc(2), 
         0.0, 
     1.4,
      false, 
      0.0);

  /* Clase para visualizar el elevador con el robot */    
  private final ElevatorVisualizer m_visualizerPosition = new ElevatorVisualizer("Elevator Position",Color.kBlack);


  /* Configuraciones de PID */
  private TrapezoidProfile.Constraints m_profile = new TrapezoidProfile.Constraints(5, 7);
  public ProfiledPIDController m_controller = new ProfiledPIDController(1.5, 0, 0, m_profile);    

  /* Medida de seguridad para desabilitar el movimiento del elevador */
  public boolean enablePID = false;

  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    /* Siempre poner esto */
    elevatorSimulator.update(0.02);


    /* Loggeo de datos para su analisis */
    Logger.recordOutput("Elevator/Position", elevatorSimulator.getPositionMeters());
    Logger.recordOutput("Elevator/Setpoint", m_controller.getSetpoint().position);
    Logger.recordOutput("Elevator/PID output", m_controller.calculate(elevatorSimulator.getPositionMeters()));

    /* Simula ponerle voltaje al elevador para que este se mueva (Funciona igual a un setPower(1)) */
    if(enablePID){
    elevatorSimulator.setInput(m_controller.calculate(elevatorSimulator.getPositionMeters()));
    }
    /* Actualiza el visualizador */
    m_visualizerPosition.update(elevatorSimulator.getPositionMeters());
  }


  public ProfiledPIDController getController(){
    return m_controller;
  }


  /* Comando para indicarle la posicion al elevador */
    public Command goToPosition(Double position) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
              getController().reset(elevatorSimulator.getPositionMeters());
              m_controller.setGoal(position);
              enablePID = true;
            },
            this);
    return ejecutable;
  }
}
