package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  /*Io and inputs */
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public static enum ClimbingState{
    PREPARING_CLIMB,
    CLIMBING
  }

  public ClimbingState climbState = ClimbingState.PREPARING_CLIMB;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command setKrakenVoltage(double voltage) {
    return run(() -> io.setTalonFXVoltage(voltage));
  }



  public Command setCageMotorVoltage(double voltage){
   return run (()  -> io.setCageMotorVolatge(voltage));
  }

  public Command setClimberPosition(double position) {
    Command ejecutable = Commands.runOnce(() -> {
      io.setTalonFXPosition(position);
    },
    this);
    return ejecutable;
  }  

   /* return     Commands.runOnce(
            () -> {
              io.setTalonFXPosition(position);
            },
            this);*/ 

/* 

 public Command goToPosition(Double position) {
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

  public void setNeoVoidVoltage(double voltage){
    io.setSparkMaxVoltage(voltage);
  }
    
      public Command setNeoVoltage(double voltage) {
    return run(() -> io.setSparkMaxVoltage(voltage));
  }  

  
  public Command setNeoPosition(double position) {
    return   
           run(
            () -> 
              io.setSparkPosition(position));
              }  


              */
    
}