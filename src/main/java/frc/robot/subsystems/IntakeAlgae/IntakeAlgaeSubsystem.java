package frc.robot.Subsystems.IntakeAlgae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAlgaeSubsystem extends SubsystemBase{

    private final IntakeAlgaeIO io;
    private final IntakeAlgaeIOInputsAutoLogged inputs = new IntakeAlgaeIOInputsAutoLogged();
    private PIDController m_controller = new PIDController(0.0032,0,0);
    private boolean enablePID = false;
    private SendableChooser<String> pivotChooser = new SendableChooser<>();

    public IntakeAlgaeSubsystem(IntakeAlgaeIO io){
        this.io = io;

    pivotChooser.setDefaultOption("Break", "Break");
    pivotChooser.addOption("Coast", "Coast");
    pivotChooser.addOption("Break", "Break");

    SmartDashboard.putData("Pivot algae mode",pivotChooser);
    }
    
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("IntakeAlgae", inputs);
        if(enablePID){
            io.setVoltagePiv(
              m_controller.calculate(inputs.positionPiv));
                    }

        Logger.recordOutput("IntakeAlgae/PID output", m_controller.calculate(inputs.positionPiv));  
        Logger.recordOutput("IntakeAlgae/PID setpoint", m_controller.getSetpoint());    
        Logger.recordOutput("IntakeAlgae/PID enables", enablePID);      
      if(DriverStation.isDisabled()){

      enablePID = false;
      switch(pivotChooser.getSelected()){
        case "Break":
        io.enableBreak(true);
        break;

        case "Coast":
        io.enableBreak(false);
        break;

        default:
        io.enableBreak(true);
        break;
      }
    }else{
      io.enableBreak(true);
    }


    }

    public double getAmperage(){
      return inputs.rollerMotorCurrent;
    }

    public double getPivotPosition(){
      return inputs.positionPiv;
    }

    public void resetController(){
      m_controller.reset();
    }
    public void setVoltagePivVoid(double pivotVolt){
        io.setVoltagePiv(pivotVolt);
    }
    
    public void stopMotors(){
      io.stopMotors();
    }
    
    public Command setVoltageCommandPiv(double pivotVolt) {
      return run(() -> io.setVoltagePiv(pivotVolt));
    }

    public Command setVoltageCommandRoll(double rollerVolt) {
        return run(() -> io.setVoltageRoll(rollerVolt));
    }

      public Command goToPosition(double position) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
              m_controller.reset();
              m_controller.setSetpoint(position);
              enablePID = true;
            },
            this);
    return ejecutable;
  }

  public Command goToPositionVoltage(double position) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
              m_controller.reset();
              m_controller.setSetpoint(position);
              enablePID = true;
            },
            this);
    return ejecutable;
  }

    public void setVoltageRollerVoid(double voltage){
      io.setVoltageRoll(voltage);
    }
}
