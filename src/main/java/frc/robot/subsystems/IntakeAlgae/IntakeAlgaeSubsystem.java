package frc.robot.Subsystems.IntakeAlgae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAlgaeSubsystem extends SubsystemBase{

    private final IntakeAlgaeIO io;
    private final IntakeAlgaeIOInputsAutoLogged inputs = new IntakeAlgaeIOInputsAutoLogged();
    private PIDController m_controller = new PIDController(0.074,0,0.001);
    private boolean enablePID = false;
    private Debouncer currentFilter = new Debouncer(0.5,DebounceType.kBoth);

    public static enum AlgaeState{
      ACTIVEPOSITION,
      BACKPOSITION
    }
    private AlgaeState systemStates = AlgaeState.BACKPOSITION;

    public IntakeAlgaeSubsystem(IntakeAlgaeIO io){
        this.io = io;

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
        Logger.recordOutput("IntakeAlgae/Algae Detected", currentFilter.calculate(inputs.rollerMotorCurrent > 54));      
   
      if(DriverStation.isDisabled()){
      enablePID = false;
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

    public AlgaeState getState() {
      return systemStates;
    }
  
    public AlgaeState changeState(AlgaeState state) {
      systemStates = state;
      return systemStates;
    }

    public double getRollerCurrent(){
      return inputs.rollerMotorCurrent;
    }

      public Command goToPosition(double position, AlgaeState state) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
              m_controller.reset();
              m_controller.setSetpoint(position);
              enablePID = true;
              systemStates = state;
            },
            this);
    return ejecutable;
  }

  public void goToPositionVoid(double position){
    m_controller.setSetpoint(position);
    enablePID = true;
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

    public boolean isGamePieceInside(){
      return inputs.rollerMotorCurrent > 40;
    }
}
