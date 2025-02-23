package frc.robot.Subsystems.IntakeAlgae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAlgaeSubsystem extends SubsystemBase{

    private final IntakeAlgaeIO io;
    private final IntakeAlgaeIOInputsAutoLogged inputs = new IntakeAlgaeIOInputsAutoLogged();

    public IntakeAlgaeSubsystem(IntakeAlgaeIO io){
        this.io = io;
    }
    
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("IntakeAlgae", inputs);
    }
    public void setVoltagePiv(double pivotVolt){
        io.setVoltagePiv(pivotVolt);
    }

    public void setVelocityPiv(double pivotVolt) {
        double valPivot = pivotVolt / 60;
        io.setVelocityPiv(valPivot);
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

    public Command setPositionCommand(double position){
        return run(()-> io.setPositionPiv(position));
    }

    public void setVoltageRollerVoid(double voltage){
      io.setVoltageRoll(voltage);
    }
}
