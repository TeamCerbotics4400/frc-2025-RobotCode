package frc.robot.Subsystems.IntakeAlgae;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeAlgaeIOInputsAutoLogged extends IntakeAlgaeIO.IntakeAlgaeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PivotMotorappliedVolts", pivotMotorappliedVolts);
    table.put("PivotMotortempCelcius", pivotMotortempCelcius);
    table.put("PivotMotorCurrent", pivotMotorCurrent);
    table.put("PivotCurrentRpms", pivotCurrentRpms);
    table.put("PositionPiv", positionPiv);
    table.put("RollerMotorappliedVolts", rollerMotorappliedVolts);
    table.put("RollerMotortempCelcius", rollerMotortempCelcius);
    table.put("RollerMotorCurrent", rollerMotorCurrent);
    table.put("RollerCurrentRpms", rollerCurrentRpms);
  }

  @Override
  public void fromLog(LogTable table) {
    pivotMotorappliedVolts = table.get("PivotMotorappliedVolts", pivotMotorappliedVolts);
    pivotMotortempCelcius = table.get("PivotMotortempCelcius", pivotMotortempCelcius);
    pivotMotorCurrent = table.get("PivotMotorCurrent", pivotMotorCurrent);
    pivotCurrentRpms = table.get("PivotCurrentRpms", pivotCurrentRpms);
    positionPiv = table.get("PositionPiv", positionPiv);
    rollerMotorappliedVolts = table.get("RollerMotorappliedVolts", rollerMotorappliedVolts);
    rollerMotortempCelcius = table.get("RollerMotortempCelcius", rollerMotortempCelcius);
    rollerMotorCurrent = table.get("RollerMotorCurrent", rollerMotorCurrent);
    rollerCurrentRpms = table.get("RollerCurrentRpms", rollerCurrentRpms);
  }

  public IntakeAlgaeIOInputsAutoLogged clone() {
    IntakeAlgaeIOInputsAutoLogged copy = new IntakeAlgaeIOInputsAutoLogged();
    copy.pivotMotorappliedVolts = this.pivotMotorappliedVolts;
    copy.pivotMotortempCelcius = this.pivotMotortempCelcius;
    copy.pivotMotorCurrent = this.pivotMotorCurrent;
    copy.pivotCurrentRpms = this.pivotCurrentRpms;
    copy.positionPiv = this.positionPiv;
    copy.rollerMotorappliedVolts = this.rollerMotorappliedVolts;
    copy.rollerMotortempCelcius = this.rollerMotortempCelcius;
    copy.rollerMotorCurrent = this.rollerMotorCurrent;
    copy.rollerCurrentRpms = this.rollerCurrentRpms;
    return copy;
  }
}
