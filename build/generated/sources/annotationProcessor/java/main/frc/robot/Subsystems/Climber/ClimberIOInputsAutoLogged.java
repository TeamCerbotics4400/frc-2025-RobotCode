package frc.robot.Subsystems.Climber;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("SparkAppliedVolts", sparkAppliedVolts);
    table.put("SparkTempCelcius", sparkTempCelcius);
    table.put("SparkPosition", sparkPosition);
    table.put("PidOutput", pidOutput);
  }

  @Override
  public void fromLog(LogTable table) {
    sparkAppliedVolts = table.get("SparkAppliedVolts", sparkAppliedVolts);
    sparkTempCelcius = table.get("SparkTempCelcius", sparkTempCelcius);
    sparkPosition = table.get("SparkPosition", sparkPosition);
    pidOutput = table.get("PidOutput", pidOutput);
  }

  public ClimberIOInputsAutoLogged clone() {
    ClimberIOInputsAutoLogged copy = new ClimberIOInputsAutoLogged();
    copy.sparkAppliedVolts = this.sparkAppliedVolts;
    copy.sparkTempCelcius = this.sparkTempCelcius;
    copy.sparkPosition = this.sparkPosition;
    copy.pidOutput = this.pidOutput;
    return copy;
  }
}
