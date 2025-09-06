package frc.robot.Subsystems.Elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorInputsAutoLogged extends ElevatorIO.ElevatorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ElevatorPosition", elevatorPosition);
    table.put("RightElevatorTemp", rightElevatorTemp);
    table.put("RightElevatorCurrent", rightElevatorCurrent);
    table.put("RightElevatorVoltage", rightElevatorVoltage);
    table.put("LeftElevatorTemp", leftElevatorTemp);
    table.put("LeftElevatorCurrent", leftElevatorCurrent);
    table.put("LeftElevatorVoltage", leftElevatorVoltage);
  }

  @Override
  public void fromLog(LogTable table) {
    elevatorPosition = table.get("ElevatorPosition", elevatorPosition);
    rightElevatorTemp = table.get("RightElevatorTemp", rightElevatorTemp);
    rightElevatorCurrent = table.get("RightElevatorCurrent", rightElevatorCurrent);
    rightElevatorVoltage = table.get("RightElevatorVoltage", rightElevatorVoltage);
    leftElevatorTemp = table.get("LeftElevatorTemp", leftElevatorTemp);
    leftElevatorCurrent = table.get("LeftElevatorCurrent", leftElevatorCurrent);
    leftElevatorVoltage = table.get("LeftElevatorVoltage", leftElevatorVoltage);
  }

  public ElevatorInputsAutoLogged clone() {
    ElevatorInputsAutoLogged copy = new ElevatorInputsAutoLogged();
    copy.elevatorPosition = this.elevatorPosition;
    copy.rightElevatorTemp = this.rightElevatorTemp;
    copy.rightElevatorCurrent = this.rightElevatorCurrent;
    copy.rightElevatorVoltage = this.rightElevatorVoltage;
    copy.leftElevatorTemp = this.leftElevatorTemp;
    copy.leftElevatorCurrent = this.leftElevatorCurrent;
    copy.leftElevatorVoltage = this.leftElevatorVoltage;
    return copy;
  }
}
