package frc.robot.Subsystems.Intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LeftMotorappliedVolts", leftMotorappliedVolts);
    table.put("LeftMotortempCelcius", leftMotortempCelcius);
    table.put("LeftMotorCurrent", leftMotorCurrent);
    table.put("LeftCurrentRpms", leftCurrentRpms);
    table.put("RightMotorappliedVolts", rightMotorappliedVolts);
    table.put("RightMotortempCelcius", rightMotortempCelcius);
    table.put("RightMotorCurrent", rightMotorCurrent);
    table.put("RightCurrentRpms", rightCurrentRpms);
    table.put("Sensor", sensor);
  }

  @Override
  public void fromLog(LogTable table) {
    leftMotorappliedVolts = table.get("LeftMotorappliedVolts", leftMotorappliedVolts);
    leftMotortempCelcius = table.get("LeftMotortempCelcius", leftMotortempCelcius);
    leftMotorCurrent = table.get("LeftMotorCurrent", leftMotorCurrent);
    leftCurrentRpms = table.get("LeftCurrentRpms", leftCurrentRpms);
    rightMotorappliedVolts = table.get("RightMotorappliedVolts", rightMotorappliedVolts);
    rightMotortempCelcius = table.get("RightMotortempCelcius", rightMotortempCelcius);
    rightMotorCurrent = table.get("RightMotorCurrent", rightMotorCurrent);
    rightCurrentRpms = table.get("RightCurrentRpms", rightCurrentRpms);
    sensor = table.get("Sensor", sensor);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.leftMotorappliedVolts = this.leftMotorappliedVolts;
    copy.leftMotortempCelcius = this.leftMotortempCelcius;
    copy.leftMotorCurrent = this.leftMotorCurrent;
    copy.leftCurrentRpms = this.leftCurrentRpms;
    copy.rightMotorappliedVolts = this.rightMotorappliedVolts;
    copy.rightMotortempCelcius = this.rightMotortempCelcius;
    copy.rightMotorCurrent = this.rightMotorCurrent;
    copy.rightCurrentRpms = this.rightCurrentRpms;
    copy.sensor = this.sensor;
    return copy;
  }
}
