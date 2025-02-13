package frc.robot.Subsystems.IntakeAlgae;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeAlgaeIO {

    @AutoLog
    public static class IntakeAlgaeIOInputs{

    public double pivotMotorappliedVolts = 0.0;
    public double pivotMotortempCelcius = 0.0;
    public double pivotMotorCurrent = 0.0;
    public double pivotCurrentRpms = 0.0;
    public double positionPiv = 0.0;

    public double rollerMotorappliedVolts = 0.0;
    public double rollerMotortempCelcius = 0.0;
    public double rollerMotorCurrent = 0.0;
    public double rollerCurrentRpms = 0.0;
    }

    public default void updateInputs(IntakeAlgaeIOInputs inputs){}

    public default void setVoltagePiv(double pivotVolt){}

    public default void setPositionPiv(double positionPiv){}

    public default void setVoltageRoll(double rollerVolt){}

    public default void setVelocityPiv(double pivotVell){}

    public default void stopMotors(){}
}