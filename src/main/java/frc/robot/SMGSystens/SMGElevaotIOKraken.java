package frc.robot.SMGSystens;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class SMGElevaotIOKraken implements SMGElevatorIO {

   /* Declarar primero los controladores */

   TalonFX rightElevatorMotor;
   TalonFX leftElevatorMotor;

   /* Declarar el nombre las configuraciones */

   TalonFXConfiguration rightELconfig;
   TalonFXConfiguration leftELconfig;

   private final VoltageOut voltageRequest = new VoltageOut(0);

   public SMGElevaotIOKraken() {
      rightElevatorMotor = new TalonFX(Constants.ElevatorConstants.rightElevatorMotorId, Constants.rioCanbus);
      leftElevatorMotor = new TalonFX(ElevatorConstants.leftElevatorMotorId, Constants.rioCanbus);

      rightELconfig = new TalonFXConfiguration();
      rightELconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      rightELconfig.CurrentLimits.StatorCurrentLimit = 40;
      rightELconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      rightELconfig.CurrentLimits.StatorCurrentLimitEnable = true;

      leftELconfig = new TalonFXConfiguration();
      leftELconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      leftELconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      leftELconfig.CurrentLimits.SupplyCurrentLimit = 40;
      leftELconfig.CurrentLimits.SupplyCurrentLimitEnable = true;

      rightElevatorMotor.setPosition(0);
      leftElevatorMotor.setPosition(0);

      rightElevatorMotor.getConfigurator().apply(rightELconfig);
      leftElevatorMotor.getConfigurator().apply(leftELconfig);
   }

   public double getElevatorPosition() {
      double pos = (rightElevatorMotor.getPosition().getValueAsDouble()
            + leftElevatorMotor.getPosition().getValueAsDouble() / 2);
      return pos;
   }

   @Override
   public void resetEncoders() {
      rightElevatorMotor.setPosition(0);
      leftElevatorMotor.setPosition(0);
   }

   @Override
public void setVolatgeElevator(double voltage){
   rightElevatorMotor.setControl(voltageRequest.withOutput(voltage));
   leftElevatorMotor.setControl(voltageRequest.withOutput(voltage));
}

   @Override
   public void updateInputs( SMGElevatorInputs inputs){
      inputs.ElevatorPos = getElevatorPosition();
      inputs.FLVoltage = leftElevatorMotor.getMotorVoltage().getValueAsDouble();
      inputs.RLVoltage = rightElevatorMotor.getMotorVoltage().getValueAsDouble();
   }

}
