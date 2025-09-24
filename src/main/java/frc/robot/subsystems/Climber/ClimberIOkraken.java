package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ClimberIOkraken {


private final TalonFX climberFx;

private final TalonFXConfiguration climberTalonFXConfiguration;

public ClimberIOkraken(){

    climberFx = new TalonFX(Constants.ClimberConstants.CLIMBERKRAKENID, Constants.rioCanbus);

    climberTalonFXConfiguration= new TalonFXConfiguration();
    climberTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
    climberTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

    climberFx.setPosition(0);
    climberFx.getConfigurator().apply(climberTalonFXConfiguration);

}

    
}
