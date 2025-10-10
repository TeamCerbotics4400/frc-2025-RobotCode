package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.Constants;

public class ClimberIOkraken implements ClimberIO {

    private final TalonFX climberFx;
    private final TalonFXConfiguration climberTalonFXConfiguration;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public ClimberIOkraken() {

        climberFx = new TalonFX(Constants.ClimberConstants.CLIMBERKRAKENID, Constants.rioCanbus);

        climberTalonFXConfiguration = new TalonFXConfiguration();
        climberTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        climberTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        climberTalonFXConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        climberFx.setPosition(0);
        climberFx.getConfigurator().apply(climberTalonFXConfiguration);

    }

    @Override
    public void setTalonFXVoltage(double voltage) {
        climberFx.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void stopMotor() {
        climberFx.stopMotor();
    }

    public double getClimberPosition() {
        double pos = (climberFx.getPosition().getValueAsDouble());
        return pos;

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberFxPosition = getClimberPosition();

    }

}
