package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Swerve.TunerConstants;

public class FieldCentricDrive extends Command {

  private CommandSwerveDrivetrain m_drive;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); 

  private final SwerveRequest.FieldCentric fieldCentricdrive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage);

  public FieldCentricDrive(
    CommandSwerveDrivetrain m_drive,
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction) {

    this.m_drive = m_drive;

    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;

    this.xLimiter = new SlewRateLimiter(MaxSpeed);
    this.yLimiter = new SlewRateLimiter(MaxSpeed);
    this.turningLimiter = new SlewRateLimiter(MaxAngularRate);

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    xSpeed = xLimiter.calculate(xSpeed) * MaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * MaxSpeed;
    turningSpeed = turningLimiter.calculate(turningSpeed) * MaxAngularRate;

    m_drive.setControl(
      fieldCentricdrive.
            withVelocityX(xSpeed).
            withVelocityY(ySpeed).
            withRotationalRate(turningSpeed));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
