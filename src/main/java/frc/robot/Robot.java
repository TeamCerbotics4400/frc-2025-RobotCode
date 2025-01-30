// Copyright (c) 2025 FRC 4400//
package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.Util.CustomDashboardUtil;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  /* Class to log the app Network tables data*/
  private CustomDashboardUtil dashboardCustom = new CustomDashboardUtil();
  private Command m_autonomousCommand;

  /* Declaring PDH to unlock an extra spot and clear faults */
  private PowerDistribution examplePD = new PowerDistribution(1, ModuleType.kRev);

  /*Alerts in case of emergencies that need to be checked */
  private Alert robotModeAlert = new Alert("The code of the robot mode isn't real", AlertType.kWarning);

  private final RobotContainer m_robotContainer;


  public Robot() {
    m_robotContainer = new RobotContainer();
    /* PDH configs */
    examplePD.setSwitchableChannel(true);
    examplePD.clearStickyFaults();

    /*Data that records the projects dates and info */
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    /* Emergency in case code is deployed onto a real robot */
    if(Robot.isReal()){
      Constants.currentMode = Constants.Mode.REAL;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    /* Idk mechanical did this so might as well copy it */
    Threads.setCurrentThreadPriority(true, 99);

    CommandScheduler.getInstance().run();

    Threads.setCurrentThreadPriority(false, 10);
    
    /* Alert if the robot mode isn't real */
    robotModeAlert.set(Constants.currentMode != Constants.Mode.REAL);

    Logger.recordOutput("Dashboard/Match Time", DriverStation.getMatchTime());
    Logger.recordOutput("Dashboard/Current Robot mode", Constants.currentMode.toString());

    Logger.recordOutput("Dashboard/Reef Position", dashboardCustom.getReefSelected());
    Logger.recordOutput("Dashboard/Level selected", dashboardCustom.getLevelEntry());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance()
        .filter(value -> value == DriverStation.Alliance.Red)
        .isPresent();
  }
}
