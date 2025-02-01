package frc.robot.Commands.AutoCommands.Paths.WorkShopPaths;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.AutoCommands.AutoCommand;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class TestAuto extends AutoCommand {

  private final PathPlannerPath first;

  public TestAuto() {
    first = loadPath("Dontdieplspath");

    addCommands(
        Commands.deadline(
            Commands.sequence(
             new PathPlannerAuto("Elevator test"))));
  }

  private PathPlannerPath loadPath(String fileName) {
    try {
      return PathPlannerPath.fromPathFile(fileName);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path: " + fileName + " - " + e.getMessage(), e.getStackTrace());
      return null; // Return a safe fallback or handle as needed
    }
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(
            safeGetPathPoses(first))
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  /*To prevent the path being null and crashing the code lol */
  private List<Pose2d> safeGetPathPoses(PathPlannerPath path) {
    return path != null ? path.getPathPoses() : new ArrayList<>();
  }

  @Override
  public Pose2d getStartingPose() {
    if (first != null) {
      return first.getStartingDifferentialPose();
    }
    DriverStation.reportError("First path is null. Returning default starting pose.", true);
    return new Pose2d();
  }
}
