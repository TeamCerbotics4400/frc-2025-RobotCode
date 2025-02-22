package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d elevator;
  private final String key;

  public ElevatorVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    LoggedMechanismRoot2d root = mechanism.getRoot("pivot", 1.68, 0.1);
    elevator = new LoggedMechanismLigament2d("Elevator", 0.6, 90.0, 6, new Color8Bit(color));
    root.append(elevator);
  }

  /** Update arm visualizer with current arm angle */
  public void update(double position) {
    // Log Mechanism2d
    elevator.setLength(position);
    Logger.recordOutput("Elevator/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d pivot = new Pose3d(-0.0, 0.0, 0.0, new Rotation3d(0.0, 0, 0.0));
    Logger.recordOutput("Elevator/Mechanism3d/" + key, getElevatorDistalPose(pivot, position));
  }

  public Pose3d getElevatorDistalPose(Pose3d armProximalPose, double position) {
    double armExtensionLength = position;
    return armProximalPose.transformBy(
        new Transform3d(new Translation3d(0.0, 0.0, armExtensionLength), new Rotation3d()));
  }
}
