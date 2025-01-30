package frc.Util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CustomDashboardUtil {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("Dashboard");
  private static NetworkTableEntry reefEntry = table.getEntry("TargetClimbPos");
  private static NetworkTableEntry levelEntry = table.getEntry("ScoringMode");
    public CustomDashboardUtil(){}

    public int getReefSelected(){
        return (int) Math.round(reefEntry.getDouble(0));
    }

    public int getLevelEntry(){
      return (int) Math.round(levelEntry.getDouble(0));
  }
}
