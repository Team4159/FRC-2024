package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public static Pose3d getVisionPose() {
        double[] ntdata = table.getEntry("botpose").getDoubleArray(new double[6]);
        return new Pose3d(ntdata[0], ntdata[1], ntdata[2], new Rotation3d(ntdata[3], ntdata[4], ntdata[5]));
    }

    public static double latestLatency() {
        return table.getEntry("cl").getDouble(-1);
    }
}
