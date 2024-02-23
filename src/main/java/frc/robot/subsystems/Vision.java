package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Vision {
    private static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static final NetworkTable rpiTable = NetworkTableInstance.getDefault().getTable("raspberrypi");

    public static Pose3d getBotPose() {
        if (!limelightTable.getEntry("botpose").exists()) return null;
        double[] ntdata = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        return new Pose3d(ntdata[0], ntdata[1], ntdata[2], new Rotation3d(ntdata[3], ntdata[4], ntdata[5]));
    }

    public static double getLimelightPing() {
        return limelightTable.getEntry("cl").getDouble(-1);
    }

    public static Translation3d getNoteTranslation() {
        double[] ntdata = rpiTable.getEntry("notetrans").getDoubleArray(new double[3]);
        return new Translation3d(ntdata[0], ntdata[1], ntdata[2]).plus(Constants.Intake.luxonisTranslation);
    }

    public static double getRpiPing() {
        return rpiTable.getEntry("cl").getDouble(-1);
    }
}
