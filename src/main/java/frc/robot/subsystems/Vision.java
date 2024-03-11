package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Vision {
    private static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static final NetworkTable rpiTable = NetworkTableInstance.getDefault().getTable("raspberrypi");

    public static Pose3d getBotPose() {
        if (!limelightTable.getEntry("botpose").exists()) return null;
        double[] ntdata = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        double xPos = ntdata[0] + Constants.Environment.fieldWidth/2;
        double yPos = ntdata[1] + Constants.Environment.fieldHeight/2;
        return new Pose3d(xPos, yPos, ntdata[2], new Rotation3d(ntdata[3], ntdata[4], ntdata[5]));
    } // limelight translation is y 12.947", z 8.03", pitch 64 deg

    public static double getLimelightPing() {
        return limelightTable.getEntry("cl").getDouble(-1);
    }

    public static Translation3d getNoteTranslation() {
        if (!limelightTable.getEntry("notetrans").exists()) return null;
        double[] ntdata = rpiTable.getEntry("notetrans").getDoubleArray(new double[3]);
        return Constants.Intake.luxonisTranslation.getTranslation().plus(new Translation3d(
            Conversions.millimetersToMeters(ntdata[0]),
            Conversions.millimetersToMeters(ntdata[1]),
            Conversions.millimetersToMeters(ntdata[2])
        ).rotateBy(Constants.Intake.luxonisTranslation.getRotation()));
    }

    public static double getRpiPing() {
        return rpiTable.getEntry("cl").getDouble(-1);
    }
}
