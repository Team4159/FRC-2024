package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static final NetworkTable rpiTable = NetworkTableInstance.getDefault().getTable("raspberrypi");

    private Kinesthetics kinesthetics;

    private static final Field2d field = new Field2d();

    public Vision(Kinesthetics k) {
        this.kinesthetics = k;

        var table = Shuffleboard.getTab("Vision");

        table.addDouble("LL Error", () -> {
            var v = Vision.getLimelightData();
            if (v == null) return -1d;
            return kinesthetics.getPose().getTranslation().getDistance(v.pose().getTranslation().toTranslation2d());
        });
        table.addBoolean("Note Seen", () -> limelightTable.getEntry("notetrans").exists());
        table.add("Vision Field", field);

        limelightTable.putValue("camerapose_robotspace_set", NetworkTableValue.makeDoubleArray(new double[]{
            Units.inchesToMeters(12.947), 0, Units.inchesToMeters(8.03), 0, 26, 0
        })); // forward+, right+, up+, roll, pitch, yaw (ccw+)
    }

    /** @param omega degrees / second */
    public static void setRobotYaw(double theta, double omega) {
        limelightTable.putValue("robot_orientation_set", NetworkTableValue.makeDoubleArray(new double[]{
            theta,omega, 0,0, 0,0
        }));
    }

    public static VisionData getLimelightData() {
        var entry = limelightTable.getEntry(false ? "botpose_orb_wpiblue" : "botpose_wpiblue");
        if (!entry.exists()) return null;
        double[] ntdata = entry.getDoubleArray(new double[7]);
        if (ntdata[0] == 0 && ntdata[1] == 0 && ntdata[2] == 0) return null;
        var o = new Pose3d(
            new Translation3d(ntdata[0], ntdata[1], ntdata[2]),
            new Rotation3d(0, 0, Units.degreesToRadians(ntdata[5]))
        );
        double area = limelightTable.getEntry("ta").getDouble(0.25);
        field.setRobotPose(o.toPose2d());
        return new VisionData(
            o,
            1 - area * 0.7,
            ntdata[6]
        );
    }

    public class Flash extends InstantCommand {
        private static final NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");
        public Flash() {
            super(() -> new Thread(() -> {
                ledMode.setBoolean(true);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {}
                ledMode.setBoolean(false);
            }).start(), Vision.this);
        }
    }

    /** @param ping seconds since image taken (+) */
    public static record VisionData(Pose3d pose, double confidence, double ping) {};

    public static Translation3d getNoteTranslation() {
        var notetrans = rpiTable.getEntry("notetrans");
        if (!notetrans.exists() || notetrans.equals(null)) return null;
        double[] ntdata = notetrans.getDoubleArray(new double[3]);
        notetrans.setValue(null);
        return Constants.Intake.luxonisTranslation.getTranslation().plus(new Translation3d(
            Conversions.millimetersToMeters(ntdata[0]),
            Conversions.millimetersToMeters(ntdata[1]),
            Conversions.millimetersToMeters(ntdata[2])
        ).rotateBy(Constants.Intake.luxonisTranslation.getRotation()));
    }
}
