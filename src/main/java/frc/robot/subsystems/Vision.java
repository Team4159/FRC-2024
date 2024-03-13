package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static final NetworkTable rpiTable = NetworkTableInstance.getDefault().getTable("raspberrypi");

    private Kinesthetics kinesthetics;
    private final GenericEntry isSingleTarget;

    public Vision(Kinesthetics k) {
        this.kinesthetics = k;

        var table = Shuffleboard.getTab("Vision");

        isSingleTarget = table
            .add("Single-Target", false)
            .withWidget(BuiltInWidgets.kToggleSwitch).getEntry("boolean");
        table.addDouble("LL Error", () -> {
            var v = Vision.getBotPose();
            if (v == null) return -1d;
            return kinesthetics.getPose().getTranslation().getDistance(v.getTranslation().toTranslation2d());
        });
        table.addBoolean("Note Seen", () -> limelightTable.getEntry("notetrans").exists());
    }

    @Override
    public void periodic() {
        int desiredPipeline = isSingleTarget.getBoolean(false) ? 1 : 0;
        if (getLimelightPipe() != desiredPipeline) setLimelightPipe(desiredPipeline);
    }

    public static Pose3d getBotPose() {
        if (!limelightTable.getEntry("botpose_wpiblue").exists()) return null;
        double[] ntdata = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        return new Pose3d(
            new Translation3d(ntdata[0], ntdata[1], ntdata[2]),
            new Rotation3d(ntdata[3], ntdata[4], ntdata[5])
        );
    } // limelight translation is y 12.947", z 8.03", pitch 64 deg

    public static double getLimelightPing() {
        return limelightTable.getEntry("cl").getDouble(-1);
    }

    public static void setLimelightPipe(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    public static double getLimelightPipe() {
        return limelightTable.getEntry("pipeline").getDouble(-1);
    }

    public static Translation3d getNoteTranslation() {
        if (!rpiTable.getEntry("notetrans").exists()) return null;
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
