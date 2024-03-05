package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photogates extends SubsystemBase {
    private final double NOTE_DIAMETER = Units.inchesToMeters(14);

    private final DigitalInput a = new DigitalInput(8), b = new DigitalInput(2);
    private final Timer timerFirst = new Timer();
    private final Timer timerSecond= new Timer();
    private final List<PhotogateData> data = new ArrayList<>();

    private DoubleSupplier xValue;
    public Photogates(Shooter s) {xValue = s::getSpin;}

    private boolean lastA = true, lastB = true;

    @Override
    public void periodic() {
        var nowA = a.get();
        if (lastA != nowA) {
            if (!nowA) { // true to false
                timerFirst.restart();
            } else { // false to false
                timerFirst.stop();
                SmartDashboard.putNumber("first", timerFirst.get());
                data.add(new PhotogateData(PhotogateDataMode.FIRST, xValue.getAsDouble(), NOTE_DIAMETER / timerFirst.get()));
            }
            lastA = nowA;
        }
        var nowB = b.get();
        if (lastB != nowB) {
            if (!nowB) {
                timerSecond.restart();
            } else {
                timerSecond.stop();
                SmartDashboard.putNumber("second", timerSecond.get());
                data.add(new PhotogateData(PhotogateDataMode.SECOND, xValue.getAsDouble(), NOTE_DIAMETER / timerSecond.get()));
            }
            lastB = nowB;
        }
        SmartDashboard.putNumber("data", data.size());
    }

    public void clearData() {
        data.clear();
    }

    public SimpleMotorFeedforward calculateRegression() {
        double a = 0, b = 0;
        double sumx = 0, sumy = 0;
        for (PhotogateData point : data) {
            sumx += point.shooterRadPerSec();
            sumy += point.photogateMetPerSec();
            a += point.shooterRadPerSec() * point.photogateMetPerSec();
            b += point.shooterRadPerSec() * point.shooterRadPerSec();
        }
        // slope in meters / radian
        double inverseSlope = (data.size() * b - sumx * sumx) / (data.size() * a - sumx * sumy);
        return new SimpleMotorFeedforward(((sumx/inverseSlope - sumy) / data.size()) * inverseSlope, inverseSlope);
    }

    public static enum PhotogateDataMode {FIRST, SECOND}
    private record PhotogateData(PhotogateDataMode mode, double shooterRadPerSec, double photogateMetPerSec) {};
}
