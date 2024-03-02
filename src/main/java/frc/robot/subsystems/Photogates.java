package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photogates extends SubsystemBase {
    private final double DELTA_DISTANCE = 0.05; // meters
    private final double NOTE_DIAMETER = Units.inchesToMeters(14);

    private final DigitalInput a = new DigitalInput(2), b = new DigitalInput(8);
    private final Timer timerDelta = new Timer();
    private final Timer timerFirst = new Timer();
    private final Timer timerSecond= new Timer();
    private final List<PhotogateData> data = new ArrayList<>();

    private DoubleSupplier xValue;
    public Photogates(Shooter s) {xValue = s::getSpin;}

    private boolean lastA = false, lastB = false;

    @Override
    public void periodic() {
        var nowA = a.get();
        if (lastA != nowA) {
            if (nowA) { // false to true
                timerFirst.restart();
                timerDelta.restart();
            } else { // true to false
                timerFirst.stop();
                data.add(new PhotogateData(PhotogateDataMode.FIRST, xValue.getAsDouble(), NOTE_DIAMETER / timerFirst.get()));
            }
            lastA = nowA;
        }
        var nowB = b.get();
        if (lastB != nowB) {
            if (nowB) {
                timerSecond.restart();
                if (nowA) {
                    timerDelta.stop();
                    data.add(new PhotogateData(PhotogateDataMode.DELTA, xValue.getAsDouble(), DELTA_DISTANCE / timerDelta.get()));
                }
            } else {
                timerSecond.stop();
                data.add(new PhotogateData(PhotogateDataMode.SECOND, xValue.getAsDouble(), NOTE_DIAMETER / timerSecond.get()));
            }
            lastB = nowB;
        }
    }

    public void clearData() {
        data.clear();
    }

    public double calculateRegression(PhotogateDataMode mode) {
        var points = data.stream().filter(n -> n.mode().equals(mode)).toList();
        double xMean = 0, yMean = 0;
        for (PhotogateData point : points) {
            xMean += point.shooterRadPerSec();
            yMean += point.photogateMetPerSec();
        }
        xMean /= points.size();
        yMean /= points.size();

        double numerator = 0, denominator = 0;
        for (PhotogateData point : points) {
            double dx = (point.shooterRadPerSec() - xMean);
            numerator += dx * (point.photogateMetPerSec() - yMean);
            denominator += dx * dx;
        }
        return numerator / denominator;
    }

    public static enum PhotogateDataMode {DELTA, FIRST, SECOND}
    private record PhotogateData(PhotogateDataMode mode, double shooterRadPerSec, double photogateMetPerSec) {};
}
