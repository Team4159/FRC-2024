package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photogates extends SubsystemBase {
    private final double DELTA_DISTANCE = 0.05; // meters
    private final double NOTE_DIAMETER = Units.inchesToMeters(14);

    private DigitalInput a = new DigitalInput(2), b = new DigitalInput(8);
    private Timer timerDelta = new Timer();
    private Timer timerFirst = new Timer();
    private Timer timerSecond= new Timer();

    private List<Double> dataDelta = new ArrayList<>();
    private List<Double> dataFirst = new ArrayList<>();
    private List<Double> dataSecond= new ArrayList<>();

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
                dataFirst.add(NOTE_DIAMETER / timerFirst.get());
            }
            lastA = nowA;
        }
        var nowB = b.get();
        if (lastB != nowB) {
            if (nowB) {
                timerSecond.restart();
                if (nowA) {
                    timerDelta.stop();
                    dataDelta.add(DELTA_DISTANCE / timerDelta.get());
                }
            } else {
                timerSecond.stop();
                dataSecond.add(NOTE_DIAMETER / timerSecond.get());
            }
            lastB = nowB;
        }
    }

    public Set<Pair<Double, Double>> exportData(int type, double xValue) {
        Set<Pair<Double, Double>> out = new HashSet<>();
        List<Double> data;
        switch (type) {
            case 0: data = dataDelta; break;
            case 1: data = dataFirst; break;
            case 2: data = dataSecond; break;
            default: return null;
        }
        for (Double yValue : data) out.add(new Pair<>(xValue, yValue));
        return out;
    }

    public void clearData() {
        dataDelta.clear();
        dataFirst.clear();
        dataSecond.clear();
    }

    public static double calculateRegression(Set<Pair<Double, Double>> points) {
        double xMean = 0, yMean = 0;
        for (Pair<Double,Double> point : points) {
            xMean += point.getFirst();
            yMean += point.getSecond();
        }
        xMean /= points.size();
        yMean /= points.size();

        double numerator = 0, denominator = 0;
        for (Pair<Double,Double> point : points) {
            double dx = (point.getFirst() - xMean);
            numerator += dx * (point.getSecond() - yMean);
            denominator += dx * dx;
        }
        return numerator / denominator;
    }
}