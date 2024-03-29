package frc.lib.math;

import java.util.Objects;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

/** Represents a 2D pose state containing translational, rotational and velocity elements. */
public class RobotState extends Pose2d {
    private final Vector<N3> m_velocity;

    public static RobotState fromVelocity(Pose2d pose, double vx, double vy, double vw) {
        var vec = new Vector<N3>(Nat.N3());
        vec.set(0, 0, vx);
        vec.set(1, 0, vy);
        vec.set(2, 0, vw);
        return new RobotState(pose.getTranslation(), pose.getRotation(), vec);
    }

    /** Creates a RobotState with no velocity component */
    public RobotState(Pose2d pose) {
        this(pose.getTranslation(), pose.getRotation(), new Vector<N3>(Nat.N3()));
    }

    public RobotState(Translation2d position, Rotation2d rotation, Vector<N3> velocity) {
        super(position, rotation);
        this.m_velocity = velocity;
    }

    /** @return meters / second (right +) */
    public double getvx() {
        return m_velocity.get(0, 0);
    }

    /** @return meters / second (forward +) */
    public double getvy() {
        return m_velocity.get(1, 0);
    }

    /** @return radians / second (ccw +) */
    public double getvw() { // ω
        return m_velocity.get(2, 0);
    }

    public Vector<N3> getVelocity() {
        return m_velocity;
    }

    @Override
    public String toString() {
        return String.format("RobotState(%s, %s, %s)", super.getTranslation(), super.getRotation(), m_velocity);
    }

    @Override
    public boolean equals(Object obj) {
        return (obj instanceof RobotState rs) ? rs.getVelocity().equals(m_velocity) && super.equals(obj) : false;
    }
    
    @Override
    public int hashCode() {
        return Objects.hash(super.getTranslation(), super.getRotation(), m_velocity);
    }
}