package frc.lib.math;

import java.util.Objects;

import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

public class RobotState extends Pose2d {
    private Vector<N2> m_velocity;

    public RobotState(Pose2d pose) {
        this(pose.getTranslation(), pose.getRotation(), new Vector<N2>(Nat.N2()));
    }

    public RobotState(Translation2d position, Rotation2d rotation, Vector<N2> velocity) {
        super(position, rotation);
        this.m_velocity = velocity;
    }

    public double getVelocityX() {
        return m_velocity.get(0, 0);
    }

    public double getVelocityY() {
        return m_velocity.get(0, 1);
    }

    @JsonProperty
    public Vector<N2> getVelocity() {
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