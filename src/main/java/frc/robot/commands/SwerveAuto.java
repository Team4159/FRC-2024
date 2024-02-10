package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.math.RobotState;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Swerve;

public class SwerveAuto extends SequentialCommandGroup {
    public SwerveAuto(Kinesthetics kinesthetics, Swerve s_Swerve, RobotState destination) {
        // TrajectoryConfig config =
        // new TrajectoryConfig(
        // Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        // Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // .setKinematics(Constants.Swerve.swerveKinematics);

        // var thetaController =
        // new ProfiledPIDController(
        // Constants.AutoConstants.kPThetaController, 0, 0,
        // Constants.AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // RamseteCommand swerveControllerCommand =
        // new RamseteCommand(
        // TrajectoryGenerator.generateTrajectory(List.of(), config),
        // kinesthetics::getPose,
        // new RamseteController(),
        // Constants.Swerve.swerveKinematics,
        // (a, b) -> {},
        // s_Swerve);
    }
}

/*
 * A trajectory along one axis.
 * 
 * This is used to construct the optimal trajectory in one axis, planning
 * in the jerk to achieve position, velocity, and/or acceleration final
 * conditions. The trajectory is initialised with a position, velocity and
 * acceleration.
 * 
 * The trajectory is optimal with respect to the integral of jerk squared.
 * 
 * Do not use this in isolation, this useful through the "RapidTrajectory"
 * class, which wraps three of these and allows to test input/state
 * feasibility.
 */
class SingleAxisTrajectory {
    private double _p0;
    private double _v0;
    private double _a0;
    private double _pf;
    private double _vf;
    private double _af;
    private double _cost;
    private double _a;
    private double _b;
    private double _g;
    private double[] _accPeakTimes = new double[2];
    private boolean _posGoalDefined = false;
    private boolean _velGoalDefined = false;
    private boolean _accGoalDefined = false;

    /* Initialise the trajectory with starting state. */
    public SingleAxisTrajectory(double pos0, double vel0, double acc0) {
        this._p0 = pos0;
        this._v0 = vel0;
        this._a0 = acc0;
        this._pf = 0;
        this._vf = 0;
        this._af = 0;
        this.reset();
    }

    /* Define the goal position for a trajectory. */
    public void set_goal_position(double posf) {
        this._posGoalDefined = true;
        this._pf = posf;
    }

    /* Define the goal velocity for a trajectory. */
    public void set_goal_velocity(double velf) {
        this._velGoalDefined = true;
        this._vf = velf;
    }

    /* Define the goal acceleration for a trajectory. */
    public void set_goal_acceleration(double accf) {
        this._accGoalDefined = true;
        this._af = accf;
    }

    /*
     * Generate a trajectory of duration Tf.
     * Generate a trajectory, using the previously defined goal end states
     * (such as position, velocity, and/or acceleration).
     */
    public void generate(double Tf) {
        // define starting position:
        double delta_a = this._af - this._a0;
        double delta_v = this._vf - this._v0 - this._a0 * Tf;
        double delta_p = this._pf - this._p0 - this._v0 * Tf - 0.5 * this._a0 * Tf * Tf;

        // powers of the end time:
        double T2 = Tf * Tf;
        double T3 = T2 * Tf;
        double T4 = T3 * Tf;
        double T5 = T4 * Tf;

        // solve the trajectories, depending on what's constrained:
        if (this._posGoalDefined && this._velGoalDefined && this._accGoalDefined) {
            this._a = (60 * T2 * delta_a - 360 * Tf * delta_v + 720 * 1 * delta_p) / T5;
            this._b = (-24 * T3 * delta_a + 168 * T2 * delta_v - 360 * Tf * delta_p) / T5;
            this._g = (3 * T4 * delta_a - 24 * T3 * delta_v + 60 * T2 * delta_p) / T5;
        } else if (this._posGoalDefined && this._velGoalDefined) {
            this._a = (-120 * Tf * delta_v + 320 * delta_p) / T5;
            this._b = (72 * T2 * delta_v - 200 * Tf * delta_p) / T5;
            this._g = (-12 * T3 * delta_v + 40 * T2 * delta_p) / T5;
        } else if (this._posGoalDefined && this._accGoalDefined) {
            this._a = (-15 * T2 * delta_a + 90 * delta_p) / (2 * T5);
            this._b = (15 * T3 * delta_a - 90 * Tf * delta_p) / (2 * T5);
            this._g = (-3 * T4 * delta_a + 30 * T2 * delta_p) / (2 * T5);
        } else if (this._velGoalDefined && this._accGoalDefined) {
            this._a = 0;
            this._b = (6 * Tf * delta_a - 12 * delta_v) / T3;
            this._g = (-2 * T2 * delta_a + 6 * Tf * delta_v) / T3;
        } else if (this._posGoalDefined) {
            this._a = 20 * delta_p / T5;
            this._b = -20 * delta_p / T4;
            this._g = 10 * delta_p / T3;
        } else if (this._velGoalDefined) {
            this._a = 0;
            this._b = -3 * delta_v / T3;
            this._g = 3 * delta_v / T2;
        } else if (this._accGoalDefined) {
            this._a = 0;
            this._b = 0;
            this._g = delta_a / Tf;
        } else {
            // Nothing to do!
            this._a = this._b = this._g = 0;
        }

        // Calculate the cost:
        this._cost = (Math.pow(this._g, 2)) + this._b * this._g * Tf + (Math.pow(this._b, 2)) * T2 / 3d
                + this._a * this._g * T2 / 3d + this._a * this._b * T3 / 4d + (Math.pow(this._a, 2)) * T4 / 20d;
    }

    /* Reset the trajectory parameters. */
    public void reset() {
        this._cost = Double.POSITIVE_INFINITY;
        this._accGoalDefined = this._velGoalDefined = this._posGoalDefined = false;
        this._accPeakTimes[0] = 0;
        this._accPeakTimes[1] = 0;
    }

    /* Return the scalar jerk at time t. */
    public double get_jerk(double t) {
        return this._g + this._b * t + (1d / 2) * this._a * t * t;
    }

    /* Return the scalar acceleration at time t. */
    public double get_acceleration(double t) {
        return this._a0 + this._g * t + (1d / 2) * this._b * t * t + (1d / 6) * this._a * t * t * t;
    }

    /* Return the scalar velocity at time t. */
    public double get_velocity(double t) {
        return this._v0 + this._a0 * t + (1d / 2) * this._g * t * t +
                (1d / 6) * this._b * t * t * t + (1d / 24) * this._a * t * t * t * t;
    }

    /* Return the scalar position at time t. */
    public double get_position(double t) {
        return this._p0 + this._v0 * t + (1d / 2) * this._a0 * t * t +
                (1d / 6) * this._g * t * t * t + (1d / 24) * this._b * t * t * t * t +
                (1d / 120) * this._a * t * t * t * t * t;
    }

    /* Return the extrema of the acceleration trajectory between t1 and t2. */
    public double[] get_min_max_acc(double t1, double t2) {
        if (this._accPeakTimes[0] == 0) {
            // uninitialised: calculate the roots of the polynomial
            if (this._a == 0) {
                // solve a quadratic
                var det = this._b * this._b - 2 * this._g * this._a;
                if (det < 0) {
                    // no real roots
                    this._accPeakTimes[0] = 0;
                    this._accPeakTimes[1] = 0;
                } else {
                    this._accPeakTimes[0] = (-this._b + Math.sqrt(det)) / this._a;
                    this._accPeakTimes[1] = (-this._b - Math.sqrt(det)) / this._a;
                }
            } else {
                // _g + _b*t == 0:
                if (this._b != 0) {
                    this._accPeakTimes[0] = -this._g / this._b;
                    this._accPeakTimes[1] = 0;
                } else {
                    this._accPeakTimes[0] = 0;
                    this._accPeakTimes[1] = 0;
                }
            }
        }

        // Evaluate the acceleration at the boundaries of the period:
        double aMinOut = Math.min(this.get_acceleration(t1), this.get_acceleration(t2));
        double aMaxOut = Math.max(this.get_acceleration(t1), this.get_acceleration(t2));

        // Evaluate at the maximum/minimum times:
        for (int i : new int[] { 0, 1 }) {
            if (this._accPeakTimes[i] <= t1)
                continue;
            if (this._accPeakTimes[i] >= t2)
                continue;

            aMinOut = Math.min(aMinOut, this.get_acceleration(this._accPeakTimes[i]));
            aMaxOut = Math.max(aMaxOut, this.get_acceleration(this._accPeakTimes[i]));
        }
        return new double[] { aMinOut, aMaxOut };
    }

    /* Return the extrema of the jerk squared trajectory between t1 and t2. */
    public double get_max_jerk_squared(double t1, double t2) {
        double jMaxSqr = Math.max(Math.pow(this.get_jerk(t1), 2), Math.pow(this.get_jerk(t2), 2));

        if (this._a != 0) {
            double tMax = -this._b / this._a;
            if (tMax > t1 && tMax < t2) {
                jMaxSqr = Math.max(Math.pow(this.get_jerk(tMax), 2), jMaxSqr);
            }
        }
        return jMaxSqr;
    }

    /* Return the parameter alpha which defines the trajectory. */
    public double get_param_alpha() {
        return this._a;
    }

    /* Return the parameter beta which defines the trajectory. */
    public double get_param_beta() {
        return this._b;
    }

    /* Return the parameter gamma which defines the trajectory. */
    public double get_param_gamma() {
        return this._g;
    }

    /* Return the start acceleration of the trajectory. */
    public double get_initial_acceleration() {
        return this._a0;
    }

    /* Return the start velocity of the trajectory. */
    public double get_initial_velocity() {
        return this._v0;
    }

    /* Return the start position of the trajectory. */
    public double get_initial_position() {
        return this._p0;
    }

    /* Return the total cost of the trajectory. */
    public double get_cost() {
        return this._cost;
    }
}