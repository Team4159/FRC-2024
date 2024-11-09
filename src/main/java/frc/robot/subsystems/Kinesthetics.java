package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.RobotState;
import frc.robot.Constants;

public class Kinesthetics extends SubsystemBase {
    // Subsystem Information
    private CommandSwerveDrivetrain s_Swerve;

    // Sensor Information
    //private Pigeon2 gyro;
    private DigitalInput shooterBeamBreak;
    private Debouncer shooterBeamBreakDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kRising);

    //private AnalogGyro analogGyro = new AnalogGyro(0);
    //private AnalogGyroSim gyroSim = new AnalogGyroSim(analogGyro);

    /** @param velocityOmega degrees / second */
    private StatusSignal<Double> velocityOmega;
    
    // Data Fields
    //private SwerveDrivePoseEstimator poseEstimator;

    // Shuffleboard
    //private final Field2d field = new Field2d();
    //private final StructArrayPublisher<SwerveModuleState> swerveStates = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

    public Kinesthetics(CommandSwerveDrivetrain s) {
        s_Swerve = s;
        //s_Swerve.setKinesthetics(this);
        
        shooterBeamBreak = new DigitalInput(Constants.Shooter.beamBreakID);
    
        ShuffleboardTab table = Shuffleboard.getTab("Kinesthetics");

        table.addBoolean("Shooter Note?", this::shooterHasNote);

        //ChoreoTrajectory traj = Choreo.getTrajectory("Test_Traj");

        // field.getObject("traj").setPoses(
        //     traj.getInitialPose(), traj.getFinalPose()
        // );
        // field.getObject("trajPoses").setPoses(
        //     traj.getPoses()
        // );
    }

    // Public Getters & Setters
    public boolean shooterHasNote() {
        return shooterBeamBreakDebouncer.calculate(!shooterBeamBreak.get());
    }
}
