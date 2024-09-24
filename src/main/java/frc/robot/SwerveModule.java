package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);

    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    private TalonFXSimState driveSimState;

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    private TalonFXSimState angleSimState;

    private boolean simulation;

    private final DCMotorSim driveMotorSimModel =
    new DCMotorSim(DCMotor.getKrakenX60Foc(1), Constants.Swerve.driveGearRatio, 0.001);

    private final DCMotorSim angleMotorSimModel =
    new DCMotorSim(DCMotor.getKrakenX60Foc(1), Constants.Swerve.angleGearRatio, 0.001);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this(moduleNumber, moduleConstants, false);
    }

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, boolean s){
        simulation = s;
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, Constants.Swerve.canBus);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
        anglePosition.UpdateFreqHz = 50;

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.Swerve.canBus);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.Swerve.canBus);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
        driveVelocity.UpdateFreqHz = 50;
        if(simulation){
            driveSimState = mDriveMotor.getSimState();
            angleSimState = mAngleMotor.getSimState();

            driveSimState.setSupplyVoltage(12.5);
            angleSimState.setSupplyVoltage(12.5);
        }
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        if(simulation){
            var dMotorVoltage = driveSimState.getMotorVoltage();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            driveMotorSimModel.setInputVoltage(dMotorVoltage);
            driveMotorSimModel.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            driveSimState.setRawRotorPosition(
                Constants.Swerve.driveGearRatio * driveMotorSimModel.getAngularPositionRotations()
            );
            driveSimState.setRotorVelocity(
                Constants.Swerve.driveGearRatio * Units.radiansToRotations(driveMotorSimModel.getAngularVelocityRadPerSec())
            );
            var aMotorVoltage = angleSimState.getMotorVoltage();

            // use the motor voltage to calculate new position and velocity
            // using WPILib's DCMotorSim class for physics simulation
            angleMotorSimModel.setInputVoltage(aMotorVoltage);
            angleMotorSimModel.update(0.020); // assume 20 ms loop time

            // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            angleSimState.setRawRotorPosition(
                Constants.Swerve.angleGearRatio * angleMotorSimModel.getAngularPositionRotations()
            );
            angleSimState.setRotorVelocity(
                Constants.Swerve.angleGearRatio * Units.radiansToRotations(angleMotorSimModel.getAngularVelocityRadPerSec())
            );
        }
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition( // FIXME WHY IS THIS NEGATIVE
            Conversions.rotationsToMeters(-mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public double getDriveCurrent() {
        return mDriveMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getAngleCurrent() {
        return mAngleMotor.getSupplyCurrent().getValueAsDouble();
    }
}