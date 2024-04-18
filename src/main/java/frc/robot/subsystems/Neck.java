package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SpinState;

public class Neck extends SubsystemBase {
    private CANSparkBase motorController;

    public Neck() {
        motorController = new CANSparkMax(Constants.Shooter.neckMotorID, MotorType.kBrushless);
    }

    private void setNeck(SpinState ss, double multiplier) {
        motorController.set(ss.multiplier * multiplier * Constants.Shooter.neckSpeed);
    }

    private void setNeck(SpinState ss) {
        setNeck(ss, 1);
    }
    
    public class ChangeState extends Command {
        private Kinesthetics kinesthetics;

        private boolean beamBreakMode; // should this command end when the beam break opens
        private SpinState desiredNeck;
        private boolean desiredSlow;

        public ChangeState(SpinState ss) {
            this(ss, false);
        }

        public ChangeState(SpinState ss, boolean slow) {
            desiredNeck = ss;
            desiredSlow = slow;
            addRequirements(Neck.this);
        }

        public ChangeState(Kinesthetics k, SpinState ss) {
            kinesthetics = k;
            desiredNeck = ss;
            addRequirements(Neck.this);
        }

        @Override
        public void initialize() {
            if (kinesthetics != null) {
                beamBreakMode = kinesthetics.shooterHasNote();
                if (!beamBreakMode) DriverStation.reportWarning("Smart Neck Command initialized without note", false);
            }
            setNeck(desiredNeck, desiredSlow ? 0.3 : 1);
        }

        @Override
        public boolean isFinished() {
            return !beamBreakMode || !kinesthetics.shooterHasNote();
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted || beamBreakMode) setNeck(SpinState.ST);
            super.end(interrupted);
        }
    }
}
