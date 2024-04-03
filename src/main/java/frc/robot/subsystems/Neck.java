package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

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
    
    public class ChangeNeck extends Command {
        private Kinesthetics kinesthetics;

        private boolean beamBreakMode; // should this command end when the beam break opens
        private SpinState desiredNeck;
        private boolean desiredSlow;

        public ChangeNeck(SpinState ss) {
            this(ss, false);
        }

        public ChangeNeck(SpinState ss, boolean slow) {
            desiredNeck = ss;
            desiredSlow = slow;
            addRequirements(Neck.this);
        }

        public ChangeNeck(Kinesthetics k, SpinState ss) {
            kinesthetics = k;
            desiredNeck = ss;
            beamBreakMode = kinesthetics.shooterHasNote();
            addRequirements(Neck.this);
        }

        @Override
        public void initialize() {
            setNeck(desiredNeck, desiredSlow ? 0.5 : 1);
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
