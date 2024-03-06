package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Kinesthetics;
import frc.robot.subsystems.Vision;
import java.lang.Math;

public class Databoard {
    private final SendableChooser<InstantCommand> pipelineSelector = new SendableChooser<>();
    private Kinesthetics kinesthetics;

    public Databoard(Kinesthetics k) {
        this.kinesthetics = k;
    }

    public void init() {
        pipelineSelector.setDefaultOption("Limelight Pipeline 0", new InstantCommand(() -> Vision.switchToPipeline(0)));
        pipelineSelector.addOption("Limelight Pipeline 1", new InstantCommand(() -> Vision.switchToPipeline(1)));

        // Add limelight pipeline selector
        Shuffleboard.getTab("Vision")
            .add("Vision", pipelineSelector)
            .withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    public void update() {
        // Display distance between odometry and limelight pose
        SmartDashboard.putNumber("Pose distance difference",
            Math.sqrt(
                Math.pow(kinesthetics.getPose().getX() - Vision.getBotPose().getX(), 2) +
                Math.pow(kinesthetics.getPose().getY() - Vision.getBotPose().getY(), 2)
            )
        );

        // Display x difference between odom and limelight
        SmartDashboard.putNumber("Pose x difference",
            Math.abs(kinesthetics.getPose().getX() - Vision.getBotPose().getX())
        );

        // Display y difference between odom and limelight
        SmartDashboard.putNumber("Pose y difference",
            Math.abs(kinesthetics.getPose().getY() - Vision.getBotPose().getY())
        );
    }

    public Command getCommand() {
        return new DataBoardCommand().ignoringDisable(true).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    private class DataBoardCommand extends Command {
        @Override
        public void initialize() {
            init();
        }

        @Override
        public void execute() {
            update();
        }
    }
}
