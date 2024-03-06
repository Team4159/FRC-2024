package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private CANSparkBase climbMotor1, climbMotor2;
    public Climber(){
        climbMotor1 = new CANSparkFlex(Constants.Climber.climbMotor1ID, MotorType.kBrushless);
        climbMotor2 = new CANSparkFlex(Constants.Climber.climbMotor2ID, MotorType.kBrushless);
        climbMotor2.follow(climbMotor1);
    }

    public void climb(){
        climbMotor1.set(Constants.Climber.climberSpeed);
    }

    public void stop(){
        climbMotor1.stopMotor();
    }
}
