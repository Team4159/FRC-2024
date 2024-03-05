// package frc.robot;

// import com.revrobotics.CANSparkBase;
// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// public class REVConfigs {
//     public CANSparkBase intakeAngleMotor, shooterAngleMotor, shooterVelocityMotor;
    
//     public REVConfigs() {
//         intakeAngleMotor = new CANSparkFlex(Constants.Intake.angleMotorID, MotorType.kBrushless);
//         shooterAngleMotor = new CANSparkFlex(Constants.Shooter.angleMotorID, MotorType.kBrushless);
//         shooterVelocityMotor = new CANSparkFlex(Constants.Shooter.shooterMLeftID, MotorType.kBrushless);

//         intakeAngleMotor.getPIDController().setP(Constants.PIDConstants.IntakeAngle.kP);
//         intakeAngleMotor.getPIDController().setI(Constants.PIDConstants.IntakeAngle.kI);
//         intakeAngleMotor.getPIDController().setD(Constants.PIDConstants.IntakeAngle.kD);

//         shooterAngleMotor.getPIDController().setP(Constants.PIDConstants.ShooterAngle.kP);
//         shooterAngleMotor.getPIDController().setI(Constants.PIDConstants.ShooterAngle.kI);
//         shooterAngleMotor.getPIDController().setD(Constants.PIDConstants.ShooterAngle.kD);

//         shooterVelocityMotor.getPIDController().setP(Constants.PIDConstants.ShooterVelocity.kP);
//         shooterVelocityMotor.getPIDController().setI(Constants.PIDConstants.ShooterVelocity.kI);
//         shooterVelocityMotor.getPIDController().setD(Constants.PIDConstants.ShooterVelocity.kD);
//     }
// }
