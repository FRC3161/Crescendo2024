package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private CANSparkMax leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.leftMotorID,
            MotorType.kBrushless);
    private CANSparkMax rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.rightMotorID,
            MotorType.kBrushless);

    private RelativeEncoder climberEncoder = leftClimberMotor.getEncoder();

    private final SparkPIDController pid;

    public Climber() {
        setupMotors();

        pid = leftClimberMotor.getPIDController();
        pid.setP(Constants.ClimberConstants.pid[0]);
        pid.setP(Constants.ClimberConstants.pid[1]);
        pid.setP(Constants.ClimberConstants.pid[2]);

    }

    private void setupMotors() {
        leftClimberMotor.restoreFactoryDefaults();
        leftClimberMotor.enableVoltageCompensation(12);
        leftClimberMotor.setSmartCurrentLimit(60, 40);
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        leftClimberMotor.setInverted(true);

        rightClimberMotor.restoreFactoryDefaults();
        rightClimberMotor.enableVoltageCompensation(12);
        rightClimberMotor.setSmartCurrentLimit(60, 40);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setInverted(true);

        rightClimberMotor.follow(leftClimberMotor, false);

    }

    public void climbExtend(double speed) {
        leftClimberMotor.set(speed);
    }

    public void climbRetract(double speed) {
        leftClimberMotor.set(-speed);

    }

    public void stop() {
        leftClimberMotor.set(0);
    }
}
