package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private CANSparkMax leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.leftMotorID,
      MotorType.kBrushless);
  private CANSparkMax rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.rightMotorID,
      MotorType.kBrushless);

  private RelativeEncoder climberEncoderLeft = leftClimberMotor.getEncoder();
  private RelativeEncoder climberEncoderRight = rightClimberMotor.getEncoder();

  // private final SparkPIDController pid;

  public Climber() {
    setupMotors();

    // pid = leftClimberMotor.getPIDController();
    // pid.setP(Constants.ClimberConstants.pid[0]);
    // pid.setP(Constants.ClimberConstants.pid[1]);
    // pid.setP(Constants.ClimberConstants.pid[2]);

  }

  private void setupMotors() {
    leftClimberMotor.enableVoltageCompensation(12);
    leftClimberMotor.setSmartCurrentLimit(60, 40);
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    leftClimberMotor.setInverted(true);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(leftClimberMotor, Usage.kPositionOnly);

    rightClimberMotor.enableVoltageCompensation(12);
    rightClimberMotor.setSmartCurrentLimit(60, 40);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setInverted(true);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(rightClimberMotor, Usage.kPositionOnly);

    rightClimberMotor.follow(leftClimberMotor, true);
  }

  public void burnToFlash() {
    leftClimberMotor.burnFlash();
    rightClimberMotor.burnFlash();
  }

  public void climbExtend(double speed) {
    leftClimberMotor.set(speed);
  }

  public void climbRetract(double speed) {
    leftClimberMotor.set(-speed);
  }

  public void set(double power) {
    leftClimberMotor.set(power);
  }

  public void stop() {
    leftClimberMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Actual Climber Left", climberEncoderLeft.getPosition());
    SmartDashboard.putNumber("Actual Climber Right", climberEncoderRight.getPosition());
  }
}
