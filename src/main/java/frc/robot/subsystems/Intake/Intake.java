package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax leader = new CANSparkMax(41, MotorType.kBrushless);
  private CANSparkMax follower = new CANSparkMax(42, MotorType.kBrushless);

  public Intake() {
    leader.setSmartCurrentLimit(40);
    leader.setIdleMode(IdleMode.kBrake);
    leader.enableVoltageCompensation(12);

    follower.setSmartCurrentLimit(40);
    follower.setIdleMode(IdleMode.kBrake);
    follower.enableVoltageCompensation(12);

    follower.follow(leader, false);
  }

  public void setSpeed(double value) {
    leader.set(value);
  }

  public void stop() {
    leader.set(0);
  }
}
