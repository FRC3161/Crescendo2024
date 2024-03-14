package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IntakeConstants.IntakeMode;

public class Intake extends SubsystemBase {
  private CANSparkMax leader = new CANSparkMax(41, MotorType.kBrushless);
  private CANSparkMax follower = new CANSparkMax(42, MotorType.kBrushless);
  private DigitalInput beamy = ShooterConstants.beam;
  public DigitalInput beamy2 = new DigitalInput(7);
  private IntakeMode intakeMode = IntakeMode.OFF;
  private double value = 0.7;

  public Intake() {
    leader.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(leader, Usage.kMinimal);
    leader.setSmartCurrentLimit(20, 20);
    leader.setIdleMode(IdleMode.kBrake);
    leader.enableVoltageCompensation(12);
    leader.setInverted(false);

    follower.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(follower, Usage.kMinimal);
    follower.setSmartCurrentLimit(20, 20);
    follower.setIdleMode(IdleMode.kBrake);
    follower.enableVoltageCompensation(12);
    leader.setInverted(false);

  }

  public void burnToFlash() {
    leader.burnFlash();
    follower.burnFlash();
  }

  public void setSpeed(double value) {
    if (beamy.get()) {
      leader.set(value);
      follower.set(value);
    } else {
      leader.set(0);
      follower.set(0);
    }
  }

  public void setOutspeed(double value) {
    leader.set(value);
    follower.set(value);
  }

  public void setIntakeMode(IntakeMode mode) {
    intakeMode = mode;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("beamy2", beamy2.get());
  }

  public void stop() {
    leader.set(0);
    follower.set(0);
  }

}
