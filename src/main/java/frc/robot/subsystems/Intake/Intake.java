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

public class Intake extends SubsystemBase {
  private CANSparkMax leader = new CANSparkMax(41, MotorType.kBrushless);
  private CANSparkMax follower = new CANSparkMax(42, MotorType.kBrushless);
  // private DigitalInput beam = new
  // DigitalInput(Constants.ShooterConstants.beamDIO);
  private DigitalInput beamy = ShooterConstants.beam;
  public DigitalInput beamy2 = new DigitalInput(7);

  public Intake() {
    leader.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(leader, Usage.kMinimal);
    leader.setSmartCurrentLimit(40, 40);
    leader.setIdleMode(IdleMode.kBrake);
    leader.enableVoltageCompensation(12);
    leader.setInverted(false);

    follower.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(follower, Usage.kMinimal);
    follower.setSmartCurrentLimit(40, 40);
    follower.setIdleMode(IdleMode.kBrake);
    follower.enableVoltageCompensation(12);
    leader.setInverted(false);

    // follower.follow(leader, false);
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

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("beamy2", beamy2.get());
  }

  public void stop() {
    leader.set(0);
    follower.set(0);
  }
}
