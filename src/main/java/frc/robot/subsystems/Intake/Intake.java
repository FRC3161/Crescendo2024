package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;

import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;
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
  private int stallLim = 20;
  private int freeLim = 20;

  public Intake() {
    leader.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(leader, Usage.kMinimal);
    leader.setSmartCurrentLimit(stallLim, freeLim);
    leader.setIdleMode(IdleMode.kCoast);
    leader.enableVoltageCompensation(12);
    leader.setInverted(false);

    follower.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(follower, Usage.kMinimal);
    follower.setSmartCurrentLimit(stallLim, freeLim);
    follower.setIdleMode(IdleMode.kCoast);
    follower.enableVoltageCompensation(12);
    leader.setInverted(false);

  }

  public void burnToFlash() {
    leader.burnFlash();
    follower.burnFlash();
  } 
  
  // public void setLimit(Boolean button){
  //   if(!button){
  //     freeLim = 15;
  //     stallLim = 15;
  //   } else {
  //     freeLim = 20;
  //     stallLim = 20;

  //   } }

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
    SmartDashboard.putNumber("plswork", freeLim);
    SmartDashboard.putNumber("prayer", stallLim);
  }

  public void stop() {
    leader.set(0);
    follower.set(0);
  }

}
