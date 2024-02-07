package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// degrees
public class GyroPigeon2 {
  private final Pigeon2 gyro;

  private double pitchOffset = 0;
  private double yawOffset = 0;
  private double rollOffset = 0;

  public GyroPigeon2(int id) {
    gyro = new Pigeon2(id);

    gyro.getYaw().setUpdateFrequency(100);
    gyro.getPitch().setUpdateFrequency(100);
    gyro.getRoll().setUpdateFrequency(100);
    gyro.optimizeBusUtilization();
  }

  public double getPitch() {
    return gyro.getPitch().getValueAsDouble() - pitchOffset;
  }

  public double getRoll() {
    return gyro.getRoll().getValueAsDouble() - rollOffset;
  }

  public double getYaw() {
    return gyro.getYaw().getValueAsDouble() - yawOffset;
  }

  public void logValues() {
    SmartDashboard.putNumber("Gyro pitch", getPitch());
    SmartDashboard.putNumber("Gyro yaw", getYaw());
    SmartDashboard.putNumber("Gyro roll", getRoll());
  }

  public void home() {
    pitchOffset = gyro.getPitch().getValueAsDouble();
    yawOffset = gyro.getYaw().getValueAsDouble();
    rollOffset = gyro.getRoll().getValueAsDouble();
  }

  public void homeYaw() {
    yawOffset = gyro.getYaw().getValueAsDouble();
  }

  public void homeRoll() {
    rollOffset = gyro.getRoll().getValueAsDouble();
  }

  public void homePitch() {
    pitchOffset = gyro.getPitch().getValueAsDouble();
  }
}
