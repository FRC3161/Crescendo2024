package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;

public class ManualArm extends Command {
  private final DoubleSupplier m_angleSupplier;
  private final Arm m_arm;

  public ManualArm(DoubleSupplier angleSupplier, Arm arm) {
    m_angleSupplier = angleSupplier;
    m_arm = arm;

    this.addRequirements(m_arm);
  }

  @Override
  public void execute() {
    double armY = MathUtil.applyDeadband(this.m_angleSupplier.getAsDouble(), 0.3);
    // double armY = this.m_angleSupplier.getAsDouble();

    double armChange = (50 / 50) * armY;
    this.m_arm.runSetpoint(new Rotation2d(
      Units.degreesToRadians(armChange + m_arm.getSetpoint().getDegrees())
    ));
  }
}
