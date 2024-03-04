package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;

public class ToAngle extends Command {
  private Timer m_timer = new Timer();
  private final DoubleSupplier m_angleSupplier;
  protected final Arm m_arm;
  private State initialState;
  private TrapezoidProfile m_profiler = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
          Constants.ArmConstants.maxVelocityPerSecond.getRadians(),
          Constants.ArmConstants.maxAcceleration.getRadians()));

  public ToAngle(DoubleSupplier targetAngle, Arm arm) {
    m_angleSupplier = targetAngle;
    m_arm = arm;

    this.addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    initialState = m_arm.getCurrenState();
    m_arm.setGoal(Rotation2d.fromRadians(m_angleSupplier.getAsDouble()));
  }

  @Override
  public void execute() {
    var nextState = m_profiler.calculate(m_timer.get(),
        initialState,
        new TrapezoidProfile.State(m_angleSupplier.getAsDouble(), 0));

    m_arm.setGoal(Rotation2d.fromRadians(m_angleSupplier.getAsDouble()));

    m_arm.runState(nextState);
  }

  @Override
  public boolean isFinished() {
    // return m_profiler.isFinished(m_timer.get()) && m_arm.atSetpoint();
    return m_profiler.isFinished(m_timer.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

}
