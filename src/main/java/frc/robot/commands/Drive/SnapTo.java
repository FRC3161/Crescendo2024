package frc.robot.commands.Drive;

import javax.naming.InitialContext;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.DriveMode;
import frc.robot.subsystems.Drive.Swerve;

public class SnapTo extends Command {
  private Timer m_timer = new Timer();
  private final Swerve m_drive;
  private final SnapMode m_snapeMode;
  private TrapezoidProfile.State initialState;
  private TrapezoidProfile m_profiler = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
          Units.degreesToRadians(400),
          Units.degreesToRadians(500)));

  public static enum SnapMode {
    SPEAKER,
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD
  }

  public SnapTo(Swerve drive, SnapMode mode) {
    m_drive = drive;
    m_snapeMode = mode;
  }

  @Override
  public void initialize() {
    initialState = new TrapezoidProfile.State(m_drive.getYawForSnap().getRadians(), 0);
    m_drive.setDriveMode(DriveMode.Snap);

    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    double setpoint = 0;
    switch (m_snapeMode) {
      case SPEAKER:
        setpoint = m_drive.getRotationRelativeToSpeaker().getRadians();
        break;
      case LEFT:
        setpoint = Units.degreesToRadians(90);
        break;
      case RIGHT:
        setpoint = Units.degreesToRadians(270);
        break;
      case FORWARD:
        setpoint = 0;
        break;
      case BACKWARD:
        setpoint = Math.PI;
        break;
      default:
        break;
    }

    if (Math.abs(setpoint - initialState.position) >= Math.PI) {
      if (setpoint < initialState.position) {
        setpoint += Math.PI * 2;
      } else {
        initialState.position = Math.PI * 2 + initialState.position;
      }
    }

    var nextState = m_profiler.calculate(m_timer.get(), initialState, new TrapezoidProfile.State(setpoint, 0));
    m_drive.setSnapSetpoint(new Rotation2d(nextState.position));
  }

  @Override
  public boolean isFinished() {
    return m_profiler.isFinished(m_timer.get()) && m_drive.isSnapAtSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_drive.setDriveMode(DriveMode.DriverInput);
  }
}
