package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.Swerve;

public class SnapNotifier extends Command {
  private Swerve drive;

  public SnapNotifier(Swerve drive) {
    this.drive = drive;
  }

  @Override
  public boolean isFinished() {
    return drive.isSnapAtGoal();
  }
}
