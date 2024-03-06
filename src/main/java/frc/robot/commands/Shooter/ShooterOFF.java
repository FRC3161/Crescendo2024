package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

public class ShooterOFF extends Command {
  private final Shooter shooter;

  public ShooterOFF(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    this.shooter.runVelocity(0, 0);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
