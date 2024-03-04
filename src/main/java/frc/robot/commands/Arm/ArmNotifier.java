package frc.robot.commands.Arm;

import javax.naming.InsufficientResourcesException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class ArmNotifier extends Command {
  private Arm arm;

  public ArmNotifier(Arm arm) {
    this.arm = arm;
  }

  @Override
  public boolean isFinished() {
    return arm.atGoal();
  }
}
