package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants.ClimberMode;
import frc.robot.subsystems.Climber.Climber;

public class ClimberBase extends Command {
  protected Climber climber;
  protected ClimberMode mode;

  public ClimberBase(Climber climber, ClimberMode mode) {
    this.climber = climber;
    this.mode = mode;

    this.addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setMode(mode);
  }

  @Override
  public boolean isFinished() {
    return climber.isDone();
  }

  @Override
  public void end(boolean interrupted) {
    climber.setMode(ClimberMode.IDLE);
  }
}
