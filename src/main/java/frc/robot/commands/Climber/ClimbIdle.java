package frc.robot.commands.Climber;

import frc.robot.Constants.ClimberConstants.ClimberMode;
import frc.robot.subsystems.Climber.Climber;

public class ClimbIdle extends ClimberBase {
  public ClimbIdle(Climber climber) {
    super(climber, ClimberMode.IDLE);
  }
}
