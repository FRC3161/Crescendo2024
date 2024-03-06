package frc.robot.commands.Climber;

import frc.robot.Constants.ClimberConstants.ClimberMode;
import frc.robot.subsystems.Climber.Climber;

public class ClimbHome extends ClimberBase {
  public ClimbHome(Climber climber) {
    super(climber, ClimberMode.HOMING);
  }
}
