package frc.robot.commands.Climber;

import frc.robot.Constants.ClimberConstants.ClimberMode;
import frc.robot.subsystems.Climber.Climber;

public class ClimbDeploy extends ClimberBase {
  public ClimbDeploy(Climber climber) {
    super(climber, ClimberMode.DEPLOY);
  }
}
