package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.Climber;

public class ClimbExtend extends Command {
    protected final Climber m_climber;

    public ClimbExtend(Climber climber){
        m_climber = climber;
    }

    @Override
    public void initialize(){
        m_climber.climbExtend(Constants.ClimberConstants.retractSpeed);
    }

    @Override 
    public void end(boolean intrerrupted){
        m_climber.stop();
    }
}