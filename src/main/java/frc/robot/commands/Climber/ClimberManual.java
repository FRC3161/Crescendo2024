package frc.robot.commands.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants.ClimberMode;
import frc.robot.subsystems.Climber.Climber;

public class ClimberManual extends Command {
  protected final Climber m_climber;
  private final DoubleSupplier changeSupplier;

  public ClimberManual(Climber climber, DoubleSupplier changeSuppplier) {
    m_climber = climber;
    this.changeSupplier = changeSuppplier;

    this.addRequirements(climber);
  }

  @Override
  public void initialize() {
    m_climber.setMode(ClimberMode.MANUAL);
  }

  @Override
  public void execute() {
    double climberY = MathUtil.applyDeadband(this.changeSupplier.getAsDouble(), 0.2);
    m_climber.set(climberY);
  }

  @Override
  public void end(boolean intrerrupted) {
    m_climber.setMode(ClimberMode.IDLE);
  }
}
