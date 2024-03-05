// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LightsConstants;
import frc.robot.commands.Arm.ArmNotifier;
import frc.robot.commands.Arm.ManualArm;
import frc.robot.commands.Arm.ToAngle;
import frc.robot.commands.Arm.ToDistanceAngle;
import frc.robot.commands.Arm.ToDistanceAngle.ArmEndBehaviour;
import frc.robot.commands.Climber.ClimberManual;
import frc.robot.commands.Drive.DriveToLocation;
import frc.robot.commands.Drive.SnapNotifier;
import frc.robot.commands.Drive.SnapTo;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Drive.SnapTo.EndBehaviour;
import frc.robot.commands.Drive.SnapTo.SnapMode;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.Outake;
import frc.robot.commands.Intake.beamMessage;
import frc.robot.commands.Lights.SolidColor;
import frc.robot.commands.Shooter.ShootFeed;
import frc.robot.commands.Shooter.FeedIn;
import frc.robot.commands.Shooter.FeedInNotifier;
import frc.robot.commands.Shooter.FeedOut;
import frc.robot.commands.Shooter.FeedSource;
import frc.robot.commands.Shooter.ToRPM;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Shooter.Shooter;

public class RobotContainer {

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(Constants.Operators.driver);
  private final CommandXboxController operator = new CommandXboxController(Constants.Operators.operator);
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  /* Subsystems */
  final Swerve s_Swerve = new Swerve();
  final Lights lights = new Lights();
  final Intake intake = new Intake();
  final Shooter shooter = new Shooter();
  final Feeder feeder = new Feeder();
  final Arm arm = new Arm();
  final Climber climber = new Climber();

  public RobotContainer() {
    configureButtonBindings();
    configureNamedCommands();
    configureAutoCommands();
    configureTestCommands();

  }

  public void disabledActions() {
    arm.resetI();
    shooter.resetI();
    feeder.resetI();
    arm.runState(new TrapezoidProfile.State(arm.getEncoderPosition().getRadians(), 0));
  }

  public void teleopInit() {
    lights.clearBuffer();
  }

  public void disabledInit() {
    lights.colors = Constants.LightsConstants.Colors.RED;
  }

  public Command getIdleCommands() {
    if (!feeder.beamy.get()) {
      return new ParallelCommandGroup(
          new ToAngle(() -> Constants.ArmConstants.min.getRadians(), arm),
          new ToRPM(() -> 3000, shooter),
          SnapTo.resetToDriverInput(s_Swerve));
    } else {
      return new ParallelCommandGroup(
          new ToAngle(() -> Constants.ArmConstants.min.getRadians(), arm),
          new ToRPM(() -> 0, shooter),
          SnapTo.resetToDriverInput(s_Swerve));
    }

  }

  public void idle() {
    getIdleCommands().schedule();
  }

  private void configureButtonBindings() {
    // default commands
    arm.setDefaultCommand(new ManualArm(() -> operator.getLeftY(), arm));
    climber.setDefaultCommand(
        new ClimberManual(climber, operator::getRightY));
    /* Driver Controller */
    s_Swerve.setDefaultCommand(new TeleopSwerve(
        s_Swerve,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX(),
        () -> driver.getRightTriggerAxis()));

    driver.x().whileTrue(new SnapTo(s_Swerve, SnapMode.LEFT));
    driver.b().whileTrue(new SnapTo(s_Swerve, SnapMode.RIGHT));
    driver.y().whileTrue(new SnapTo(s_Swerve, SnapMode.FORWARD));
    driver.a().whileTrue(new SnapTo(s_Swerve, SnapMode.BACKWARD));
    driver.povDown().whileTrue(new Outake(intake));
    // driver.povLeft().whileTrue(new SnapTo(s_Swerve, SnapMode.SPEAKER, true));
    driver.back().onTrue(new InstantCommand(s_Swerve::zeroGyro));

    driver.povRight().whileTrue(
        new ProxyCommand(
            DriveToLocation.driveTo(new Pose2d(1.5768705606460571, 6.266633987426758, new Rotation2d()), s_Swerve)));
    driver.rightBumper().whileTrue(new SequentialCommandGroup(
        new SolidColor(lights, Constants.LightsConstants.Colors.RED),
        new ToAngle(() -> Units.degreesToRadians(15), arm),
        new ParallelCommandGroup(
            new FeedIn(feeder).deadlineWith(new IntakeIn(intake)),
            new SequentialCommandGroup(
                new beamMessage(intake),
                new SolidColor(lights, Constants.LightsConstants.Colors.GREEN),
                new InstantCommand(() -> {
                  new ToRPM(() -> 3000, shooter).schedule();
                }))),
        new SolidColor(lights, Constants.LightsConstants.Colors.BLUE)));

    /* Operator Controller */
    operator.rightTrigger().whileTrue(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SnapNotifier(s_Swerve),
                new ArmNotifier(arm),
                new ToRPM(() -> 4700, shooter),
                new FeedIn(feeder).deadlineWith(new IntakeIn(intake))),
            new ShootFeed(feeder).withTimeout(0.7))
            .finallyDo(this::idle)
            .deadlineWith(
                new SnapTo(s_Swerve, SnapMode.SPEAKER, EndBehaviour.NEVER_ENDING),
                new ToDistanceAngle(s_Swerve, arm, ArmEndBehaviour.NEVER_ENDING)));

    operator.leftTrigger().whileTrue(new SequentialCommandGroup(
        new SolidColor(lights, Constants.LightsConstants.Colors.RED),
        new ToAngle(() -> Constants.ArmConstants.min.getRadians(), arm),
        new SolidColor(lights, Constants.LightsConstants.Colors.BLUE),
        new FeedOut(feeder)));

    driver.leftBumper().whileTrue(
        new SequentialCommandGroup(
            new SolidColor(lights, Constants.LightsConstants.Colors.RED),
            new ParallelCommandGroup(
                new ToAngle(() -> Constants.ArmConstants.min.getRadians(), arm),
                new FeedSource(feeder)),
            new SolidColor(lights, Constants.LightsConstants.Colors.GREEN)));
    operator.leftBumper().whileTrue(new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SolidColor(lights, Constants.LightsConstants.Colors.RED),
            new ToAngle(() -> Units.degreesToRadians(50), arm),
            new ToRPM(() -> 4500, shooter)),
        new SolidColor(lights, Constants.LightsConstants.Colors.BLUE),
        new ShootFeed(feeder).withTimeout(1),
        getIdleCommands()));

    operator.b().whileTrue(new SnapTo(s_Swerve, SnapMode.SPEAKER, EndBehaviour.NEVER_ENDING));
    operator.a().whileTrue(
        new SequentialCommandGroup(
            new SolidColor(lights, LightsConstants.Colors.RED),
            new FeedIn(feeder).deadlineWith(new IntakeIn(intake)),
            new SolidColor(lights, LightsConstants.Colors.BLUE)));

    // lights.setDefaultCommand(new SolidColor(LightsType.IDLE, 0, lights, new int[]
    // {100, 0, 300} ));

    // operator.x().whileTrue(new ClimbExtend(climber));
    // operator.x().onTrue(new ToAngle(() -> Units.degreesToRadians(80), arm)); <-
    // max arm angle ^
    // operator.a().whileTrue(new ClimbRetract(climber));

    /* Subwoofer shot */
    operator.rightBumper().onTrue(new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SolidColor(lights, Constants.LightsConstants.Colors.RED),
            new ToAngle(() -> Units.degreesToRadians(48.5), arm),
            new ToRPM(() -> 4500, shooter)),
        new SolidColor(lights, Constants.LightsConstants.Colors.BLUE),
        new ShootFeed(feeder).withTimeout(1),
        getIdleCommands()));

  }

  public void configureNamedCommands() {
    // EXAMPLE
    // NamedCommands.registerCommand("shoot", new ShooterDistance(drive, shooter));
    // NamedCommands.registerCommand("intake", new IntakeIn(intake));
    // NamedCommands.registerCommand("intake", new Outake(intake));
    NamedCommands.registerCommand("subShot", new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SolidColor(lights, Constants.LightsConstants.Colors.RED),
            new ToAngle(() -> Units.degreesToRadians(48.5), arm),
            new ToRPM(() -> 4500, shooter)),
        new SolidColor(lights, Constants.LightsConstants.Colors.BLUE),
        new ShootFeed(feeder).withTimeout(1),
        getIdleCommands()));

    NamedCommands.registerCommand("shoot", new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SnapNotifier(s_Swerve),
            new ArmNotifier(arm),
            new ToRPM(() -> 4700, shooter),
            new FeedIn(feeder).deadlineWith(new IntakeIn(intake))),
        new ShootFeed(feeder).withTimeout(0.7))
        .finallyDo(this::idle)
        .deadlineWith(
            new SnapTo(s_Swerve, SnapMode.SPEAKER_AUTO, EndBehaviour.NEVER_ENDING),
            new ToDistanceAngle(s_Swerve, arm, ArmEndBehaviour.NEVER_ENDING)));

    NamedCommands.registerCommand("Intake",
        new SequentialCommandGroup(
            new SolidColor(lights, Constants.LightsConstants.Colors.RED),
            new ToAngle(() -> Units.degreesToRadians(15), arm),
            new ParallelCommandGroup(
                new FeedIn(feeder).deadlineWith(new IntakeIn(intake)),
                new SequentialCommandGroup(
                    new beamMessage(intake),
                    new SolidColor(lights, Constants.LightsConstants.Colors.GREEN))),
            new SolidColor(lights, Constants.LightsConstants.Colors.BLUE)));

  }

  public void configureAutoCommands() {
    m_chooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("autos", m_chooser);
  }

  public void configureTestCommands() {
    SmartDashboard.putData("Suboofer", new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SolidColor(lights, Constants.LightsConstants.Colors.RED),
            new ToAngle(() -> Units.degreesToRadians(48.5), arm),
            new ToRPM(() -> 4500, shooter),
            new FeedIn(feeder).deadlineWith(new IntakeIn(intake))),
        new SolidColor(lights, Constants.LightsConstants.Colors.BLUE),
        new ShootFeed(feeder).withTimeout(1)));

    SmartDashboard.putData("Shoot", new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SnapNotifier(s_Swerve),
            new ArmNotifier(arm),
            new ToRPM(() -> 4700, shooter),
            new FeedIn(feeder).deadlineWith(new IntakeIn(intake))),
        new ShootFeed(feeder).withTimeout(0.7))
        .finallyDo(this::idle)
        .deadlineWith(
            new SnapTo(s_Swerve, SnapMode.SPEAKER, EndBehaviour.NEVER_ENDING),
            new ToDistanceAngle(s_Swerve, arm, ArmEndBehaviour.NEVER_ENDING)));

    /* Glass and SmartDashboard stuff */
    SmartDashboard.putData("Arm up", new ToAngle(() -> Units.degreesToRadians(90), arm));
    SmartDashboard.putData("Arm down", new ToAngle(() -> Units.degreesToRadians(37), arm));
    SmartDashboard.putData("Shooter test command", new SequentialCommandGroup(
        new ToRPM(() -> 4700, shooter),
        new ShootFeed(feeder).withTimeout(3),
        new ToRPM(() -> 1000, shooter)));

    SmartDashboard.putData("Feed IN", new FeedIn(feeder));
    SmartDashboard.putData("Feed OUT", new FeedOut(feeder));
    SmartDashboard.putNumber("joystick", operator.getLeftX());
    SmartDashboard.putNumber("Arm Angle", arm.getSetpoint().getDegrees());

    SmartDashboard.putData("burn to flash", new InstantCommand(() -> {
      s_Swerve.burnToFlash();
      arm.burnToFlash();
      shooter.burnToFlash();
      intake.burnToFlash();
      feeder.burnToFlash();
      climber.burnToFlash();
    }));

    SmartDashboard.putData("Reset Pose", new InstantCommand(() -> {
      s_Swerve.setPose(new Pose2d());
    }));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();

  }
}
