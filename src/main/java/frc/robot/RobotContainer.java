// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Arm.ManualArm;
import frc.robot.commands.Arm.ToAngle;
import frc.robot.commands.Arm.ToDistanceAngle;
import frc.robot.commands.Climber.ClimbExtend;
import frc.robot.commands.Climber.ClimbRetract;
import frc.robot.commands.Drive.Tags;
import frc.robot.commands.Drive.DriveToLocation;
import frc.robot.commands.Drive.SnapTo;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Drive.SnapTo.SnapMode;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.Outake;
import frc.robot.commands.Intake.beamMessage;
import frc.robot.commands.Lights.SolidColor;
import frc.robot.commands.Shooter.ShootFeed;
import frc.robot.commands.Shooter.FeedIn;
import frc.robot.commands.Shooter.FeedOut;
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
  }

  public void teleopInit() {
    lights.clearBuffer();
  }

  public void disabledInit() {
    lights.colors = new int[] { 255, 0, 0 };
  }

  private void configureButtonBindings() {
    /* Driver Controller */
    s_Swerve.setDefaultCommand(new TeleopSwerve(
        s_Swerve,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX()));
    driver.x().whileTrue(new SnapTo(s_Swerve, SnapMode.LEFT));
    driver.b().whileTrue(new SnapTo(s_Swerve, SnapMode.RIGHT));
    driver.y().whileTrue(new SnapTo(s_Swerve, SnapMode.FORWARD));
    driver.a().whileTrue(new SnapTo(s_Swerve, SnapMode.BACKWARD));
    driver.povLeft().whileTrue(new SnapTo(s_Swerve, SnapMode.SPEAKER, true));
    driver.back().onTrue(new InstantCommand(s_Swerve::zeroGyro));

    driver.povDown().whileTrue(
        new ProxyCommand(() -> Tags.DriveToClosestTag(new Transform2d(5, 0, new Rotation2d()), s_Swerve)));
    driver.povRight().whileTrue(
        new ProxyCommand(
            DriveToLocation.driveTo(new Pose2d(1.5768705606460571, 4.524226188659668, new Rotation2d()), s_Swerve)));
    driver.leftBumper().whileTrue(new SequentialCommandGroup(
        new SolidColor(lights, new int[] { 255, 0, 0 }),
        new ToAngle(() -> Units.degreesToRadians(15), arm),
        new ParallelCommandGroup(
            new FeedIn(feeder).deadlineWith(new IntakeIn(intake)),
            new SequentialCommandGroup(
                new beamMessage(intake),
                new SolidColor(lights, new int[] { 0, 255, 0 }))),
        new SolidColor(lights, new int[] { 0, 0, 255 })));

    /* Operator Controller */

    // operator.y().whileTrue(new SequentialCommandGroup(
    // new ToRPM(() -> -4000, shooter)));
    operator.y().whileTrue(new SolidColor(lights, new int[] { 255, 10, 255 }));

    operator.rightBumper().whileTrue(new IntakeIn(intake));
    operator.rightTrigger().onTrue(new SequentialCommandGroup(
        new ToRPM(() -> 4700, shooter),
        new ShootFeed(feeder).withTimeout(1),
        new ToRPM(() -> 400, shooter)));
    // shooter.setDefaultCommand(new ToRPM(() -> 0, shooter));

    operator.leftTrigger().whileTrue(new Outake(intake));
    operator.b().whileTrue(new SequentialCommandGroup(
        new SolidColor(lights, new int[] { 255, 0, 0 }),
        new ToAngle(() -> Units.degreesToRadians(10), arm),
        new SolidColor(lights, new int[] { 0, 0, 255 }),
        new FeedOut(feeder)));

    arm.setDefaultCommand(new ManualArm(() -> operator.getLeftY(), arm));
    // lights.setDefaultCommand(new SolidColor(LightsType.IDLE, 0, lights, new int[]
    // {100, 0, 300} ));

    operator.x().whileTrue(new ClimbExtend(climber));
    // operator.x().onTrue(new ToAngle(() -> Units.degreesToRadians(80), arm)); <-
    // max arm angle
    operator.a().whileTrue(new ClimbRetract(climber));

    driver.povUp().onTrue(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ToRPM(() -> 4700, shooter),
                new SnapTo(s_Swerve, SnapMode.SPEAKER)),
            new ShootFeed(feeder).withTimeout(1),
            new ToRPM(() -> 2000, shooter))
            .deadlineWith(new ToDistanceAngle(s_Swerve, arm)));

    /* Subwoofer shot */
    operator.leftBumper().onTrue(new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SolidColor(lights, new int[] { 150, 0, 0 }),
            new ToAngle(() -> Units.degreesToRadians(48.5), arm),
            new ToRPM(() -> 4500, shooter)),
        new SolidColor(lights, new int[] { 0, 150, 0 }),
        new ShootFeed(feeder).withTimeout(1),
        new ParallelCommandGroup(
            new ToRPM(() -> 400, shooter),
            new ToAngle(() -> Units.degreesToRadians(10), arm))));

  }

  public void configureNamedCommands() {
    // EXAMPLE
    // NamedCommands.registerCommand("shoot", new ShooterDistance(drive, shooter));
    // NamedCommands.registerCommand("intake", new IntakeIn(intake));
    // NamedCommands.registerCommand("intake", new Outake(intake));
    NamedCommands.registerCommand("shoot",
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SnapTo(s_Swerve, SnapMode.SPEAKER),
                new SolidColor(lights, new int[] { 150, 0, 0 }),
                new ToRPM(() -> 4500, shooter),
                new ToDistanceAngle(s_Swerve, arm)),
            new SolidColor(lights, new int[] { 0, 150, 0 }),
            new ShootFeed(feeder).withTimeout(0.5)));
  }

  public void configureAutoCommands() {
    m_chooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("autos", m_chooser);
  }

  public void configureTestCommands() {
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

    SmartDashboard.putData("Reset Pose", new InstantCommand(() -> {
      s_Swerve.setPose(new Pose2d());
    }));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
