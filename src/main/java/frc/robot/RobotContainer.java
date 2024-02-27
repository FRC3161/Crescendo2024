// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.Constants.LightsConstants.LightsType;
import frc.robot.Constants.SwerveConstants.DriveMode;
import frc.robot.commands.Arm.ManualArm;
import frc.robot.commands.Arm.ToAngle;
import frc.robot.commands.Climber.ClimbExtend;
import frc.robot.commands.Climber.ClimbRetract;
import frc.robot.commands.Drive.Tags;
import frc.robot.commands.Drive.SnapTo;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Drive.SnapTo.SnapMode;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.Outake;
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
    configureAutoCommands();
    configureTestCommands();
  }

  public void disabledActions() {
    arm.resetI();
    shooter.resetI();

  }

  public void disabledInit() {
    lights.clearQueue();
    new SolidColor(LightsType.DISABLED, 0, lights, new int[] { 255, 0, 0 }).schedule();
  }

  private void configureButtonBindings() {
      /* Driver Controller */
    s_Swerve.setDefaultCommand(new TeleopSwerve(
        s_Swerve,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX()));
    driver.x().onTrue(new SnapTo(s_Swerve, SnapMode.LEFT));
    driver.b().onTrue(new SnapTo(s_Swerve, SnapMode.RIGHT));
    driver.y().onTrue(new SnapTo(s_Swerve, SnapMode.FORWARD));
    driver.a().onTrue(new SnapTo(s_Swerve, SnapMode.BACKWARD));
    driver.back().onTrue(new InstantCommand(s_Swerve::zeroGyro));
    driver.povDown().whileTrue(
        new ProxyCommand(() -> Tags.DriveToClosestTag(new Transform2d(5, 0, new Rotation2d()), s_Swerve)));
    driver.leftBumper().whileTrue(new SequentialCommandGroup(
      new ToAngle(() -> Units.degreesToRadians(15), arm),
      new ParallelCommandGroup(
        new FeedIn(feeder),
        new IntakeIn(intake)
      )
      ));
        /* Operator Controller */

    operator.y().whileTrue(new SequentialCommandGroup(
      new ToRPM(() -> -4000, shooter)));

    operator.rightBumper().whileTrue(new IntakeIn(intake));
    

    operator.rightTrigger().onTrue(new SequentialCommandGroup(
      new ToRPM(() -> 4700, shooter),
      new ShootFeed(feeder).withTimeout(1),
      new ToRPM(() -> 400, shooter)));
    // shooter.setDefaultCommand(new ToRPM(() -> 0, shooter));

    operator.leftTrigger().whileTrue(new FeedIn(feeder));
    operator.b().whileTrue(new ShootFeed(feeder));
    
    arm.setDefaultCommand(new ManualArm(() -> operator.getLeftY(), arm));

    // operator.x().whileTrue(new ClimbExtend(climber));
    operator.x().onTrue(new ToAngle(() -> Units.degreesToRadians(80), arm));
    operator.a().whileTrue(new ClimbRetract(climber));
    /* Subwoofer shot */
    operator.leftBumper().onTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ToAngle(() -> Units.degreesToRadians(48.5), arm),
        new ToRPM(()-> 4500, shooter)),
      new ShootFeed(feeder).withTimeout(1),
      new ParallelCommandGroup(
        new ToRPM(() -> 400, shooter),
        new ToAngle(() -> Units.degreesToRadians(10), arm)
          )));
    // operator.b().whileTrue(new SolidColor(null, 0, lights, null)); Ignore this
    // please :)
    
    

  }

  public void configureAutoCommands() {
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
