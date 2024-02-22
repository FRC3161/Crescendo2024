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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LightsConstants.LightsType;
import frc.robot.Constants.SwerveConstants.DriveMode;
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
    driver.povLeft().whileTrue(new SnapTo(s_Swerve, SnapMode.LEFT));
    driver.povRight().whileTrue(new SnapTo(s_Swerve, SnapMode.RIGHT));
    driver.povUp().whileTrue(new SnapTo(s_Swerve, SnapMode.FORWARD));
    driver.povDown().whileTrue(new SnapTo(s_Swerve, SnapMode.BACKWARD));
    driver.back().onTrue(new InstantCommand(s_Swerve::zeroGyro));
    driver.b().whileTrue(
        new ProxyCommand(() -> Tags.DriveToClosestTag(new Transform2d(1, 0, new Rotation2d()), s_Swerve)));
    /* Operator Controller */
    operator.y().onTrue(new ToAngle(() -> Units.degreesToRadians(30), arm));
    // operator.button(3).onTrue(new ToAngle(() -> Units.degreesToRadians(10),
    // arm));
    // operator.leftBumper().whileTrue(new ParallelCommandGroup(new ToRPM(() ->
    // 4500, shooter), new FeedIn(feeder)));
    operator.rightBumper().whileTrue(new IntakeIn(intake));
    operator.leftBumper().whileTrue(new Outake(intake));
    operator.rightTrigger().whileTrue(new ToRPM(() -> 4500, shooter));
    operator.leftTrigger().whileTrue(new FeedIn(feeder));

    operator.x().whileTrue(new ClimbExtend(climber));
    operator.a().whileTrue(new ClimbRetract(climber));

    // operator.a().whileTrue(new FeedIn(feeder));
    operator.b().whileTrue(new FeedOut(feeder));
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
    SmartDashboard.putData("Shooter test command", new ToRPM(() -> 4500, shooter));

    SmartDashboard.putData("Feed IN", new FeedIn(feeder));
    SmartDashboard.putData("Feed OUT", new FeedOut(feeder));

    SmartDashboard.putData("Reset Pose", new InstantCommand(() -> {
      s_Swerve.setPose(new Pose2d());
    }));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
