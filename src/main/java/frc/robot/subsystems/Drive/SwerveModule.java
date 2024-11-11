package frc.robot.subsystems.Drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.Encoders.ModuleEncoder;
import frc.robot.subsystems.Drive.Encoders.ModuleEncoderThrifty;

public class SwerveModule {
  /* Module details */
  public int moduleNumber;

  /* Motors */
  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;
  private static int driveStall = 30;
  private static int driveFree = 30;

  /* Encoders and their values */
  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private ModuleEncoder angleEncoder;
  private double lastAngle;

  /* Controllers */
  public final SparkPIDController driveController;
  public final SparkPIDController angleController;
  public final double[] anglePID;
  public final double[] driveSVA;
  public final double[] drivePID;
  public final PIDController drivePIDController;
  public SimpleMotorFeedforward feedforward;

  // For logging
  private double driveSetpoint = 0f;
  private double angleSetpoint = 0f;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;

    this.anglePID = moduleConstants.anglePID;
    this.driveSVA = moduleConstants.driveSVA;
    this.drivePID = moduleConstants.drivePID;
    this.drivePIDController = new PIDController(drivePID[0], drivePID[1], drivePID[2]);

    /* Angle Encoder Config */
    angleEncoder = new ModuleEncoderThrifty(moduleConstants.cancoderID);
    angleEncoder.setOffset(Rotation2d.fromDegrees(moduleConstants.angleOffset));

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle.getDegrees();
  }

  public void burnToFlash() {
    driveMotor.burnFlash();
    angleMotor.burnFlash();
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // this.updateControllerValues();
    desiredState = OnboardModuleState.optimize(
        desiredState,
        getState().angle); // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which REV and CTRE are not

    this.setSpeed(desiredState);
    this.setAngle(desiredState);
  }

  public void resetToAbsolute() {
    double integratedAngleEncoderPosition = this.integratedAngleEncoder.getPosition();
    double absolutePosition = integratedAngleEncoderPosition - integratedAngleEncoderPosition % 360
        + angleEncoder.getAbsolutePosition().getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(30, 30);
    angleMotor.setInverted(true);
    angleMotor.setIdleMode(IdleMode.kBrake);
    integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
    angleController.setP(anglePID[0]);
    angleController.setI(anglePID[1]);
    angleController.setD(anglePID[2]);
    angleController.setFF(0);
    angleMotor.enableVoltageCompensation(12);
    this.resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    
    driveMotor.setSmartCurrentLimit(driveStall, driveFree);
    if (moduleNumber == 1 || moduleNumber == 3) {
      driveMotor.setInverted(true);
    } else {
      driveMotor.setInverted(false);
    }
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor);
    driveController.setP(drivePID[0]);
    driveController.setI(drivePID[1]);
    driveController.setD(drivePID[2]);
    driveController.setFF(0);
    this.feedforward = new SimpleMotorFeedforward(driveSVA[0], driveSVA[1], driveSVA[2]);
    driveMotor.enableVoltageCompensation(12);
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState) {
    driveSetpoint = desiredState.speedMetersPerSecond;

    // driveController.setReference(
    // desiredState.speedMetersPerSecond,
    // ControlType.kVelocity,
    // 0,
    // feedforward.calculate(desiredState.speedMetersPerSecond));
  }


  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle
            .getDegrees();
    angleSetpoint = angle;
    angleController.setReference(angle, ControlType.kPosition);
    lastAngle = angle;
  }

  public void logValues() {
    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " Desired Speed", driveSetpoint);
    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " Actual Speed", this.getSpeed());

    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " Desired Angle",
        angleSetpoint % 360);
    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " Actual Angle",
        angleEncoder.getAbsolutePosition().getDegrees());
  }

  public void goToHome() {
    Rotation2d angle = getAngle();
    angleController.setReference(angle.getDegrees() - angle.getDegrees() % 360,
        ControlType.kPosition);
    lastAngle = angle.getDegrees() - angle.getDegrees() % 360;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(this.integratedAngleEncoder.getPosition());
  }

  public void setLimit(boolean overdrive){
    if(overdrive){
      driveMotor.setSmartCurrentLimit(60, 60);
    } else {
      driveMotor.setSmartCurrentLimit(35, 35);
    }
  }
  public SwerveModuleState getState() {
    return new SwerveModuleState(this.getSpeed(), this.getAngle());
  }

  public double getSpeed() {
    return this.driveEncoder.getVelocity();
  }

  public double getDistance() {
    return this.driveEncoder.getPosition();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(this.getDistance(), this.getAngle());
  }

  public SwerveModulePosition getRedPosition() {
    return new SwerveModulePosition(this.getDistance(), Rotation2d.fromDegrees(-this.getAngle().getDegrees()));
  }

  public CANSparkMax getDriveMotor() {
    return this.driveMotor;
  }

  public void periodic() {
    var pidOutput = MathUtil.clamp(drivePIDController.calculate(getSpeed(),
        driveSetpoint),
        -Constants.SwerveConstants.maxSpeed,
        Constants.SwerveConstants.maxSpeed);
    var ffOutput = feedforward.calculate(driveSetpoint);

    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " PID output", pidOutput);
    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " FF output", ffOutput);

    driveController.setReference((pidOutput / Constants.SwerveConstants.maxSpeed)
        * 12 + ffOutput,
        ControlType.kVoltage);
  }
}