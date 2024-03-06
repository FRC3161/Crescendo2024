package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberMode;

public class Climber extends SubsystemBase {

  private CANSparkMax leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.leftMotorID,
      MotorType.kBrushless);
  private CANSparkMax rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.rightMotorID,
      MotorType.kBrushless);

  private final RelativeEncoder encoderLeft;
  private final RelativeEncoder encoderRight;

  private double leftPower = 0;
  private double rightPower = 0;

  private ClimberMode climberMode = ClimberMode.IDLE;
  private boolean isLeftDone = false;
  private boolean isRightDone = false;

  public Climber() {
    setupMotors();

    encoderLeft = leftClimberMotor.getEncoder();
    resetEncoderLeft();

    encoderRight = rightClimberMotor.getEncoder();
    resetEncoderRight();
  }

  private void setupMotors() {
    leftClimberMotor.restoreFactoryDefaults();
    leftClimberMotor.enableVoltageCompensation(12);
    leftClimberMotor.setSmartCurrentLimit(60, 40);
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    leftClimberMotor.setInverted(true);

    rightClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.enableVoltageCompensation(12);
    rightClimberMotor.setSmartCurrentLimit(60, 40);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setInverted(true);
  }

  public void burnToFlash() {
    leftClimberMotor.burnFlash();
    rightClimberMotor.burnFlash();
  }

  public void setMode(ClimberMode mode) {
    climberMode = mode;
    isLeftDone = false;
    isRightDone = false;
  }

  public void set(double power) {
    setLeft(power);
    setRight(power);
  }

  public void stop() {
    stopLeft();
    stopRight();
  }

  public void setLeft(double power) {
    leftPower = power;
  }

  public void setRight(double power) {
    rightPower = power;
  }

  public void stopLeft() {
    setLeft(0);
  }

  public void stopRight() {
    setRight(0);
  }

  private void resetEncoderLeft() {
    encoderLeft.setPosition(0);
  }

  private void resetEncoderRight() {
    encoderRight.setPosition(0);
  }

  public void logValues() {
    SmartDashboard.putNumber("Actual Climber Left", encoderLeft.getPosition());
    SmartDashboard.putNumber("Actual Climber Right", encoderRight.getPosition());

    SmartDashboard.putNumber("Climber Left Current", leftClimberMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber Right Current", rightClimberMotor.getOutputCurrent());
  }

  public boolean isDone() {
    return isLeftDone && isRightDone;
  }

  public boolean isRightOutOfBounds() {
    var encoderValue = encoderRight.getPosition();
    return encoderValue <= ClimberConstants.min || encoderValue >= ClimberConstants.max;
  }

  public boolean isLeftOutOfBounds() {
    var encoderValue = encoderLeft.getPosition();
    return encoderValue <= ClimberConstants.min || encoderValue >= ClimberConstants.max;
  }

  private void handleLeft() {
    switch (climberMode) {
      case HOMING:
        if (leftClimberMotor.getOutputCurrent() >= ClimberConstants.homingCurrentThreshold) {
          stopLeft();
          isLeftDone = true;
          encoderLeft.setPosition(0);
        } else {
          setLeft(ClimberConstants.selfHomeSpeed);
        }
        break;
      case DEPLOY:
        if (encoderLeft.getPosition() >= ClimberConstants.max) {
          stopRight();
          isLeftDone = true;
        } else {
          setLeft(-ClimberConstants.deploySpeed);
        }
        break;
      case RETRACT:
        if (encoderLeft.getPosition() <= ClimberConstants.min) {
          stopLeft();
          isLeftDone = true;
        } else {
          setLeft(ClimberConstants.retractSpeed);
        }
        break;
      case MANUAL:
        if (isLeftOutOfBounds()) {
          stopLeft();
        }
        break;
      case IDLE:
        stop();
        break;
      default:
        break;
    }
  }

  private void handleRight() {
    switch (climberMode) {
      case HOMING:
        if (rightClimberMotor.getOutputCurrent() >= ClimberConstants.homingCurrentThreshold) {
          stopRight();
          isRightDone = true;
          encoderRight.setPosition(0);
        } else {
          setRight(ClimberConstants.selfHomeSpeed);
        }
        break;
      case DEPLOY:
        if (encoderRight.getPosition() >= ClimberConstants.max) {
          stopRight();
          isRightDone = true;
        } else {
          setRight(-ClimberConstants.deploySpeed);
        }
        break;
      case RETRACT:
        if (encoderRight.getPosition() <= ClimberConstants.min) {
          stopRight();
          isRightDone = true;
        } else {
          setRight(ClimberConstants.retractSpeed);
        }
        break;
      case MANUAL:
        if (isRightOutOfBounds()) {
          stopRight();
        }
        break;
      case IDLE:
        stop();
        break;

      default:
        break;
    }
  }

  @Override
  public void periodic() {
    logValues();

    handleLeft();
    handleRight();

    leftClimberMotor.set(leftPower);
    rightClimberMotor.set(rightPower);

  }
}
