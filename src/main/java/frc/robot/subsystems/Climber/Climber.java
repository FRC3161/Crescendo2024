package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
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

  private Timer timer = new Timer();

  private final SparkPIDController leftPID;
  private final SparkPIDController rightPID;

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
    leftPID = leftClimberMotor.getPIDController();

    encoderRight = rightClimberMotor.getEncoder();
    resetEncoderRight();
    rightPID = rightClimberMotor.getPIDController();
  }

  private static enum States {
    BADBADBAD,
    OKAYUP,
    OKAYDOWN,
    GOOD
  }

  private void setupMotors() {
    leftClimberMotor.restoreFactoryDefaults();
    leftClimberMotor.enableVoltageCompensation(12);
    leftClimberMotor.setSmartCurrentLimit(20, 20);
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    leftClimberMotor.setInverted(true);

    rightClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.enableVoltageCompensation(12);
    rightClimberMotor.setSmartCurrentLimit(20, 20);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setInverted(true);
  }

  public void burnToFlash() {
    leftClimberMotor.burnFlash();
    rightClimberMotor.burnFlash();
  }

  public void setMode(ClimberMode mode) {
    climberMode = mode;
    if (climberMode == ClimberMode.HOMING) {
      timer.reset();
      timer.start();
    }
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

  public States outOfBounds(double encoderValue) {
    if (encoderValue <= ClimberConstants.min) {
      return States.BADBADBAD;
    } else if (encoderValue <= ClimberConstants.desiredMin) {
      return States.OKAYUP;
    } else if (encoderValue < ClimberConstants.desiredMax) {
      return States.GOOD;
    } else if (encoderValue < ClimberConstants.max) {
      return States.OKAYDOWN;
    } else if (encoderValue >= ClimberConstants.max) {
      return States.BADBADBAD;
    } else {
      return States.BADBADBAD;
    }
  }

  public States isRightOutOfBounds() {
    return outOfBounds(-encoderRight.getPosition());
  }

  public States isLeftOutOfBounds() {
    return outOfBounds(-encoderLeft.getPosition());
  }

  private void handleLeft() {
    switch (climberMode) {
      case HOMING:
        if (leftClimberMotor.getOutputCurrent() >= ClimberConstants.homingCurrentThreshold) {
          if (timer.get() < 1)
            return;
          stopLeft();
          isLeftDone = true;
          encoderLeft.setPosition(0);
        } else {
          setLeft(ClimberConstants.selfHomeSpeedVoltage);
        }
        break;
      case DEPLOY:
        if (-encoderLeft.getPosition() >= ClimberConstants.max) {
          stopRight();
          isLeftDone = true;
        } else {
          setLeft(-ClimberConstants.deploySpeed);
        }
        break;
      case RETRACT:
        if (-encoderLeft.getPosition() <= ClimberConstants.min) {
          stopLeft();
          isLeftDone = true;
        } else {
          setLeft(ClimberConstants.retractSpeed);
        }
        break;
      case MANUAL:
        switch (isLeftOutOfBounds()) {
          case BADBADBAD:
            stopLeft();
            break;
          case OKAYUP:
            if (leftPower > 0) {
              setLeft(0);
            }
            break;
          case OKAYDOWN:
            if (leftPower < 0) {
              setLeft(0);
            }
            break;
          default:
            break;
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
          if (timer.get() < 1)
            return;
          stopRight();
          isRightDone = true;
          encoderRight.setPosition(0);
        } else {
          setRight(ClimberConstants.selfHomeSpeedVoltage);
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
        switch (isRightOutOfBounds()) {
          case BADBADBAD:
            stopRight();
            break;

          case OKAYUP:
            if (rightPower > 0) {
              setRight(0);
            }
            break;

          case OKAYDOWN:
            if (rightPower < 0) {
              setRight(0);
            }
            break;
          default:
            break;
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

    if (leftPower > 1) {
      leftPID.setReference(leftPower, ControlType.kVoltage);
    } else {
      leftPID.setReference(leftPower, ControlType.kDutyCycle);
    }

    if (rightPower > 1) {
      rightPID.setReference(rightPower, ControlType.kVoltage);
    } else {
      rightPID.setReference(rightPower, ControlType.kDutyCycle);
    }
  }
}
