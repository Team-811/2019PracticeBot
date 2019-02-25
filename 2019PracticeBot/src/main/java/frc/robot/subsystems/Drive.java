/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.Utility.MecanumDrive;
import frc.robot.Utility.MotionProfiling;
import frc.robot.Utility.Output;
import frc.robot.commands.*;
import jaci.pathfinder.Waypoint;

import javax.swing.plaf.TreeUI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.AHRSProtocol;
import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX topLeftMotor;
  private TalonSRX topRightMotor;
  private TalonSRX bottomLeftMotor;
  private TalonSRX bottomRightMotor;

  private AHRS gyro;

  private MecanumDrive drivetrain;

  private MotionProfiling motionProfile;


  private final int kTimeoutMs = 30;

  private final double wheelDiameter = 0.1524;
  private final double wheelbase = 0.7493;

  private int kP;
  private int kI;
  private int kD;
  private int kF;

  public Drive()
  {
      topLeftMotor = new TalonSRX(Robot.robotMap.DRIVE_TOP_LEFT_MOTOR);
      topRightMotor = new TalonSRX(Robot.robotMap.DRIVE_TOP_RIGHT_MOTOR);
      bottomLeftMotor = new TalonSRX(Robot.robotMap.DRIVE_BOTTOM_LEFT_MOTOR);
      bottomRightMotor = new TalonSRX(Robot.robotMap.DRIVE_BOTTOM_RIGHT_MOTOR);

      bottomLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1);
      bottomRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1);

      bottomLeftMotor.setSensorPhase(true);
      bottomRightMotor.setSensorPhase(true);

      bottomLeftMotor.setSelectedSensorPosition(0);
      bottomRightMotor.setSelectedSensorPosition(0);

      bottomLeftMotor.configNominalOutputForward(0, kTimeoutMs);
      bottomLeftMotor.configNominalOutputReverse(0, kTimeoutMs);
      bottomLeftMotor.configPeakOutputForward(1, kTimeoutMs);
      bottomLeftMotor.configPeakOutputReverse(-1, kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
	    bottomLeftMotor.config_kF(0, kF, kTimeoutMs);
		bottomLeftMotor.config_kP(0, kP, kTimeoutMs);
		bottomLeftMotor.config_kI(0, kI, kTimeoutMs);
        bottomLeftMotor.config_kD(0, kD, kTimeoutMs);
        

    bottomRightMotor.configNominalOutputForward(0, kTimeoutMs);
    bottomRightMotor.configNominalOutputReverse(0, kTimeoutMs);
    bottomRightMotor.configPeakOutputForward(1, kTimeoutMs);
    bottomRightMotor.configPeakOutputReverse(-1, kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		bottomRightMotor.config_kF(0, kF, kTimeoutMs);
		bottomRightMotor.config_kP(0, kP, kTimeoutMs);
		bottomRightMotor.config_kI(0, kI, kTimeoutMs);
		bottomRightMotor.config_kD(0, kD, kTimeoutMs);

      topRightMotor.setInverted(true);
      bottomRightMotor.setInverted(true);

      gyro = new AHRS(SerialPort.Port.kMXP);
      gyro.reset();

      drivetrain = new MecanumDrive();
      drivetrain.invertForwardBackward(false);
      drivetrain.invertStrafing(false);

      motionProfile = new MotionProfiling(0.55, 0.55, 0.55, wheelbase);
  }


  public void DriveWithJoy(double leftJoy, double rightJoy, double strafe)
  {
      
      
      SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
      SmartDashboard.putNumber("LeftEncoder", bottomLeftMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("RightEncoder", bottomRightMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Left Velocity", unitsPer100MSToMetersPerSecond(bottomLeftMotor.getSelectedSensorVelocity(), wheelDiameter));
      SmartDashboard.putNumber("Right Velocity", unitsPer100MSToMetersPerSecond(bottomRightMotor.getSelectedSensorVelocity(), wheelDiameter));
      //Output driveOutput = drivetrain.curvatureMecanumDrive(leftJoy, rightJoy, quickTurn, false, strafe, 0.1);
      double correction = gyroCorrection(3 * rightJoy);
      Output driveOutput = drivetrain.fieldOrientedDrive(leftJoy, strafe, rightJoy + correction, gyro.getAngle());

      topLeftMotor.set(ControlMode.PercentOutput, driveOutput.getTopLeftValue());
      topRightMotor.set(ControlMode.PercentOutput, driveOutput.getTopRightValue());
      bottomLeftMotor.set(ControlMode.PercentOutput, driveOutput.getBottomLeftValue());
      bottomRightMotor.set(ControlMode.PercentOutput, driveOutput.getBottomRightValue());

  }

  private double gyroCorrectRate = 0.02;

  public double gyroCorrection(double tangentialVelocity)
  {
      return ( (tangentialVelocity/(wheelbase/2)) - Math.toRadians(gyro.getAngle()) ) * gyroCorrectRate;
  }


  public void loadTrajectory(Waypoint[] path)
  {
      motionProfile.loadTrajectory(path);
  }

  public void followTrajectory(boolean reverse)
  {

    double leftEncoderMeters = encoderTicksToMeters(bottomLeftMotor.getSelectedSensorPosition(), wheelDiameter);
    double rightEncoderMeters = encoderTicksToMeters(bottomRightMotor.getSelectedSensorPosition(), wheelDiameter);

    Output driveOutput = motionProfile.getNextDriveSignal(reverse, topLeftMotor.getSelectedSensorPosition(), topRightMotor.getSelectedSensorPosition(), gyro.getAngle());

    double velocityLeft = metersPerSecondToUnitsPer100MS(driveOutput.getLeftValue(), wheelDiameter);
    double velocityRight = metersPerSecondToUnitsPer100MS(driveOutput.getLeftValue(), wheelDiameter);

    topLeftMotor.set(ControlMode.Velocity, velocityLeft);
    topRightMotor.set(ControlMode.Velocity, velocityRight);
    bottomLeftMotor.set(ControlMode.Velocity, velocityLeft);
    bottomRightMotor.set(ControlMode.Velocity, velocityRight);
  }

  public void setVelocityTo1()
  {
    double velocityLeft = metersPerSecondToUnitsPer100MS(1, wheelDiameter);
    double velocityRight = metersPerSecondToUnitsPer100MS(1, wheelDiameter);

    topLeftMotor.set(ControlMode.Velocity, velocityLeft);
    topRightMotor.set(ControlMode.Velocity, velocityRight);
    bottomLeftMotor.set(ControlMode.Velocity, velocityLeft);
    bottomRightMotor.set(ControlMode.Velocity, velocityRight);
  }

  public void setVelocityTo0()
  {
    double velocityLeft = metersPerSecondToUnitsPer100MS(1, wheelDiameter);
    double velocityRight = metersPerSecondToUnitsPer100MS(1, wheelDiameter);

    topLeftMotor.set(ControlMode.Velocity, velocityLeft);
    topRightMotor.set(ControlMode.Velocity, velocityRight);
    bottomLeftMotor.set(ControlMode.Velocity, velocityLeft);
    bottomRightMotor.set(ControlMode.Velocity, velocityRight);
  }



  public boolean trajectoryIsFinished()
  {
        return motionProfile.isFinished();
  }

  private double encoderTicksToMeters(double encoderTicks, double wheelDiameter)
  {
        return (encoderTicks * 1024)/(2 * Math.PI * wheelDiameter);
  }

  private double metersPerSecondToUnitsPer100MS(double velocity, double wheelDiameter)
  {
      return (velocity * (1/(2 * Math.PI * wheelDiameter)) * 1024 / 10);
  }

  private double unitsPer100MSToMetersPerSecond(double velocity, double wheelDiameter)
  {
    return velocity * ((2 * Math.PI * wheelDiameter) * 10 / 1024);
  }

  public void zeroSensors()
  {
    bottomLeftMotor.setSelectedSensorPosition(0);
    bottomRightMotor.setSelectedSensorPosition(0);
    gyro.reset();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
      setDefaultCommand(new DriveWithJoy());
  }
}
