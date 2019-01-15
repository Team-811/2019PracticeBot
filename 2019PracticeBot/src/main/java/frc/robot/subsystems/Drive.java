/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.*;
import frc.robot.Utility.MecanumDrive;
import frc.robot.Utility.Output;
import frc.robot.commands.*;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

  private MecanumDrive drivetrain;

  public Drive()
  {
      topLeftMotor = new TalonSRX(Robot.robotMap.DRIVE_TOP_LEFT_MOTOR);
      topRightMotor = new TalonSRX(Robot.robotMap.DRIVE_TOP_RIGHT_MOTOR);
      bottomLeftMotor = new TalonSRX(Robot.robotMap.DRIVE_BOTTOM_LEFT_MOTOR);
      bottomRightMotor = new TalonSRX(Robot.robotMap.DRIVE_BOTTOM_RIGHT_MOTOR);

      topLeftMotor.setInverted(true);
      bottomLeftMotor.setInverted(true);

      drivetrain = new MecanumDrive();
  }


  public void DriveWithJoy(double leftJoy, double rightJoy, double leftTrigger, double rightTrigger)
  {
      
    if(leftTrigger <= 0.2)
    {
        leftTrigger = 0;
    }
    else
    {
        leftTrigger = leftTrigger;
    }

    if(rightTrigger <= 0.2)
    {
        rightTrigger = 0;
    }
    else
    {
        rightTrigger = rightTrigger;
    }

    
    
    double strafe = rightTrigger - leftTrigger;
      boolean quickTurn;

      if(leftJoy <= .2 && leftJoy >= -.2)
      {
          quickTurn = true;
      }
      else
      {
          quickTurn = false;
      }

      Output driveOutput = drivetrain.arcadeMecanumDrive(leftJoy, rightJoy, strafe, 0.2);
      //Output driveOutput = drivetrain.tankMecanumDrive(leftJoy, rightJoy, strafe, 0.2);
      //Output driveOutput = drivetrain.curvatureMecanumDrive(leftJoy, rightJoy, quickTurn, false, strafe, 0.1);

      topLeftMotor.set(ControlMode.PercentOutput, driveOutput.getTopLeftValue());
      topRightMotor.set(ControlMode.PercentOutput, driveOutput.getTopRightValue());
      bottomLeftMotor.set(ControlMode.PercentOutput, driveOutput.getBottomLeftValue());
      bottomRightMotor.set(ControlMode.PercentOutput, driveOutput.getBottomRightValue());

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
      setDefaultCommand(new DriveWithJoy());
  }
}
