/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotMap;

/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.  This subsystem deals with both
 * intakes on the robot.  First the hatch intake uses two pneumatic pistons and a limit switch.  One pneumatic piston is used to 
 * hold the hatch by extending in the center of the hatch.  The other piston is used to extend and retract the whole hatch 
 * holding mechanism.  The limit switch is used to detect the presence of a hatch in the mechanism.  The cargo intake contains one
 * pneumatic piston, a motor, and a distance sensor.  The pneumatic piston is used to drop down and raise the intake.  The motor
 * is used to suck in and spit out the cargo.  The distance sensor is used  to detect the presence of a cargo.
 */

public class Intakes extends Subsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Intakes instance = new Intakes();

  public static Intakes getInstance() {
    return instance;
  }


  private DoubleSolenoid cargoPiston;
  private TalonSRX cargoMotor;
  private AnalogInput distanceSensor;

  public Intakes()
  {
      cargoMotor = new TalonSRX(RobotMap.INTAKE_CARGO_MOTOR);
      cargoMotor.setInverted(true);
      distanceSensor = new AnalogInput(RobotMap.INTAKE_DISTANCE_SENSOR);
  }



  
   //Cargo Intake Methods

   public void intakeCargo()
  {
      cargoMotor.set(ControlMode.PercentOutput, -1);
      
  }

   public void releaseCargo()
  {
      cargoMotor.set(ControlMode.PercentOutput, 1);
  }

  public void stopCargo()
  {
      cargoMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean hasCargo()
  {
      if(distanceSensor.getVoltage() > 0.828)  //Any object that is 7cm or closer will produce 0.8V or more
        return true;
      else
        return false;
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
