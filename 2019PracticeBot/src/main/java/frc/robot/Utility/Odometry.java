/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

/**
 * Add your docs here.
 */
public class Odometry 
{
    private double xCoordinate;
    private double yCoordinate;
    private double theta;

    //Default Constructor
    public Odometry()
    {
        xCoordinate = 0;
        yCoordinate = 0;
        theta = 0;
    }

    //Parameterized constructor
    public Odometry(double xCoordinate, double yCoordinate, double theta)
    {
        this.xCoordinate = xCoordinate;
        this.yCoordinate = yCoordinate;
        this.theta = theta;
    }

    //Getter methods

    public double getX ()
    {
        return xCoordinate;
    }

    public double getY()
    {
        return yCoordinate;
    }

    public double getTheta()
    {
        return theta;
    }



    //Setter Methods

    public void setX(double xCoordinate)
    {
        this.xCoordinate = xCoordinate;
    }

    public void setY(double yCoordinate)
    {
        this.yCoordinate = yCoordinate;
    }

    public void setTheta(double theta)
    {
        this.theta = theta;
    }


}
