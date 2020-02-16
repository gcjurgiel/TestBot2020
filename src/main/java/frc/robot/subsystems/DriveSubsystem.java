/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants.Drive.Wheels;
import frc.robot.Constants.Drive.Robot;

public class DriveSubsystem extends SubsystemBase {
  
  // Motor Controllers and Drive
  private static final VictorSP frontLeft= new VictorSP(Wheels.FrontLeft.port);
  private static final VictorSP frontRight = new VictorSP(Wheels.FrontRight.port);
  private static final VictorSP rearLeft = new VictorSP(Wheels.RearLeft.port);
  private static final VictorSP rearRight = new VictorSP(Wheels.RearRight.port);
  private static final MecanumDrive drive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

  //PID Controllers
  private static final PIDController frontLeftPID = new PIDController(Wheels.FrontLeft.p, Wheels.FrontLeft.i, Wheels.FrontLeft.d);
  private static final PIDController frontRightPID = new PIDController(Wheels.FrontRight.p, Wheels.FrontRight.i, Wheels.FrontRight.d);
  private static final PIDController rearLeftPID = new PIDController(Wheels.RearLeft.p, Wheels.RearLeft.i, Wheels.RearLeft.d);
  private static final PIDController rearRightPID = new PIDController(Wheels.RearRight.p, Wheels.RearRight.i, Wheels.RearRight.d);

  private static final PIDController robotXPID = new PIDController(Robot.X.p, Robot.X.i, Robot.X.d);
  private static final PIDController robotYPID = new PIDController(Robot.Y.p, Robot.Y.i, Robot.Y.d);
  private static final PIDController robotThetaPID = new PIDController(Robot.Theta.p, Robot.Theta.i, Robot.Theta.d);
  
  //Gyro
  private static final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public DriveSubsystem() {
  }

  // Method called Periodical (continusly) while the robot is powered on
  @Override
  public void periodic() { 
  } 

  public void driveCartesian(double x, double y, double rotation) {
    drive.driveCartesian(y, x, rotation);
  } 
  public void driveCartesian(double x, double y, double rotation, double angle) {
    drive.driveCartesian(y, x, rotation, angle);
  } 

  public void zeroHeading() {
    gyro.reset();
  }
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (Robot.isGyroReversed ? -1.0 : 1.0);
  }
  public double getTurnRate() {
    return gyro.getRate() * (Robot.isGyroReversed ? -1.0 : 1.0);
  }
  public AHRS getGyro() {
    return gyro;
  }

  public PIDController getFrontLeftPID() {
    return frontLeftPID;
  }
  public PIDController getFrontRightPID() {
    return frontRightPID;
  }
  public PIDController getRearLeftPID() {
    return rearLeftPID;
  }
  public PIDController getRearRightPID() {
    return rearRightPID;
  }

  public PIDController getRobotXPID() {
    return robotXPID;
  }
  public PIDController getRobotYPID() {
    return robotYPID;
  }
  public PIDController getRobotThetaPID() {
    return robotThetaPID;
  }
}