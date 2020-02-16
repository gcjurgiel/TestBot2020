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

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants.Drive;

public class DriveSubsytem extends SubsystemBase {
  
  // Motor Controllers and Drive
  private static final VictorSP frontLeft= new VictorSP(Drive.Wheels.FrontLeft.port);
  private static final VictorSP frontRight = new VictorSP(Drive.Wheels.FrontRight.port);
  private static final VictorSP rearLeft = new VictorSP(Drive.Wheels.RearLeft.port);
  private static final VictorSP rearRight = new VictorSP(Drive.Wheels.RearRight.port);
  private static final MecanumDrive drive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

  //Gyro
  private static final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public DriveSubsytem() {
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

//   // Rezeros gyro heading
  public void zeroHeading() {
    gyro.reset();
  }

//   // returns gyro heading
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (Drive.Robot.isGyroReversed ? -1.0 : 1.0);
  }

//   // returns gyro turn rate
  public double getTurnRate() {
    return gyro.getRate() * (Drive.Robot.isGyroReversed ? -1.0 : 1.0);
  }

  public AHRS getGyro() {
    return gyro;
  }
}