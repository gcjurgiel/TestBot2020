/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.constraint.MecanumDriveKinematicsConstraint;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsytem extends SubsystemBase {
  
  // Motor Controllers and Drive Init
  private static final VictorSP frontRight= new VictorSP(DriveConstants.frontRightPort);
  private static final VictorSP frontLeft = new VictorSP(DriveConstants.frontLeftPort);
  private static final VictorSP rearRight = new VictorSP(DriveConstants.rearRightPort);
  private static final VictorSP rearLeft = new VictorSP(DriveConstants.rearLeftPort);
  private static final MecanumDrive drive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

  // Helper for computing simple DC Motor feed forward
  // private static final SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(
  //   DriveConstants.ks, DriveConstants.kv, DriveConstants.ka
  // );

  // // Wheel Locations (meters)
  // private static final Translation2d frontRightPos = new Translation2d(0.381, 0.381);
  // private static final Translation2d frontLeftPos = new Translation2d(-0.381, 0.381);
  // private static final Translation2d rearRightPos = new Translation2d(0.381, -0.381);
  // private static final Translation2d rearLeftPos = new Translation2d(-0.381, -0.381);

  // // Kinematics object to convert chasse velocity to indiviual wheel velocity 
  // private static final MecanumDriveKinematics driveKinematics = new MecanumDriveKinematics(
  //   frontLeftPos, frontRightPos,
  //   rearLeftPos, rearRightPos
  // );

  // // Constrains on kinematics so motors dont go over max velocity
  // private static final MecanumDriveKinematicsConstraint driveConstraints = new MecanumDriveKinematicsConstraint(
  //   driveKinematics, 
  //   DriveConstants.maxVel
  // );

  // Odometry object to track robots location on the feild 
  // private MecanumDriveOdometry odometry;

  // // Initialize pose variable with starting position
  // private Pose2d pose = new Pose2d(
  //   DriveConstants.startPosLong, 
  //   DriveConstants.startPosShort, 
  //   Rotation2d.fromDegrees(getHeading())
  // );

  // Define variable to hold the speed
  // private MecanumDriveWheelSpeeds speed;

  // //Gyro Init
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public DriveSubsytem() {
    // //Define Encoder type through SRX Controller
    // // Relative/Quadrature: faster update rate.
    // // Absolute/Pulse Width: Solid refrence with no need to rezero within one rotation. 
    // frontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    // frontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    // rearLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    // rearRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    // //Initialize Odometry with kinematics, gyroposition, and position on the feild.
    // odometry = new MecanumDriveOdometry(
    //   driveKinematics, 
    //   Rotation2d.fromDegrees(getHeading()), 
    //   pose
    // );

    // // Initialize speed
    // speed = getWheelSpeeds();
  }

  // Method called Periodical (continusly) while the robot is powered on
  @Override
  public void periodic() { 
    // // update speed
    // speed = getWheelSpeeds();
    // //update odometry position and hold in pose variable
    // pose = odometry.update(Rotation2d.fromDegrees(getHeading()), speed);
  } 

//   // returns position from odometry
//   public Pose2d getPose() {
//     return odometry.getPoseMeters();
//   }

//   public SimpleMotorFeedforward getFeedforward() {
//     return motorFeedforward;
//   }

//   public MecanumDriveKinematics getKinematics() {
//     return driveKinematics;
//   }

//   public MecanumDriveKinematicsConstraint getKinematicsConstraint() {
//     return driveConstraints;
//   }

//   // returns the wheel speeds (meters per second) 
//   public MecanumDriveWheelSpeeds getWheelSpeeds() {
//     // Converts from units per 100ms to meters per second
//     return new MecanumDriveWheelSpeeds(
//       ((frontLeft.getSelectedSensorVelocity() / DriveConstants.unitsPerRotation) * DriveConstants.wheelCircumfrence) * 10, 
//       ((frontRight.getSelectedSensorVelocity() / DriveConstants.unitsPerRotation) * DriveConstants.wheelCircumfrence) * 10, 
//       ((rearLeft.getSelectedSensorVelocity() / DriveConstants.unitsPerRotation) * DriveConstants.wheelCircumfrence) * 10, 
//       ((rearRight.getSelectedSensorVelocity() / DriveConstants.unitsPerRotation) * DriveConstants.wheelCircumfrence) * 10
//     );
//   }

//   // sets wheels to speeds
//   public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
//     // Converts from meters per second to units per 100ms
//     frontLeft.set(ControlMode.Velocity, ((wheelSpeeds.frontLeftMetersPerSecond / DriveConstants.wheelCircumfrence) * DriveConstants.unitsPerRotation) / 10);
//     frontRight.set(ControlMode.Velocity, ((wheelSpeeds.frontRightMetersPerSecond / DriveConstants.wheelCircumfrence) * DriveConstants.unitsPerRotation) / 10);
//     rearLeft.set(ControlMode.Velocity, ((wheelSpeeds.rearLeftMetersPerSecond / DriveConstants.wheelCircumfrence) * DriveConstants.unitsPerRotation) / 10);
//     rearRight.set(ControlMode.Velocity, ((wheelSpeeds.rearRightMetersPerSecond / DriveConstants.wheelCircumfrence) * DriveConstants.unitsPerRotation) / 10);
//   }

//   // sets voltages of motors
//   public void setVoltage(MecanumDriveMotorVoltages voltages) {
//     frontLeft.setVoltage(voltages.frontLeftVoltage);
//     frontRight.setVoltage(voltages.frontRightVoltage);
//     rearLeft.setVoltage(voltages.rearLeftVoltage);
//     rearRight.setVoltage(voltages.rearRightVoltage);
//   }

//  // resets odometry to given position
//   public void resetOdometry(Pose2d pose) {
//     resetEncoders();
//     odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
//   }


  public void driveCartesian(double x, double y, double rotation) {
    drive.driveCartesian(y, x, rotation);
  } 

  public void driveCartesian(double x, double y, double rotation, double angle) {
    drive.driveCartesian(y, x, rotation, angle);
  } 

//   // resets the encoders back to 0. 
//   public void resetEncoders() {
//     frontLeft.getSensorCollection().setQuadraturePosition(0, 10);
//     frontRight.getSensorCollection().setQuadraturePosition(0, 10);
//     rearLeft.getSensorCollection().setQuadraturePosition(0, 10);
//     rearRight.getSensorCollection().setQuadraturePosition(0, 10);
//   }

//   //limits maximum output
//   public void setMaxOutput(double maxOutput) {
//     drive.setMaxOutput(maxOutput);
//   }

//   // Rezeros gyro heading
  public void zeroHeading() {
    gyro.reset();
  }

//   // returns gyro heading
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.isGyroReversed ? -1.0 : 1.0);
  }

//   // returns gyro turn rate
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.isGyroReversed ? -1.0 : 1.0);
  }

  public AHRS getGyro() {
    return gyro;
  }
}