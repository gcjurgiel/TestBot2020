/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import frc.robot.commands.PIDCartesianDriveCommand;
import frc.robot.Constants.PIDConstants.ChassePIDConstants.*;
import frc.robot.Constants.PIDConstants.MotorPIDConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsytem driveSubsystem = new DriveSubsytem();
  private final JoystickSubsystem joystickSubsystem = new JoystickSubsystem();

  private final DriveCartesianCommand driveCartesianCommand = new DriveCartesianCommand(driveSubsystem, joystickSubsystem);
  // private final PIDCartesianDriveCommand driveCartesianCommandPID = new PIDCartesianDriveCommand(driveSubsystem,joystickSubsystem);
  private final RotateToCommand rotateTo = new RotateToCommand(90.0, driveSubsystem);
  Double maxVelFor = 0.0;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings



    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new JoystickButton(joystickSubsystem.getJoystick(), 11).whenPressed(rotateTo);
  }

  public Command getDriveCartesianCommand() {
    return driveCartesianCommand;
  }

  // public Command getPIDDriveCartesianCommand() {
  //   return driveCartesianCommandPID;
  // }

  // public Command getFollowTrajectoryCommand() {
  //   // Object to configure trajectory motion profiling
  //   TrajectoryConfig config = new TrajectoryConfig(DriveConstants.maxVel, DriveConstants.maxAcc)
  //     .setKinematics(driveSubsystem.getKinematics())
  //     .addConstraint(driveSubsystem.getKinematicsConstraint());

  //   // Trajectory from start, path, end, and config
  //   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  //     new Pose2d(0.0, 0.0, new Rotation2d(0.0)), 
  //     // S Curve
  //     List.of(
  //       new Translation2d(1, 1),
  //       new Translation2d(2, -1)
  //     ), 
  //     new Pose2d(0.0, 0.0, new Rotation2d(0.0)), 
  //     config
  //   );

  //   // Take all the data and construct a command to run the trajectory
  //   // Outputs are from -12 to 12 volts to the motors
  //   // Note: Controler does not set voltage to zero after completion for paths have non stationary end states
  //   // Note2: rotation is calculated based on the final pose, not the pose at each step
  //   MecanumControllerCommand command = new MecanumControllerCommand(
  //     exampleTrajectory, // Trajectoory to follow
  //     driveSubsystem::getPose, // function that will return the robots position
  //     driveSubsystem.getFeedforward(), // feed forward
  //     driveSubsystem.getKinematics(), // kinematics
  //     new PIDController(X.p, X.i, X.d, X.period), // PID controllers for robot as a whole
  //     new PIDController(Y.p, Y.i, Y.d, Y.period),
  //     new ProfiledPIDController(Theta.p, Theta.i, Theta.d, new Constraints(Theta.maxVel, Theta.maxAcc), Theta.period),
  //     DriveConstants.maxWheelVel, // maximum wheel speed 
  //     new PIDController(MotorPIDConstants.FrontLeft.p, MotorPIDConstants.FrontLeft.i, MotorPIDConstants.FrontLeft.d), // PID controllers for wheels
  //     new PIDController(MotorPIDConstants.FrontRight.p, MotorPIDConstants.FrontRight.i, MotorPIDConstants.FrontRight.d), 
  //     new PIDController(MotorPIDConstants.RearLeft.p, MotorPIDConstants.RearLeft.i, MotorPIDConstants.RearLeft.d), 
  //     new PIDController(MotorPIDConstants.RearRight.p, MotorPIDConstants.RearRight.i, MotorPIDConstants.RearRight.d), 
  //     driveSubsystem::getWheelSpeeds, // function that will return the wheel speeds
  //     driveSubsystem::setVoltage, // function that can be used to set the motors voltage
  //     driveSubsystem // required subsystem
  //   );

  //   // Alternat Constructor
  //   // Necesary to do PID controll for motors seperatly
  //   // Note: Controler does not set voltage to zero after completion for paths have non stationary end states
  //   // Note2: rotation is calculated based on the final pose, not the pose at each step
  //   // MecanumControllerCommand command = new MecanumControllerCommand(
  //   //   exampleTrajectory, 
  //   //   driveSubsystem::getPose,
  //   //   driveSubsystem.getKinematics(),
  //   //   new PIDController(X.p, X.i, X.d, X.period), 
  //   //   new PIDController(Y.p, Y.i, Y.d, Y.period),  
  //   //   new ProfiledPIDController(Theta.p, Theta.i, Theta.d, new Constraints(Theta.maxVel, Theta.maxAcc), Theta.period), 
  //   //   DriveConstants.maxWheelVel,
  //   //   driveSubsystem::setWheelSpeeds,
  //   //   driveSubsystem
  //   // );

  //   return command.andThen(() -> driveSubsystem.setVoltage( new MecanumDriveMotorVoltages(0.0, 0.0, 0.0, 0.0)));
  // }
}
