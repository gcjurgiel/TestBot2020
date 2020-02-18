/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Vision;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private static final JoystickSubsystem joystickSubsystem = new JoystickSubsystem();
  private static final VisionSubsystem visionSubsystem = new VisionSubsystem(Vision.host, Vision.port);

  private static final DriveCartesianCommand driveCartesianCommand = new DriveCartesianCommand(driveSubsystem, joystickSubsystem);
  private static final DriveStraightCommand driveStrightCommand = new DriveStraightCommand(driveSubsystem, joystickSubsystem);
  private static final RotateToCommand rotateToCommand = new RotateToCommand(90, driveSubsystem);
  private static final RotateToLoadingBayCommand rotateToLoadingBayCommand = new RotateToLoadingBayCommand(
      driveSubsystem, visionSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // joystickSubsystem.getButton(1)
    // .whileActiveContinuous(driveStrightCommand)
    // .whenInactive(driveCartesianCommand);

    joystickSubsystem.getButton(11).whenPressed(() -> {
      try {
System.out.println("Trying Connection...");
        visionSubsystem.startUp();
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    });

    joystickSubsystem.getButton(1).whenPressed(() -> {
      System.out.println(visionSubsystem.getLoadingBayBearing());
    });

    joystickSubsystem.getButton(12).whenPressed(() -> {
      try {
        visionSubsystem.shutDown();
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    });

    joystickSubsystem.getButton(9).whenPressed(rotateToLoadingBayCommand);
  }

  public Command getDriveCartesianCommand() {
    return driveCartesianCommand;
  }
}
