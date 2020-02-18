/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive.Robot.Theta;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RotateToLoadingBayCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private JoystickSubsystem joystickSubsystem;
  private PIDController controller;

  public RotateToLoadingBayCommand(DriveSubsystem drive, VisionSubsystem vision, JoystickSubsystem stick) {
    driveSubsystem = drive;
    visionSubsystem = vision;
    joystickSubsystem = stick;
    controller = drive.getRobotThetaPID();
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(Theta.poseTolerance, Theta.velTolerance);
    addRequirements(drive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //   try {
  //     visionSubsystem.startUp();
  //   } catch (IOException e) {
  //     // TODO Auto-generated catch block
  //     e.printStackTrace();
  //   }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = controller.calculate(visionSubsystem.getLoadingBayBearing(), 0.0);
    driveSubsystem.driveCartesian(-joystickSubsystem.getY(), joystickSubsystem.getX(), -output / 51.43);
    System.out.println(visionSubsystem.getLoadingBayBearing());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // try {
    //   visionSubsystem.shutDown();
    // } catch (IOException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    driveSubsystem.driveCartesian(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
