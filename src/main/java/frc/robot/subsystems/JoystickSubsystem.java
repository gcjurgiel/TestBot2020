/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.Controller;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JoystickSubsystem extends SubsystemBase {

  private static Joystick stick = new Joystick(Controller.Joystick.port);

  public JoystickSubsystem() {
  }

  public double getX() {
    return (Math.abs(stick.getX()) <= 0.2) ? 0.0 : stick.getX();
  }

  public double getRawX() {
    return stick.getX();
  }

  public double getY() {
    return (Math.abs(stick.getY()) <= 0.2) ? 0.0 : stick.getY();
  }
  public double getRawY() {
    return stick.getY();
  }

  public double getTwist() {
    double t = (Math.abs(Math.pow(stick.getTwist(),2)) <= 0.2) ? 0.0 : Math.abs(Math.pow(stick.getTwist(),2));
    return (stick.getTwist() >= 0) ? t : -t;
  }

  public double getRawTwist() {
    return stick.getTwist();
  }

  public Joystick getJoystick() {
    return stick;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
