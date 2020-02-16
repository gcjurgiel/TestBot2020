/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Motor Controller Constants
        public static final int frontLeftPort = 4;
        public static final int frontRightPort = 5;
        public static final int rearLeftPort = 1;
        public static final int rearRightPort = 2;

        // Encoder Constants
        public static final int unitsPerRotation = 4096;
        public static final boolean encoderIsInverted = false;

        // Gyro Constnats
        public static final boolean isGyroReversed = false;

        // Initial Position
        public static final double startPosLong = 5.0;
        public static final double startPosShort = 13.5;

        // Wheel Locations (meters)
        public static final Translation2d frontLeft = new Translation2d(-0.381, 0.381);
        public static final Translation2d frontRight = new Translation2d(0.381, 0.381);
        public static final Translation2d rearLeft = new Translation2d(-0.381, -0.381);
        public static final Translation2d rearRight = new Translation2d(0.381, -0.381);
       
        //Wheel Size (meters)
        public static final double wheelDiameter = 0.1524; // 6in
        public static final double wheelRadius = wheelDiameter / 2; // 3in, 0.076m 
        public static final double wheelCircumfrence = wheelDiameter * Math.PI; // 18.85in, 0.48m 

        // Maximums
        public static final double maxWheelVel = 0.0;   // meters per second

        public static final double maxVelForward = 3.73975;        // meters per second
        public static final double maxAccForward = 0.0;        // meters per second squared
        public static final double maxJerkForward = 0.0;       // meters per second cubed

        public static final double maxVelSideways = 1.8770752;
        public static final double maxVelRot = 424.98776;

        // Gains
        public static final double ks = 0.0;
        public static final double kv = 0.0;
        public static final double ka = 0.0;
    }

    public static final class PIDConstants {
        public static final class ChassePIDConstants {
            public static final class X {
                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
                public static final double period = 0.02;
            }
            public static final class Y {
                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
                public static final double period = 0.02;
            }
            public static final class Theta {
                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
                public static final double period = 0.02;
                public static final double maxAcc = 0.0;
                public static final double maxVel = 0.0;
            }
        }
        public static final class MotorPIDConstants {
            public static final class FrontLeft {
                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
                public static final double period = 0.02;
            }
            public static final class FrontRight {
                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
                public static final double period = 0.02;
            }
            public static final class RearLeft {
                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
                public static final double period = 0.02;
            }
            public static final class RearRight {
                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
                public static final double period = 0.02;
            }
        }

    }
    public static final class OIConstants {
        //Input Ports
        public static final int xboxControllerPort = -1;
        public static final int joyStickPort = 1;
    }
}
