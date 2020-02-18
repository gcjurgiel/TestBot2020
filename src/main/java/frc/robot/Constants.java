/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants { 
    public static final class Drive {
        public static final class Wheels {
            public static final double diameter = 0.1524;
            public static final double radius = diameter / 2;
            public static final double circumfrence = diameter * Math.PI;

            public static final double maxVel = 0.0;
            public static final double maxAcc = 0.0;
            public static final double maxjerk = 0.0;

            public static final int unitsPerRotation = 4096;
            public static final boolean encoderIsInverted = false;

            public static final double p = 0.0;
            public static final double i = 0.0;
            public static final double d = 0.0;

            public static final class FrontLeft {
                public static final int port = 4;

                public static final double x = -0.381;
                public static final double y = 0.381;

                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
            }

            public static final class FrontRight {
                public static final int port = 5;

                public static final double x = 0.381;
                public static final double y = 0.381;

                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
            }

            public static final class RearLeft {
                public static final int port = 1;
                
                public static final double x = -0.381;
                public static final double y = -0.381;

                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
            }

            public static final class RearRight {
                public static final int port = 2;
                
                public static final double x = 0.381;
                public static final double y = -0.381;

                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
            }
        }
        public static final class Robot {
            public static final boolean isGyroReversed = false;

            public static final class X {
                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;
                
                public static final double maxVel = 3.73975;
                public static final double maxAcc = 0.0;
                public static final double maxJerk = 0.0;

                public static final double startPos = 5.0;
            }

            public static final class Y {
                public static final double p = 0.0;
                public static final double i = 0.0;
                public static final double d = 0.0;  
                
                public static final double maxVel = 1.8770752;
                public static final double maxAcc = 0.0;
                public static final double maxJerk = 0.0;

                public static final double startPos = 13.5;
            }

            public static final class Theta {
                public static final double p = 1.5;
                public static final double i = 0.0;
                public static final double d = 0.05; 

                public static final double poseTolerance = 2.0;
                public static final double velTolerance = 0.05;

                public static final double maxVel = 424.98776;
                public static final double maxAcc = 0.0;
                public static final double maxJerk = 0.0;
            }
        }
    }

    public static final class Controller {
        public static final class Joystick {
            public static final int port = 1;
        }
        public static final class XboxController {
            public static final int port = -1;
        }
    }

    public static final class Vision {
        public static final String host = "10.43.30.20";
        public static final int port = 9001;
    }
}
