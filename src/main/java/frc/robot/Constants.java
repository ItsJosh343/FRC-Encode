// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class RobotMap{
        public static final int kLeftMaster = 3;
        public static final int kLeftSlave = 1;
        public static final int kRightMaster = 2;
        public static final int kRightSlave = 4;
        public static final int kSpinner = 0;
        public static final int kLiftMaster = 5;
        public static final int kLiftSlave = 6;
    }

    public static final class PID{
        public static final double kTolerance = .001;
        public static final double kPDrive = 5e-20;
        public static final double kIDrive = 1e-6 + 1e-3;
        public static final double kIZoneDrive = 0;
        public static final double kDDrive = 0;
        public static final double kFFDrive = 0.000156;
        public static final double kForwardRR = 0;
        public static final double kStrafeRR = 0;
        public static final double kIsMoreStrafe = .5;
    }

    public static final class Lift{
        public static final float kUpwardLimit = 10;
        public static final float kDownwardLimit = 0;
        public static final double kLiftSpeed = .01;
    }

    public static final class OIConstants{
        public static final int kJoystick1 = 0;
        public static final int kJoystick2 = 1;
    }
}
