// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class IntakeConstants {
        public static final int kIntakeMotorID1 = 1;
        public static final int kIntakeMotorID2 = 2;

        public static final boolean kMotor1Inverted = false;
        public static final boolean kMotor2Inverted = false;

        public static final double kIntakeVolts = 4;
    }

    public static final class ShooterConstants {

        public static final int kShooterMotorID1 = 3;
        public static final int kShooterMotorID2 = 4;
        public static final boolean kMotor1Inverted = false;
        public static final boolean kMotor2Inverted = false;

        public static final boolean kDebug = true;

        public static double kP = 5e-5;
        public static double kI = 1e-6;
        public static double kD = 0.00002;
        public static double kIz = 0;
        public static double kFF = 0.000156;
        public static double kMaxOutput = 0.1;
        public static double kMinOutput = -0.1;
        public static double kMaxRPM = 5700;

        public static double kMaxVel = 5000;
        public static double kMinVel = 1;
        public static double kMaxAcc = 200;
        public static double kAllowedErr = 0.1;

        public static final int kSmartMotionSlot = 1;

    }
}
