/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants{
        public static final double TRACKWIDTH = 0.65;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                        TRACKWIDTH);

        public static final double ENCODER_CPR = 256 * 1.8 * 3 * 4;
        public static final double WHEEL_DIAMETER = 0.1;
        public static final double ENCODER_DISTANCE_PER_PULSE = (WHEEL_DIAMETER * Math.PI) / ENCODER_CPR;
		public static final boolean GYRO_REVERSED = false;


    }
    public static final class ShooterConveyer{
        public static final double OPERATING_VOLTAGE = 12;
    }

    public static final class RouletteConstants{
        public static final int ROULETTE_MOTOR_PORT = 0;
        public enum Colors{Red, Yellow, Blue, Green};
        public static int NUMBER_OF_COLORS_TO_CHANGE = 8*3;
        public static double OPTIMAL_SPEED = 4;
        public static double NEIGHBOR_SPEED = 2;
        public static double FAR_SPEED = 3;
        public static double BACK_SPEED =1;
        public static int timeToStopInSec = 5; 
    }

    public static final class ClimberConstants {
        public static final int CLIMBER_MOTOR_PORTS = 12;
        public static final double LIMIT_TICKS = 24000;
        public static final double CLIMB_SPEED = 6;
        public static final double CLIMB_PERCENTAGE = 0.8;
        public static final int CRANK_MOTOR = 8;
        public static final double CRANK_VOLTAGE = 4;
        public static final double CRANK_PERCENTAGE = 0.3;
        public static enum ClimbState {
            UP, DOWN
        }
    }

    public static final class IntakeConstants {
    public static final double GRIPPER_VOLTAGE=6;
    public static final double GRIPPER_PERCENTAGE=1;
    public static final int GRIPPER_MOTOR_PORT = 10;
    public static final int[] LIFTER_SELENOID_PORT = {0, 1};

    public static final double ROLL_SPEED = 0.7;
}
    public final static class Ports {
        public final static int MAIN_CONVEYER = 11;
        public final static int SHOOTER_MASTER = 6;
        public final static int SHOOTER_SLAVE = 7;

    }

    public final static class PIDBZ {
        public final static class Chassis{
            public final static double Kp = 1;
            public final static double Ki = 0.0;
            public final static double Kd = 0;
            public final static double B = 2.0;
            public final static double Zeta = 0.7;
            public final static double Ks = 0;
            public final static double Kv = 0.0;
            public final static double Ka = 0;
        }
        public final static class MainConveyer {
            public final static double Kp = 10;
            public final static double Ki = 0.0;
            public final static double Kd = 0.5;
            public final static double B = 2.0;
            public final static double Zeta = 0.7;
        }
    }

    public final static class Constraints {
        public final static class MainConveyer {
            public final static double maxVelocity = 9;
            public final static double maxAcceleration = 9;
            public final static VoltageConstraint voltageConstraint = new VoltageConstraint();

            private static class VoltageConstraint implements TrajectoryConstraint {
                public VoltageConstraint() {
                }

                @Override
                public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter,
                        double velocityMetersPerSecond) {
                    return maxVelocity;
                }

                @Override
                public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter,
                        double velocityMetersPerSecond) {
                    return new MinMax(0, maxAcceleration);
                }
            }
        }
    }
    public final static class Robot{
        public final static int SHIFTER_SELENOID_PORT = 4;
		public static boolean GYRO_REVERSED =false;
        public final static double POWER_CELL_DIAMETER = 0.177;
        public final static int MAX_POWER_CELLS = 5;

        public static final double TURN_TOLERANCE_DEG = 5;
        public static final double TURN_RATE_TOLERANCE_DEG_PERS = 10;

        
        public static final double ENCODER_CPR = 256 * 1.8 * 3 * 4;
        public static final double WHEEL_DIAMETER = 0.1524;
        public static final double ENCODER_DISTANCE_PER_PULSE = (WHEEL_DIAMETER * Math.PI) / ENCODER_CPR;
        
        public final static class MainConveyer{
            public final static double ACTIVATION_SPEED = 6;
            public final static double ENCODER_CPR = 256;
            public final static double DIAMETER = 0.1128; // the diameter of the conveyer strips' shaft
            public final static double GEAR_RATIO = 50;

        }
        public final static class Shooter{
            public final static double ACTIVATION_VOLTS = 16;
            public final static double ENCODER_CPR = 256;
            public final static double DIAMETER = 0.1;
			public static final double MAX_VOLTAGE = 12;
			public static final double THREE_METERS_PERCENTAGE = 1;
        }
        public final static class Sonars{
            //in milimeters
            public final static double CONVEYER_ENTRY_PLACEMENT = 50;
            public final static double SHOOTER_CONVEYOR_PLACEMENT = 100;
            public final static double TURRET_PLACEMENT = 250;
        }
    }
	public static final class OIConstants{
        public final static int LEFT_JOYSTICK_PORT=0;
        public final static int INTAKE_GRIPPER_BUTTON_PORT=3;
		public static final int INTAKE_LIFTER_BUTTON_PORT = 5;
		public static final int INTAKE_GRIPPER_EMERGENCY_BUTTON_PORT = 6;
        public static final int MAIN_CONVEYER_BUTTON_OUTSIDE = 2;
        public static final int MAIN_CONVEYER_BUTTON_INSIDE = 3;
    }
}
