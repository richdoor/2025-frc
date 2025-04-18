// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class LiftConstants
  {
    public static final int kLift1ID = 17;
    public static final int kLift2ID = 18;
    public static final double kLiftRaiseSpeed = 0.4;
    public static final double kLiftLowerSpeed = 0.1;
    public static final double kLiftHangSpeed = 0.5;

    public static final double kLiftSetpoint1 = 0;
    public static final double kLiftSetpoint2 = 9.75;
    public static final double kLiftSetpoint3 = 22;
    public static final double kLiftSetpoint4 = 41.3;
    public static final double kEps_lift = 2;
    public static final double kLiftSlowDown = 5;
  }

  public static class ClawConstants
  {
    public static final int kRotationalMotorID = 15;
    public static final int kWheelMotorID = 16;
    public static final int kCoralSensorID = 14;
    public static final int kClawEncoderID = 19;
    
    public static final double kRotationalSpeed = 0.1;
    public static final double kWheelSpeed = 0.2;
    public static final double kEps_claw = 3;
    public static final double kAngleSlowDown = 5;

    public static final double kClawSetpoint1 = 0;
    public static final double kClawSetpoint2 = 4;
    public static final double kClawSetpoint3 = 22;

    public static final double kCoralDist = 0.1;
    public static final double kAddedRotations = 3.5;
  }

  public static class PipelineConstants
  {
    public static final int kPipeline_default = 0;
    public static final int kPipeline_reef = 1;
    public static final int kPipeline_coralStation = 2;
    public static final int kPipeline_processor = 3;
    public static final int kPipeline_barge = 4;
  }
  
  public static class AutoStrafeConstants
  {
    public static final double kTxOffset = 0;
    public static final double kStrafeEps = 2;
    public static final double kMaxStrafeSpeed = 0.3;
  }

  public static class SetYawConstants
  {
    public static final double kYawEps = 3;
    public static final double kYawSetpoint1 = 0;
    public static final double kYawSetpoint2 = 60;
    public static final double kYawSetpoint3 = 120;
    public static final double kYawSetpoint4 = 180;
    public static final double kYawSetpoint5 = 240;
    public static final double kYawSetpoint6 = 300;
  }
}
