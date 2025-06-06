// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static frc.robot.Constants.LiftConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;


public class LiftSubsystem extends SubsystemBase {
  public TalonFX m_liftFollower;
  public TalonFX m_liftLeader;

  public LiftSubsystem() {
    m_liftFollower = new TalonFX(kLift1ID);
    m_liftLeader = new TalonFX(kLift2ID);
    

    //right is the leader, left is the follower
    m_liftFollower.setControl(new Follower(m_liftLeader.getDeviceID(), false));
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands. 
   */

  public Command raiseLift() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setLiftSpeed(kLiftRaiseSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopLift();
        });
  }

  public Command lowerLift() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setLiftSpeed(-kLiftLowerSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopLift();
        });
  }

  public Command hang() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setLiftSpeed(-kLiftHangSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopLift();
        });
  }
  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLiftSpeed(double speed) {
    m_liftLeader.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stopLift() {
    m_liftLeader.set(0);
  }
}
