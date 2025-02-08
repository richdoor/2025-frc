// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// this is a comment

package frc.robot.subsystems.swervedrive;

import static frc.robot.Constants.ClawConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;


public class ClawSubsystem extends SubsystemBase {
  TalonFX m_RotationalMotor;
  TalonFX m_WheelMotor;

  public ClawSubsystem() {
    m_RotationalMotor = new TalonFX(kRotationalMotor);
    m_WheelMotor = new TalonFX(kWheelMotor);
    

    //right is the leader, left is the follower
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands. 
   */

  public Command ClawUp() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setClawRotationSpeed(kRotationalSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopClawRotation();
        });
  }

  public Command ClawDown() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setClawRotationSpeed(-kRotationalSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopClawRotation();
        });
  }

  public Command wheelForward() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setClawWheelSpeed(kWheelSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopClawWheel();
        });
  }

  public Command wheelBackward() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setClawWheelSpeed(-kWheelSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stopClawWheel();
        });
  }
  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setClawRotationSpeed(double speed) {
    m_RotationalMotor.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stopClawRotation() {
    m_RotationalMotor.set(0);
  }

  public void setClawWheelSpeed(double speed) {
    m_WheelMotor.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stopClawWheel() {
    m_WheelMotor.set(0);
  }
}
