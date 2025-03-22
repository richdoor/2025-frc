// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ClawConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.swervedrive.ClawSubsystem;


public class CoralIntake_copy extends Command {
  ClawSubsystem m_claw;
  double setpoint;
  boolean addedSpin = false;
  boolean atSensor;
  double positionAtSensor;
  //double coralIntakeTolerance = 0.005;

  /** Creates a new Command. */
  public CoralIntake_copy(ClawSubsystem claw) {
    m_claw = claw;

    //means this command will take priority over others using same subsystem
    addRequirements(m_claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.m_WheelMotor.set(-kWheelSpeed);
    m_claw.m_WheelMotor.setPosition(0.0);
    atSensor = false;
    addedSpin = false;
    System.out.println("in command init");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // fluctuate between +- 0.005 from kCoralDist
      if ((m_claw.m_coralDist.getDistance().getValueAsDouble()<=kCoralDist) && atSensor == false && (m_claw.m_coralDist.getDistance().getValueAsDouble()!=0.0))
      {
        positionAtSensor = -m_claw.m_WheelMotor.getPosition().getValueAsDouble();
        atSensor = true;
      } else {
        atSensor = false;
      }
    System.out.println("in command excecute");
    System.out.println("CANRange Pos "+ m_claw.m_coralDist.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Intake Motor Pos", positionAtSensor);
    SmartDashboard.putBoolean("At Sensor", atSensor);
    SmartDashboard.putNumber("CANRange Pos", m_claw.m_coralDist.getDistance().getValueAsDouble());
    System.out.println("motor position" + positionAtSensor);
    System.out.println("atsensor"+atSensor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stopClawWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    System.out.println("in command isfinished");
    System.out.println("desired position" + (positionAtSensor+kAddedRotations));

    if (-m_claw.m_WheelMotor.getPosition().getValueAsDouble() > (positionAtSensor + kAddedRotations))
    {
      addedSpin = true;
    }
    return (addedSpin == true && m_claw.m_coralDist.getDistance().getValueAsDouble()<=kCoralDist && m_claw.m_coralDist.getDistance().getValueAsDouble()!=0.0);
  }
}
