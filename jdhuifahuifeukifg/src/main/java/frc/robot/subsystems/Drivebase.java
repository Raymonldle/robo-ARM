// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveBaseConstants;

import com.kauailabs.navx.frc.AHRS;


public class Drivebase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
   AHRS m_AHRS = new AHRS();
  PIDController m_PIDController = new PIDController(0,0,0);

  public Drivebase() {}

  // private final int diameter = 6;

  //className objectName = new classConstructor(arguments);

  CANSparkMax m_leftMaster  = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_leftSlave   = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_rightMaster = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_rightSlave  = new CANSparkMax(4, MotorType.kBrushless);

  MotorControllerGroup m_leftgroup  = new MotorControllerGroup(m_leftMaster,m_leftSlave);
  MotorControllerGroup m_rightgroup = new MotorControllerGroup(m_rightMaster,m_rightSlave);

  DifferentialDrive m_differentialDrive = new DifferentialDrive(m_rightMaster,m_leftMaster);
 
  RelativeEncoder m_leftEncoder  = m_leftMaster.getAlternateEncoder(Type.kQuadrature, DriveBaseConstants.kEncoderCount);
  RelativeEncoder m_rightEncoder = m_rightMaster.getAlternateEncoder(Type.kQuadrature, DriveBaseConstants.kEncoderCount);

 
  public void arcadeDrive(double forwardOrBackwardSpeed, double turnSpeed) {
    //objectName.methodName(arguments);
    m_differentialDrive.arcadeDrive(forwardOrBackwardSpeed, turnSpeed);
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getLeftDistanceInch() {
    return (m_leftEncoder.getPosition()/DriveBaseConstants.kEncoderCount * DriveBaseConstants.kWheelDiameter * Math.PI);
  }

  public double getRightDistanceInch() {
    return (m_rightEncoder.getPosition()/DriveBaseConstants.kEncoderCount * DriveBaseConstants.kWheelDiameter * Math.PI);
  }

  public double getAverageDistanceInch() {
    return (getAverageDistanceInch() + getLeftDistanceInch())/2;
  }

  public double getAngleX() {
    return ();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}