// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// IMPORTS
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {
  // Instance variables: motors and encoders
  // Create 4 brushless motors to be our drive motors
  private SparkMax m_rightFrontMotor;
  private SparkMax m_leftFrontMotor;
  private SparkMax m_rightBackMotor;
  private SparkMax m_leftBackMotor;

  // Create a variable to store gearbox info
  private DCMotor gearbox = DCMotor.getNEO(1);

  // Create 4 encoders to track the values of the motors
  private SparkAbsoluteEncoder m_rightFrontEncoder;
  private SparkAbsoluteEncoder m_leftFrontEncoder;
  private SparkAbsoluteEncoder m_rightBackEncoder;
  private SparkAbsoluteEncoder m_leftBackEncoder;
  
  // Create the drive train
  private DifferentialDrive m_chassis;

  // Create a simulated feild
  Field2d m_feild = new Field2d();

  // Sim motors
  private SparkMaxSim m_simRightFrontMotor;
  private SparkMaxSim m_simLeftFrontMotor;

  // Sim encoder
  private SparkAbsoluteEncoderSim m_simRightFrontEncoder;
  private SparkAbsoluteEncoderSim m_simLeftFrontEncoder;

  // Sim chassis
  private DifferentialDrivetrainSim m_simChassis;

  public DriveSubsystem() {
    // Initialize the motors based on port number
    m_rightFrontMotor = new SparkMax(Constants.DriveConstants.k_rightFrontPort, MotorType.kBrushless);
    m_leftFrontMotor = new SparkMax(Constants.DriveConstants.k_leftFrontPort, MotorType.kBrushless);
    m_rightBackMotor = new SparkMax(Constants.DriveConstants.k_rightBackPort, MotorType.kBrushless);
    m_leftBackMotor = new SparkMax(Constants.DriveConstants.k_leftBackPort, MotorType.kBrushless);

    // Define the encoders
    m_rightFrontEncoder = m_rightFrontMotor.getAbsoluteEncoder();
    m_leftFrontEncoder = m_leftFrontMotor.getAbsoluteEncoder();
    m_rightBackEncoder = m_rightBackMotor.getAbsoluteEncoder();
    m_leftBackEncoder = m_leftBackMotor.getAbsoluteEncoder();

    // Define the sim motors
    m_simRightFrontMotor = new SparkMaxSim(m_rightFrontMotor, gearbox);
    m_simLeftFrontMotor = new SparkMaxSim(m_leftFrontMotor, gearbox);

    // Define the sim encoders
    m_simRightFrontEncoder = m_simRightFrontMotor.getAbsoluteEncoderSim();
    m_simLeftFrontEncoder = m_simLeftFrontMotor.getAbsoluteEncoderSim();

    // TODO: Update using configs
    // Invert the right motors
    m_rightBackMotor.setInverted(true);
    m_rightFrontMotor.setInverted(true);

    // Create an object to allow us to drive the robot
    m_chassis = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);

    // Create a simulated drive train
  }

  public void drive(double power, double rotation){
    m_chassis.arcadeDrive(power, rotation);
    m_simRightFrontMotor.setVelocity(power);
  }

  
  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // Update field
    m_feild.setRobotPose(new Pose2d(new Translation2d(0, 0), new Rotation2d()));

    // Put values to SmartDashboard
    SmartDashboard.putNumber("LeftFront Motor Speed", m_simLeftFrontEncoder.getVelocity());
    SmartDashboard.putNumber("RightFront Motor Speed", m_simRightFrontEncoder.getVelocity());
    SmartDashboard.putData("Feild", m_feild);
  }
}
