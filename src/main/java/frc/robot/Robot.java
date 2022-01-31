// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  


   private WPI_TalonSRX leftFront = new WPI_TalonSRX(1);
   private WPI_TalonSRX leftBack = new WPI_TalonSRX(2);
   private WPI_TalonSRX rightFront = new WPI_TalonSRX(3);
   private WPI_TalonSRX rightBack = new WPI_TalonSRX(4);

   private MecanumDrive drive = new MecanumDrive(leftFront,leftBack,rightFront,rightBack);
   private Joystick driverJoystick = new Joystick(0);
   private Joystick operatorJoystick = new Joystick(1);

   // unit conversion
   private final double kDriveRotations2Feet = Math.PI / 96;
  public void robotInit() 
  {
    //Inverted Settings
    leftFront.setInverted(true);
    rightFront.setInverted(false);

    // Follow setup
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    leftBack.setInverted(InvertType.FollowMaster);
    rightBack.setInverted(InvertType.FollowMaster);

  
    //init encoders
    leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,10);
    
    rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,10);
    
    leftFront.setSensorPhase(false);
    rightFront.setSensorPhase(true);
    leftBack.setSensorPhase(true);

    //reset encoders to 0
    leftFront.setSelectedSensorPosition(0,0,10);
    rightFront.setSelectedSensorPosition(0,0,10);
    
  }

  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.putNumber("Right Drive Value", leftFront.getSelectedSensorPosition() * kDriveRotations2Feet);

    SmartDashboard.putNumber("Left Drive Value", leftFront.getSelectedSensorPosition() * kDriveRotations2Feet);
  }

  @Override
  public void autonomousInit() 
  {
    enableMotors(true);
    leftFront.setSelectedSensorPosition(0,0,10);
    rightFront.setSelectedSensorPosition(0,0,10);
    errorSum = 0;
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = 0;
  }

  final double kP = 0.5;
  final double kI = 0.05;
  final double kD = 0.1;
  final double iLimit = 1;

  double setpoint = 5;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0; 

  @Override
  public void autonomousPeriodic() 
  {
    double leftPosition = leftFront.getSelectedSensorPosition() * kDriveRotations2Feet;
    double rightPosition = rightFront.getSelectedSensorPosition() * kDriveRotations2Feet;
    double distance = (leftPosition + rightPosition)/2;
    
    
    
    //Calculations
    double error = setpoint - distance;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    
    if(Math.abs(error) < iLimit)
    {
    errorSum += error * dt;
    }

    double errorRate = (error - lastError) /dt ;
    double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

    leftFront.set(outputSpeed);
    rightFront.set(outputSpeed);
    System.out.println("This line of code ran!");

    // update last - variables
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;
  }
  


  @Override
  public void teleopInit() 
  {
    enableMotors(true);
    
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    enableMotors(false);
  }


  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  private void enableMotors(boolean on)
  {
    NeutralMode mode;

    if(on) 
    {
      mode = NeutralMode.Brake;
    }
    else
    {
      mode = NeutralMode.Coast;
    }
    
  leftFront.setNeutralMode(mode);
  leftBack.setNeutralMode(mode);
  rightFront.setNeutralMode(mode);
  rightBack.setNeutralMode(mode);
  }
}
