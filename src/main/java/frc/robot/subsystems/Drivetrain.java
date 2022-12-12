// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  public WPI_TalonFX l1, l2, r1, r2;
  public MotorControllerGroup l, r;
  public DifferentialDrive ddrive;

  // public Solenoid shifterL, shifterR;
 // public AHRS gyro = new AHRS(SerialPort.Port.kUSB1);
 // public DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyroAngle)

  public Drivetrain() {
    //initalize all the motors
    l1 = new WPI_TalonFX(Constants.MOTOR_L1_ID);
    l2 = new WPI_TalonFX(Constants.MOTOR_L2_ID);
    r1 = new WPI_TalonFX(Constants.MOTOR_R1_ID);
    r2 = new WPI_TalonFX(Constants.MOTOR_R2_ID);
    //set the right side as inverted
    r1.setInverted(true);
    r2.setInverted(true);
    //make the two motors on each side follow each other
    l2.follow(l1);
    r2.follow(r1);
    //group the two motors on each side
    l = new MotorControllerGroup(l1, l2);
    r = new MotorControllerGroup(r1, r2);
    //ddrive controls all of the robot
    ddrive = new DifferentialDrive(l, r);
    
  }

  public void move(double power, double offset){ 
    // power is the throttle (drive stick), offset is turning (turn stick)
    ddrive.arcadeDrive(power, offset);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
