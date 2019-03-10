package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem{

  Talon talon;
  private static Intake inst;

  public static Intake getInstance(){
    if (inst==null){
      inst = new Intake(RobotConfig.intakePort);
    }
    return inst;
  }

  protected Intake(int port){
    this.talon = new Talon(port);
  }

  public void setSpeed(double speed){
    talon.set(speed);
  }

  public void stop(){
    this.setSpeed(0);
  }

  @Override
  protected void initDefaultCommand() {
    Joystick stick  = new Joystick(RobotConfig.intakeJoystickPort);
    setDefaultCommand(new IntakeTeleop(stick, RobotConfig.intakeJoystickAxis)); 
  }

}