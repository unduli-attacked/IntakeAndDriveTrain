package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.DriveTrain;
import frc.robot.OI;
import frc.robot.PWMDriveTrain;

public class TeleopDriving extends Command{

    // DriveTrain drivetrain = DriveTrain.getInstance();
    // Subsystem dt = drivetrain.getWpiSubsystem();
    PWMDriveTrain drivetrain = PWMDriveTrain.getInstance();
    PWMDriveTrain dt = drivetrain;

    public static enum DriveType{
        TANK, ARCADE, CURVATURE;
    }

    DriveType type = DriveType.ARCADE;
    public TeleopDriving(DriveType type) {
        requires(dt);
        this.type = type;
    }

    @Override
    protected void initialize() {
        drivetrain.setNeutralMode(NeutralMode.Brake);
        drivetrain.getLeftMotor().configOpenloopRamp(0.1);
        drivetrain.getRightMotor().configOpenloopRamp(0.1);
        drivetrain.tankDrive(0, 0);
        System.out.println("arcade drive command init");
    }

    @Override
    protected void execute() {
        double arg1 = OI.getForwardAxis();
        double arg2 = OI.getTurnAxis();

        switch(type){
            case TANK:
                //FIXME this uses forward and turn as left and right
                drivetrain.tankDrive(arg1, arg2);
            case ARCADE:
                drivetrain.arcadeDrive(arg1 * 1,
                    arg2 * 1);
            case CURVATURE:
                boolean isQuickTurn = false; //FIXME help i forgot how this was done
                drivetrain.curvatureDrive(arg1, arg2, isQuickTurn);
        }
        

    }

    @Override
    protected boolean isFinished() {return false;}

    @Override
    protected void end() {
        drivetrain.tankDrive(0, 0);
        System.out.println("arcade end called");
    }

    @Override
    protected void interrupted() {
        drivetrain.tankDrive(0, 0);
    }
}