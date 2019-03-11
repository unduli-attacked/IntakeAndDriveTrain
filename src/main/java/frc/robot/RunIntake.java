package frc.robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

public class RunIntake extends TimedCommand{
    double demand;
    public RunIntake(double percentDemand, double timeout){
        super(timeout);
        requires(Intake.getInstance());
        this.demand = percentDemand;
    }

    @Override
    protected void initialize(){
        Intake.getInstance().setSpeed(demand);
    }

    @Override
    protected void execute(){
        Intake.getInstance().setSpeed(demand);
    }

    @Override
    protected void end(){
        Intake.getInstance().setSpeed(0);
    }

    @Override
    protected void interrupted(){
        Intake.getInstance().setSpeed(0);
    }
}