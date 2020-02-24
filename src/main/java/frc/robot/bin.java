
package frc.robot;
// This is simply a "junk yard" to declutter the rest of the .java files with stuff that isn't 
// needed yet. 
// Ergo, the Island of Unwanted Classes.
//--------------------------------------------------------------------------------------------\\
// ***Robot Section:

// Since we are not using a gyro yet, this is not necessary. 02/23/20
    /*
    public void setAngle() {
        if (controller.getPOV() == 0) {
            base.setAngle(0);
        } else if (controller.getPOV() == 90) {
            base.setAngle(90);
        } else if (controller.getPOV() == 180) {
            base.setAngle(180);
        } else if (controller.getPOV() == 270) {
            base.setAngle(270);
        }
    }

    public void reseter() {  
        // This was taken out since it was not necessary for build. 02/23/20

        if (controller.getAButtonReleased()) {
            //base.reset(); //Could cause locking, we don't have gyro, need to look at reset
            //System.out.print("Base Reset.");
        }
        */



//--------------------------------------------------------------------------------------------\\
// ***DriveBase Section:

/*
    public void Lift() {
        if (controller.getAngle)
    }

    public void snapToAngle() {
        //double turn = turnController.calculate(getAngle());

        leftfrontmotor.set(turn);
        leftrearmotor.set(turn);

        rightfrontmotor.set(turn);
        rightrearmotor.set(turn);
    }

  
    public void ballFollowDrive() {
        if (camera.getValues()[1] != 0) {
            tx_prev = camera.getValues()[1];
        }

        if (camera.getValues()[0] == 0) {
            ballSeekDrive();
        } else {
            double throttle = -ballDriveController.calculate(camera.getValues()[2]);
            double turn = -ballTurnController.calculate(camera.getValues()[1]);

            leftfrontmotor.set(turn - throttle);
            leftrearmotor.set(turn - throttle);

            rightfrontmotor.set(turn + throttle);
            rightrearmotor.set(turn + throttle);
        }
        
    }

    private void ballSeekDrive() {
        if (tx_prev > 0) {
            leftfrontmotor.set(0.2);
            leftrearmotor.set(0.2);

            rightfrontmotor.set(0.2);
            rightrearmotor.set(0.2);
        } else if (tx_prev < 0) {
            leftfrontmotor.set(-0.2);
            leftrearmotor.set(-0.2);

            rightfrontmotor.set(-0.2);
            rightrearmotor.set(-0.2);
        }
    }
*/



//--------------------------------------------------------------------------------------------\\
// Limelight Section (Entire File "Limelight.java"):

/*
package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    public Limelight(Pipeline pipeline) {
        int pipelineID;

        switch (pipeline) {
            case RETRO:
                pipelineID = 0;
                break;
            case BALL:
                pipelineID = 1;
                break;
            default:
                throw new IllegalArgumentException();
        }

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineID);
    }

    public Pipeline getPipeline() {
        int pipelineID = (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getDouble(-1);

        switch (pipelineID) {
            case 0:
                return Pipeline.RETRO;
            case 1:
                return Pipeline.BALL;
            default:
                return Pipeline.INVALID;
        }
    }

    public void setPipeline(Pipeline pipeline) {
        int pipelineID;

        switch (pipeline) {
            case RETRO:
                pipelineID = 0;
                break;
            case BALL:
                pipelineID = 1;
                break;
            default:
                throw new IllegalArgumentException();
        }

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineID);
    }

    public double[] getValues() {
        double[] values = new double[3];

        values[0] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        values[1] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        values[2] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        return values;
    }
}
*/



//--------------------------------------------------------------------------------------------\\
// TurnAction Section (Entire File "TurnAction.java"):

/*
package frc.robot.actions;

import frc.robot.Robot;

public class TurnAction implements Action {

    private double angle;

    private boolean complete = false;

    /*
     *-1: Task failed
     *0: Task not started
     *1: Task in progress
     *2: Task complete
     
    private int status = 0;

    public TurnAction(double target) {
        angle = target;
    }

    public boolean isComplete() {
        return complete;
    }

    public void start() {
        status = 1;

        Robot.base.reset();
        Robot.base.setAngle(angle);
        Robot.base.snapToAngle();

        int counter = 0;

        while (!Robot.base.turnOnTarget()) {
            counter++;
            Robot.base.snapToAngle();

            if (counter == 10000) {
                status = -1;
                return;
            }

            continue;
        }

        complete = true;
        status = 2;
    }

    public int status() {
        return status;
    }
}
*/



//--------------------------------------------------------------------------------------------\\
// OneTimeAction Section (Entire File "OneTimeAction.java"):

/*
package frc.robot.actions; 

public abstract class OneTimeAction implements Action {
    @Override public boolean isFinished(){ return true; }
    @Override public void start (){ run(); }
    @Override public int status(){}
    @Override public void update(){}
    @Override public void done  (){}
    public abstract void run();
}
*/



//--------------------------------------------------------------------------------------------\\
