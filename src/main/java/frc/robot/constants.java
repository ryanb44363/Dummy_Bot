package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

//import frc.robot.sensors.Limelight;
import frc.robot.sensors.Pipeline;

public class constants {


    public class DriveBase {
        
        /*
        //This is to serve as an example:
        public static final int leftMaster = 1;
        public static final int leftSlave1 = 9;
        public static final int leftSlave2 = 8;
        public static final int rightMaster = 2;
        public static final int rightSlave1 = 7;
        public static final int rightSlave2 = 6;
        */

        /*
        private WPI_TalonSRX leftMaster_ = new WPI_TalonSRX(Constants.Drive.leftMaster);
        */
    }
    public class Dummy {
        // This is an example from Dummy.java:
        //public WPI_TalonSRX Talon_Motor; 

        //public static final double TalonMotor = 1;  //This did not work for whatever reason (Could not find class from Dummy.class)
        //public static final double VictorMotor = 2; //This did not work for whatever reason (Could not find class from Dummy.class)
    }
    public static final int TalonMotor = 0;
    public static final int VictorMotor = 1;

}
