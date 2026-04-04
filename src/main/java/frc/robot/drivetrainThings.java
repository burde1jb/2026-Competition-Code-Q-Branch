package frc.robot;

import edu.wpi.first.math.util.Units;

public class drivetrainThings{
    public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    public static final double minXposeErrorMetersToCorrect = Units.inchesToMeters(6);//.6;
    public static final double minYposeErrorMetersToCorrect = Units.inchesToMeters(6);//.6;
    public static final double minRZErrorToCorrect = 4;//1;//.45;//0.5;//1;//2;//1.25; //Degrees of error

    public static  double k_PoseX_P = 3.0;//3.0;//2.1;//4;
    public static  double k_PoseX_I = 0.1;//.6;//0.0;//0.000001;//0.02;
    public static  double k_PoseX_D = 0.0;//.0;//0.06;

    public static  double k_PoseY_P = k_PoseX_P;//.5;//1.20;
    public static  double k_PoseY_I = k_PoseX_I;//0.000001;//0.02;
    public static  double k_PoseY_D = k_PoseX_D;//0.15;//0.002; 

    public static  double k_RZ_P = 0.11;//.05;
    public static  double k_RZ_I = 0.01;//0.00;
    public static  double k_RZ_D = 0.000000;//0.00;

    //if we are really far away lets keep pid from going insane.
    public static final double maxYvelocity = 2.5;
    public static final double maxXvelocity = 2.5;
    public static final double maxRZvelocity = MaxAngularRate /2;
  }