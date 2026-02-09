package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.AlphaBots.NT;
import frc.robot.LimelightHelpers.PoseEstimate;

public class C_Align extends Command{
  public static class drivetrainThings{
    public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    public static final double minXposeErrorMetersToCorrect = Units.inchesToMeters(.9);//.6;
    public static final double minYposeErrorMetersToCorrect = Units.inchesToMeters(.9);//.6;
    public static final double minRZErrorToCorrect = .9;//1;//.45;//0.5;//1;//2;//1.25;

    public static  double k_PoseX_P = 2.5;//3.0;//2.1;//4;
    public static  double k_PoseX_I = 0.1;//.6;//0.0;//0.000001;//0.02;
    public static  double k_PoseX_D = 0.0;//.0;//0.06;

    public static  double k_PoseY_P = k_PoseX_P;//.5;//1.20;
    public static  double k_PoseY_I = k_PoseX_I;//0.000001;//0.02;
    public static  double k_PoseY_D = k_PoseX_D;//0.15;//0.002; 

    public static  double k_RZ_P = 0.11;//.05;
    public static  double k_RZ_I = 0.01;//0.00;
    public static  double k_RZ_D = 0.000000;//0.00;

    //if we are really far away lets keep pid from going insane.
    public static final double maxYvelocity = 1.0;
    public static final double maxXvelocity = 1.0;
    public static final double maxRZvelocity = MaxAngularRate /2;

    


}
    //Get ClassName to help network tables auto sort by creating a sub Table with the same name.
    String className = this.getClass().getSimpleName();
    public final Timer TimeToAlignTimer = new Timer();
    public final frc.robot.subsystems.CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    public final SwerveRequest.FieldCentric FCdriveAuton = new SwerveRequest.FieldCentric();//.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance); using blue alliance forward we dont need to mirror our drive commands. 

    private final PIDController AlignXPid = new PIDController(drivetrainThings.k_PoseX_P,drivetrainThings.k_PoseX_I,drivetrainThings.k_PoseX_D);
    private final PIDController AlignYPid = new PIDController(drivetrainThings.k_PoseY_P,drivetrainThings.k_PoseY_I,drivetrainThings.k_PoseY_D);
    private final PIDController AlignRZPid = new PIDController(drivetrainThings.k_RZ_P,drivetrainThings.k_RZ_I,drivetrainThings.k_RZ_D);



    public double MaxSpeedPercent = 1.0;//9; //6; // 6 meters per second desired top speed
    public double MaxAngularRatePercent = 1.0; //2.5 // 3/4 of a rotation per second max angular velocity

    double maxYvelocity = 1;
    double maxXvelocity = 1;
    double maxRZvelocity = 1;

    double minXposeErrorToCorrect = .03175; //.03175 Meters error is 1.25"
    double minYposeErrorToCorrect = .03175;
    double minRZDegreesErrorToCorrect = 1.0;

    Pose2d CurrentPose;//this is our latest position according to our chassis odometry
    Pose2d TargetPose;//this is where we wnt to go in field space coords X,y,Rotation
    Pose2d PoseOffset;//This is how far we are from where we want to be. this is CurrentPose minus TargetPose.

    public static final String PidAlignmentClassname = "pidAlignment";
    public static StructEntry<Pose2d> NT_AlignSetpoint = NT.getStructEntry_Pose2D("Poses","AlignSetpoint",new Pose2d());
    public static StringEntry NT_AlignedUsing = NT.getStringEntry(PidAlignmentClassname, "AlignedUsing", "none");
    public static DoubleEntry NT_TimeToAlign = NT.getDoubleEntry(PidAlignmentClassname, "TimeToAlign",0.0);
     public static DoubleEntry NT_XPGain = NT.getDoubleEntry(PidAlignmentClassname, "XP Gain",drivetrainThings.k_PoseX_P);
    public static DoubleEntry NT_XIGain = NT.getDoubleEntry(PidAlignmentClassname, "XI Gain",drivetrainThings.k_PoseX_I);
    public static DoubleEntry NT_XDGain = NT.getDoubleEntry(PidAlignmentClassname, "XD Gain",drivetrainThings.k_PoseX_D);

    public static DoubleEntry NT_ZPGain = NT.getDoubleEntry(PidAlignmentClassname, "ZP Gain",drivetrainThings.k_RZ_P);
    public static DoubleEntry NT_ZIGain = NT.getDoubleEntry(PidAlignmentClassname, "ZI Gain",drivetrainThings.k_RZ_I);
    public static DoubleEntry NT_ZDGain = NT.getDoubleEntry(PidAlignmentClassname, "ZD Gain",drivetrainThings.k_RZ_D);
    public static BooleanEntry NT_Xok = NT.getBooleanEntry(PidAlignmentClassname, "Xok", false);
    public static BooleanEntry NT_Yok = NT.getBooleanEntry(PidAlignmentClassname, "Yok", false);
    public static BooleanEntry NT_Zok = NT.getBooleanEntry(PidAlignmentClassname, "Zok", false);
    public C_Align(Pose2d PosePositionGoal){
        TargetPose = PosePositionGoal;
        AlignXPid.setSetpoint(TargetPose.getX());
        AlignYPid.setSetpoint(TargetPose.getY());
        AlignRZPid.setSetpoint(0);
       


        //when we startup an alignment pull the latest numbers to try from the user.
        
        NT_XPGain.set(NT_XPGain.getAsDouble());
        NT_XIGain.set(NT_XIGain.getAsDouble());
        NT_XDGain.set(NT_XDGain.getAsDouble());
        NT_ZPGain.set(NT_ZPGain.getAsDouble());
        NT_ZIGain.set(NT_ZIGain.getAsDouble());
        NT_ZDGain.set(NT_ZDGain.getAsDouble());
        addRequirements(drivetrain);
    }
    Alliance allianceOnInit;//DriverStation.getAlliance().get();
    @Override
    public void initialize() {
        allianceOnInit = DriverStation.getAlliance().get();
        CurrentPose = drivetrain.getState().Pose;    
        NT_AlignSetpoint.set(TargetPose); 
        //setposeoffsets();
        AlignXPid.reset();
        AlignYPid.reset();
        AlignRZPid.reset();
        TimeToAlignTimer.restart();
    }

    

    @Override
    public void execute() {
        setposeoffsets();
        //PID
        double RZAdjust = AlignRZPid.calculate(PoseOffset.getRotation().getDegrees());
        double xpose_adjust = AlignXPid.calculate(CurrentPose.getX());//GetXPoseAdjust(XP_buffer, min_xpose_command);
        double Ypose_adjust = AlignYPid.calculate(CurrentPose.getY());//GetYPoseAdjust(YP_buffer, min_Ypose_command );    
        //drive drive drivetrain with PID clamped something to not go crazy or something
        //clamp all results to a max (and negative max) top speed
        Ypose_adjust = MathUtil.clamp(Ypose_adjust, -maxYvelocity, maxYvelocity);
        xpose_adjust = MathUtil.clamp(xpose_adjust, -maxXvelocity, maxXvelocity);
        RZAdjust = MathUtil.clamp(RZAdjust, -maxRZvelocity, maxRZvelocity);

        //MOVE!!!
        MoveRobotToTargetInFieldCoordinates(Ypose_adjust, xpose_adjust, RZAdjust);

        //tune:
        PidTune();
        PidTuneRotation();
    }

    private void setposeoffsets() {
      if(CurrentPose == null){System.err.println("CurrentPose missing"); return;}
      if(CurrentPose == null){System.err.println("TargetPose missing");  return;}
      //System.out.println("Running setposeoffsets");
      //get position
      PoseEstimate frontLimelightMt1 =  LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      PoseEstimate LimelightMt1 = frontLimelightMt1;
  

      double TagdistMaxMeters = 6;
      boolean shoulduseLLMT1Pose = LimelightMt1 !=null && LimelightMt1.tagCount > 0;// & LimelightMt1.avgTagDist < TagdistMaxMeters;
      if(shoulduseLLMT1Pose){
        CurrentPose = LimelightMt1.pose;
        NT_AlignedUsing.set("LimelightMt1");
      }
      else
      {
        CurrentPose = drivetrain.getState().Pose;
        NT_AlignedUsing.set("chassisPose");
      }
      
      //get offsets
      //SUBTRACT where we need to go, from where we are. this will give us the translations we need to make 
      double Xpose_Offset = CurrentPose.getX() - TargetPose.getX();
      double Ypose_Offset = CurrentPose.getY() - TargetPose.getY();             
      Rotation2d RZ_Offset2 = CurrentPose.getRotation().minus(TargetPose.getRotation());
     
      PoseOffset = new Pose2d(Xpose_Offset, Ypose_Offset, RZ_Offset2);
      if(PoseOffset == null){System.err.println("No pose OFFSET created");}
    }
        // If "isfinished" end true OR if we cancel this command for some reason. 
    // we need some actions to happen no matter what. 
    @Override
    public void end(boolean interrupted) {
      StopDriveTrain();
      NT_AlignedUsing.set("End Align");
      NT_TimeToAlign.set(TimeToAlignTimer.get());
      
    }


    public final Timer SettleDebounceTimer = new Timer();
    private double debounceSecondsNeeded = .02;

    @Override
    public boolean isFinished(){
      if(PoseOffset == null){System.err.println("No pose OFFSET! Broken CODE?"); return false;}
        //near the final positon x
        boolean Xok = IsXInTarget();
        boolean Yok = IsYInTarget();
        boolean Zok = isRotInTarget();

        NT_Xok.set(Xok);
        NT_Yok.set(Yok);
        NT_Zok.set(Zok);
        //near the final positon y
        //near the final positon z (rotation)
        //stop driving
        boolean isatSetpos = Xok && Yok  && Zok;
    
        if(isatSetpos){
          if(SettleDebounceTimer.get() > debounceSecondsNeeded)
            {
            //Stop movement if we are there.
            StopDriveTrain();
            //
            return true;
            }             
        }
        else{
            SettleDebounceTimer.restart();
            

        }
        return false;
        
    }

    public void MoveRobotToTargetInFieldCoordinates(double YposeAxis, double XposeAxis, double RZposeAxis) {
      
      var xyMirrorRed = (DriverStation.getAlliance().get() == Alliance.Blue) ? 1.0:-1.0; //our drivetrain auto flips itself when we are on red. so we have to aswell. 

        drivetrain.setControl(FCdriveAuton
            .withVelocityX(XposeAxis * MaxSpeedPercent * xyMirrorRed) // Drive forward with // negative Y (forward)
            .withVelocityY(YposeAxis * MaxSpeedPercent * xyMirrorRed) // Drive left with negative X (left)
            .withRotationalRate(RZposeAxis * MaxAngularRatePercent) // Drive counterclockwise with negative X (left)
        );
      }
      public void StopDriveTrain() {
        drivetrain.setControl(FCdriveAuton.withVelocityX(0) // Drive forward with // negative Y (forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        );
      }

      private boolean IsXInTarget() {
        return Math.abs(PoseOffset.getX()) < minXposeErrorToCorrect;
      }
    
      private boolean IsYInTarget() {
        return Math.abs(PoseOffset.getY()) < minYposeErrorToCorrect;
      }
    
      private boolean isRotInTarget() {
        return Math.abs(PoseOffset.getRotation().getDegrees()) < minRZDegreesErrorToCorrect;
      }

      //This is used with the SmartDashboard to Tune the PID. Unneeded for competition.
  private void PidTune() {
    double p = NT_XPGain.getAsDouble();
    double i = NT_XIGain.getAsDouble();
    double d = NT_XDGain.getAsDouble();
    
      
    if((p != AlignXPid.getP())) { AlignXPid.setP(p); drivetrainThings.k_PoseX_P = p;}
    if((i != AlignXPid.getI())) { AlignXPid.setI(i); drivetrainThings.k_PoseX_I = i;}
    if((d != AlignXPid.getD())) { AlignXPid.setD(d); drivetrainThings.k_PoseX_D = d;}

    if((p != AlignYPid.getP())) { AlignYPid.setP(p); drivetrainThings.k_PoseY_P = p;}
    if((i != AlignYPid.getI())) { AlignYPid.setI(i); drivetrainThings.k_PoseY_I = i;}
    if((d != AlignYPid.getD())) { AlignYPid.setD(d); drivetrainThings.k_PoseY_D = d;}
  }
  private void PidTuneRotation() {
    double p = NT_ZPGain.getAsDouble();
    double i = NT_ZIGain.getAsDouble();
    double d = NT_ZDGain.getAsDouble();
    
      
    if((p != AlignRZPid.getP())) { AlignRZPid.setP(p); drivetrainThings.k_RZ_P = p;}
    if((i != AlignRZPid.getI())) { AlignRZPid.setI(i); drivetrainThings.k_RZ_I = i;}
    if((d != AlignRZPid.getD())) { AlignRZPid.setD(d); drivetrainThings.k_RZ_D = d;}

  }
    
}
