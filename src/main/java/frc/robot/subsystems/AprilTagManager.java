package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaBots.Tools;
import frc.robot.commands.C_Align;
import frc.robot.Robot;
import frc.robot.AlphaBots.AprilTag;
import frc.robot.AlphaBots.AprilTag.TagType;
import frc.robot.AlphaBots.NT;
import frc.robot.RobotConstants;

public class AprilTagManager extends SubsystemBase
  {
    //public static ArrayList<AprilTag> tagList = new ArrayList<AprilTag>(22);
    public static double RobotDefaultOffset = 4.50;//adds a 1/4 inch extra space at locations. 
    public static double bumperthickness = 3.00*2; //real 3.75"
    public static double robotsize = 27/2;//size no bumpers divided by 2
    public static double robotmetersdistToCenter = Units.inchesToMeters(RobotDefaultOffset+robotsize+bumperthickness);
    
    public static double TowerOffset = RobotConstants.TowerOffset;
    public static double TowerOffsetRight = RobotConstants.TowerOffsetRight;
    public static double HubOffset = RobotConstants.HubOffset;
    public static double HubOffsetLeft = RobotConstants.HubOffsetLeft;

    public static final double TowerCenterOffset = Units.inchesToMeters(20)/2;// used during test Units.inchesToMeters(12.875)/2; //Reef Width CenteronCenter divided in half
    public static final double HubCenterOffset = Units.inchesToMeters(24)/2; //hub Width CenteronCenter divided in half
    // public static final double ExtraMetersoffsetForAlgaePickup = Units.inchesToMeters(7.25);
    public static final double OutpostOffset = Units.inchesToMeters(-0.5); //LIVE
    //public static double final SourceOffset =  Units.inchesToMeters(60); //TESTING ONLY
    
    StructEntry<Pose2d> NT_myloc = NT.getStructEntry_Pose2D("Poses","TagLoc",new Pose2d());
    StructEntry<Pose2d> NT_myStraightloc = NT.getStructEntry_Pose2D("Poses","StraightLoc",new Pose2d());//straight out from the april tag. a centered pick for de-algae/processor/pickup 
    StructEntry<Pose2d> NT_Leftloc = NT.getStructEntry_Pose2D("Poses","leftLoc",new Pose2d());//LEFT FROM Robot TOWARDS tag view!
    StructEntry<Pose2d> NT_rightloc = NT.getStructEntry_Pose2D("Poses","rightLoc",new Pose2d());//Right Given View Robot TOWARDS tag!
    StructEntry<Pose2d> NT_ClosestTag = NT.getStructEntry_Pose2D("Poses","ClosestTag",new Pose2d());//shows closest tag to robotchassis
    StructEntry<Pose2d> NT_Simloc = NT.getStructEntry_Pose2D("Poses","ChassisLoc",new Pose2d());//NT_Simloc
    StructEntry<Pose2d> NT_ClosestTower = NT.getStructEntry_Pose2D("Poses","ClosestTower",new Pose2d());
    StructEntry<Pose2d> NT_ClosestHub = NT.getStructEntry_Pose2D("Poses","ClosestHub",new Pose2d());
    StructEntry<Pose2d> NT_ClosestOutpost = NT.getStructEntry_Pose2D("Poses","ClosestOutpost",new Pose2d());
    StructEntry<Pose2d> NT_ClosestBump = NT.getStructEntry_Pose2D("Poses","ClosestBump",new Pose2d());
    StructEntry<Pose2d> NT_ClosestTrench = NT.getStructEntry_Pose2D("Poses","ClosestTrench",new Pose2d());
    // StructEntry<Pose2d> NT_ClosestBarge = NT.getStructEntry_Pose2D("Poses","ClosestBarge",new Pose2d());
    //DoubleEntry NT_tagID = NT.getDoubleEntry("", "TagID", chosenAprilTagID);
    
    public CommandSwerveDrivetrain drivetrain;
    public int chosenAprilTagID = 0;
    public AprilTagManager(CommandSwerveDrivetrain _drivetrain)
    {
      drivetrain =_drivetrain;
    } 

    
    @Override
    public void periodic() {
      //NT_myloc.set(ourtag.Pose);
      Pose2d SimRobotChassisLoc = null;
      if(Robot.isSimulation())
      {
        SimRobotChassisLoc = new Pose2d();//Robot.SimRobotChassisLoc;
      }
      else
      {
        SimRobotChassisLoc = drivetrain.getState().Pose;
      }
      
     
      // NT_myStraightloc.set(AprilTagManager.getStraightOutLoc(chosenAprilTagID, Units.inchesToMeters(6)));
      // NT_Leftloc.set(getOffSet90Loc(chosenAprilTagID, Units.inchesToMeters(6), constants.ReefWidthCenteronCenter,true));
      // NT_rightloc.set(getOffSet90Loc(chosenAprilTagID, Units.inchesToMeters(6), constants.ReefWidthCenteronCenter,false));
      if(DriverStation.isDSAttached())
      {
        NT_Simloc.set(SimRobotChassisLoc);
        NT_ClosestTower.set(getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,frc.robot.AlphaBots.AprilTag.TagType.Tower).Pose);
        NT_ClosestHub.set(getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Hub).Pose);
        NT_ClosestOutpost.set(getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Outpost).Pose);
        NT_ClosestBump.set(getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Bump).Pose);
        NT_ClosestTrench.set(getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,TagType.Trench).Pose);
        // NT_ClosestBarge.set(getClosestTagofTypeToRobotCenter(SimRobotChassisLoc,(DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? TagType.BlueBarge:TagType.RedBarge).Pose);
        NT_ClosestTag.set(getClosestTagToRobotCenter(SimRobotChassisLoc).Pose);
      }
  
    }

    public static AprilTag getTagbyID(int _ID)
    {
      for (AprilTag aprilTag : tagList) {
        if (aprilTag.ID == _ID)
        {
          return aprilTag;
        }
      }
      System.err.println("No Tag found for ID :" + _ID);
        return new AprilTag(0, "NotFound", new Pose2d(), 0,Alliance.Blue);
    }
    public static AprilTag getClosestTagToRobotCenter(Pose2d RobotLoc)
    {
      double closestSoFar = 99999;
      AprilTag closesAprilTag = new AprilTag(0, "NotFound", new Pose2d(), 0,Alliance.Blue);
      for (AprilTag aprilTag : tagList) {
        double distToThisPose = Tools.getdistancetopose(RobotLoc, aprilTag.Pose);
        if (distToThisPose < closestSoFar)
        {
          closestSoFar = distToThisPose;
          closesAprilTag = aprilTag;
        }
      }
        return closesAprilTag;
    }

    public static AprilTag getClosestTagofTypeToRobotCenter(Pose2d RobotLoc,TagType Type)
    {
      double closestSoFar = 99999;
      AprilTag closesAprilTag = new AprilTag(0, "NotFound", new Pose2d(), 0,Alliance.Blue);
      for (AprilTag aprilTag : tagList) {
        double distToThisPose = Tools.getdistancetopose(RobotLoc, aprilTag.Pose);
        if (aprilTag.tagType == Type && distToThisPose < closestSoFar)
        {
          closestSoFar = distToThisPose;
          closesAprilTag = aprilTag;
        }
      }
        return closesAprilTag;
    }
    /// get the tag that is the closest to the robots center, for a given tag type, and for the robots alliance.
    public static AprilTag getClosestTagofTypeToRobotCenterForAlliance(Pose2d RobotLoc,TagType Type)
    {
      double closestSoFar = 99999;
      AprilTag closesAprilTag = new AprilTag(0, "NotFound", new Pose2d(), 0,Alliance.Blue);
      if(!DriverStation.getAlliance().isPresent()){return closesAprilTag;}//no alliance? return default. 

      Alliance thisAlliance =  DriverStation.getAlliance().get();
      
      for (AprilTag aprilTag : tagList) {
        double distToThisPose = Tools.getdistancetopose(RobotLoc, aprilTag.Pose);
        if (aprilTag.tagType == Type && aprilTag.tagsAlliance == thisAlliance && distToThisPose < closestSoFar)
        {
          closestSoFar = distToThisPose;
          closesAprilTag = aprilTag;
        }
      }
        return closesAprilTag;
    }
    // public static AprilTag getClosestSourceToRobotCenter(Pose2d RobotLoc)
    // {
    //   AprilTag LeftAprilTag = getTagbyID(GetLeftSourceID());
    //   AprilTag RightAprilTag = getTagbyID(GetRightSourceID());
    //   double LeftdistToThisPose = Tools.getdistancetopose(RobotLoc, LeftAprilTag.Pose);
    //   double RightdistToThisPose = Tools.getdistancetopose(RobotLoc, RightAprilTag.Pose);
      
        
    //     if (LeftdistToThisPose < RightdistToThisPose)
    //     {
    //       return LeftAprilTag;
    //     }
    //     return RightAprilTag;
    // }
    
    //public static int GetLeftSourceID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 13:1;}//first num blue second red.
    //public static int GetRightSourceID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 12:2;}//first num blue second red.

    //public static int GetProcessorID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 16:3;}//first num blue second red.
    //public static int GetBargeID(){return (DriverStation.getAlliance().isPresent() & DriverStation.getAlliance().get().equals(Alliance.Blue))? 14:5;}//first num blue second red.

    public static Pose2d getStraightOutLoc(int TagID,double MetersFromAprilTag)
    {
        AprilTag Thistag = getTagbyID(TagID);
        return getPose2DStraightLocTranslation(Thistag,MetersFromAprilTag);
    }
    public static Pose2d getReverseStraightOutLoc(int TagID,double MetersFromAprilTag)
    {
        AprilTag Thistag = getTagbyID(TagID);
        return getReversePose2DStraightLocTranslation(Thistag,MetersFromAprilTag);
    }
    public static Pose2d getReversePose2DStraightLocTranslation(AprilTag Thistag,double MetersFromAprilTag)
    {
      Pose2d results = getPose2DStraightLocTranslation(Thistag,MetersFromAprilTag);
      return new Pose2d(results.getX(),results.getY(),Rotation2d.fromDegrees(results.getRotation().getDegrees() +180));
    }
    public static Pose2d getOffSet90Loc(int TagID,double MetersFromAprilTag,double offcenter90distMeters,boolean positive)
    {
        AprilTag Thistag = getTagbyID(TagID);
       return getOffSet90Loc(Thistag,MetersFromAprilTag,offcenter90distMeters,positive);
    }
    
    public static Pose2d getOffSet90Loc(AprilTag Thistag,double MetersFromAprilTag,double offcenter90distMeters,boolean positive)
    {
        double offsetangle = positive ? -Math.PI/2 : Math.PI/2;//offset 90 degrees from tag ID. and if positive or not.

        Pose2d StraightLoc = getPose2DStraightLocTranslation(Thistag,MetersFromAprilTag);
        Pose2d offsetLoc = getPose2DOffset90LocTranslation(Thistag,offcenter90distMeters,offsetangle);
        return new Pose2d(StraightLoc.getX() +offsetLoc.getX(), StraightLoc.getY() + offsetLoc.getY(), Thistag.Pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    public static Pose2d getPose2DStraightLocTranslation(AprilTag Thistag,double MetersFromAprilTag)
    {
      double numberwithrobotdepth = MetersFromAprilTag+robotmetersdistToCenter;
        double newX = Thistag.Pose.getX() + ((Thistag.extraOffsetWhenTargeting + numberwithrobotdepth)* Math.cos(Thistag.Pose.getRotation().getRadians()));
        double newY = Thistag.Pose.getY() + ((Thistag.extraOffsetWhenTargeting + numberwithrobotdepth)* Math.sin(Thistag.Pose.getRotation().getRadians()));
        return new Pose2d(newX, newY, Thistag.Pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    public static Pose2d getPose2DOffset90LocTranslation(AprilTag Thistag,double offcenter90distMeters,double offsetangle)
    {

      double offcenterX = ((Thistag.offset90Offset + offcenter90distMeters)* Math.cos(Thistag.Pose.getRotation().getRadians()+offsetangle));
      double offcenterY = ((Thistag.offset90Offset + offcenter90distMeters)* Math.sin(Thistag.Pose.getRotation().getRadians()+offsetangle));
        return new Pose2d(offcenterX, offcenterY, Thistag.Pose.getRotation());
    }


    public static List<AprilTag> tagList = Arrays.asList(
      new AprilTag(1,"Trench Red N Bottom",new Pose2d(Units.inchesToMeters(467.64),Units.inchesToMeters(292.31),Rotation2d.fromDegrees(180)),0,Alliance.Red).WithType(TagType.Trench),
      new AprilTag(2,"Hub Red Bump Bottom",new Pose2d(Units.inchesToMeters(469.11),Units.inchesToMeters(182.6),Rotation2d.fromDegrees(90)),0,Alliance.Red).WithType(TagType.Bump),
      new AprilTag(3,"Hub Red N Left",new Pose2d(Units.inchesToMeters(445.35),Units.inchesToMeters(172.84),Rotation2d.fromDegrees(180)),0,Alliance.Red).WithType(TagType.Hub),
      new AprilTag(4,"Hub Red N Middle",new Pose2d(Units.inchesToMeters(445.35),Units.inchesToMeters(158.84),Rotation2d.fromDegrees(180)),0,Alliance.Red).WithType(TagType.Hub),
      new AprilTag(5,"Hub Red Bump Top",new Pose2d(Units.inchesToMeters(469.11),Units.inchesToMeters(135.09),Rotation2d.fromDegrees(270)),0,Alliance.Red).WithType(TagType.Bump),
      new AprilTag(6,"Trench Red N Top",new Pose2d(Units.inchesToMeters(467.64),Units.inchesToMeters(25.37),Rotation2d.fromDegrees(180)),0,Alliance.Red).WithType(TagType.Trench),
      new AprilTag(7,"Trench Red A Top",new Pose2d(Units.inchesToMeters(470.59),Units.inchesToMeters(25.37),Rotation2d.fromDegrees(0)),0,Alliance.Red).WithType(TagType.Trench),
      new AprilTag(8,"Hub Red A Bump Top",new Pose2d(Units.inchesToMeters(483.11),Units.inchesToMeters(135.09),Rotation2d.fromDegrees(270)),0,Alliance.Red).WithType(TagType.Bump),
      new AprilTag(9,"Hub Red A Left",new Pose2d(Units.inchesToMeters(492.88),Units.inchesToMeters(144.84),Rotation2d.fromDegrees(0)),0,Alliance.Red).WithType(TagType.Hub),
      new AprilTag(10,"Hub Red A Middle",new Pose2d(Units.inchesToMeters(492.88),Units.inchesToMeters(158.84),Rotation2d.fromDegrees(0)),0,Alliance.Red).WithType(TagType.Hub),
      new AprilTag(11,"Hub Red A Bump Bottom",new Pose2d(Units.inchesToMeters(483.11),Units.inchesToMeters(182.6),Rotation2d.fromDegrees(90)),0,Alliance.Red).WithType(TagType.Bump),
      new AprilTag(12,"Trench Red A Bottom",new Pose2d(Units.inchesToMeters(470.59),Units.inchesToMeters(292.31),Rotation2d.fromDegrees(0)),0,Alliance.Red).WithType(TagType.Trench),
      new AprilTag(13,"Outpost Red Middle",new Pose2d(Units.inchesToMeters(650.92),Units.inchesToMeters(291.47),Rotation2d.fromDegrees(180)),0,Alliance.Red).WithType(TagType.Outpost),
      new AprilTag(14,"Outpost Red Right",new Pose2d(Units.inchesToMeters(650.92),Units.inchesToMeters(274.47),Rotation2d.fromDegrees(180)),0,Alliance.Red).WithType(TagType.Outpost),
      new AprilTag(15,"Tower Red Middle",new Pose2d(Units.inchesToMeters(650.9),Units.inchesToMeters(170.22),Rotation2d.fromDegrees(180)),0,Alliance.Red).WithType(TagType.Tower),
      new AprilTag(16,"Tower Red Right",new Pose2d(Units.inchesToMeters(650.9),Units.inchesToMeters(153.22),Rotation2d.fromDegrees(180)),0,Alliance.Red).WithType(TagType.Tower),
      new AprilTag(17,"Trench Blue N Top",new Pose2d(Units.inchesToMeters(783.59),Units.inchesToMeters(25.37),Rotation2d.fromDegrees(0)),0,Alliance.Blue).WithType(TagType.Trench),
      new AprilTag(18,"Hub Blue Bump Top",new Pose2d(Units.inchesToMeters(782.11),Units.inchesToMeters(135.09),Rotation2d.fromDegrees(270)),0,Alliance.Blue).WithType(TagType.Bump),
      new AprilTag(19,"Hub Blue N Left",new Pose2d(Units.inchesToMeters(205.87),Units.inchesToMeters(144.84),Rotation2d.fromDegrees(0)),0,Alliance.Blue).WithType(TagType.Hub),
      new AprilTag(20,"Hub Blue N Middle",new Pose2d(Units.inchesToMeters(205.87),Units.inchesToMeters(158.84),Rotation2d.fromDegrees(0)),0,Alliance.Blue).WithType(TagType.Hub),
      new AprilTag(21,"Hub Blue Bump Bottom",new Pose2d(Units.inchesToMeters(182.11),Units.inchesToMeters(182.6),Rotation2d.fromDegrees(90)),0,Alliance.Blue).WithType(TagType.Bump),
      new AprilTag(22,"Trench Blue N Bottom",new Pose2d(Units.inchesToMeters(183.59),Units.inchesToMeters(292.31),Rotation2d.fromDegrees(0)),0,Alliance.Blue).WithType(TagType.Trench),
      new AprilTag(23,"Trench Blue A Bottom",new Pose2d(Units.inchesToMeters(180.64),Units.inchesToMeters(292.31),Rotation2d.fromDegrees(180)),0,Alliance.Blue).WithType(TagType.Trench),
      new AprilTag(24,"Hub Blue A Bump Bottom",new Pose2d(Units.inchesToMeters(168.11),Units.inchesToMeters(182.6),Rotation2d.fromDegrees(90)),0,Alliance.Blue).WithType(TagType.Bump),
      new AprilTag(25,"Hub Blue A Left",new Pose2d(Units.inchesToMeters(158.34),Units.inchesToMeters(172.84),Rotation2d.fromDegrees(180)),0,Alliance.Blue).WithType(TagType.Hub),
      new AprilTag(26,"Hub Blue A Middle",new Pose2d(Units.inchesToMeters(158.34),Units.inchesToMeters(158.84),Rotation2d.fromDegrees(180)),0,Alliance.Blue).WithType(TagType.Hub),
      new AprilTag(27,"Hub Blue A Bump Top",new Pose2d(Units.inchesToMeters(168.11),Units.inchesToMeters(135.09),Rotation2d.fromDegrees(270)),0,Alliance.Blue).WithType(TagType.Bump),
      new AprilTag(28,"Trench Blue A Top",new Pose2d(Units.inchesToMeters(180.64),Units.inchesToMeters(25.37),Rotation2d.fromDegrees(180)),0,Alliance.Blue).WithType(TagType.Trench),
      new AprilTag(29,"Outpost Blue Middle",new Pose2d(Units.inchesToMeters(0.3),Units.inchesToMeters(26.22),Rotation2d.fromDegrees(0)),0,Alliance.Blue).WithType(TagType.Outpost),
      new AprilTag(30,"Outpost Blue Right",new Pose2d(Units.inchesToMeters(0.3),Units.inchesToMeters(43.22),Rotation2d.fromDegrees(0)),0,Alliance.Blue).WithType(TagType.Outpost),
      new AprilTag(31,"Tower Blue Middle",new Pose2d(Units.inchesToMeters(0.32),Units.inchesToMeters(147.47),Rotation2d.fromDegrees(0)),0,Alliance.Blue).WithType(TagType.Tower),
      new AprilTag(32,"Tower Blue Right",new Pose2d(Units.inchesToMeters(0.32),Units.inchesToMeters(164.47),Rotation2d.fromDegrees(0)),0,Alliance.Blue).WithType(TagType.Tower)
    );

  private int selectTower() {

    return AprilTagManager.getClosestTagofTypeToRobotCenter(this.drivetrain.getState().Pose,TagType.Tower).ID;
  }
  private int selectHub() {

    return AprilTagManager.getClosestTagofTypeToRobotCenter(this.drivetrain.getState().Pose,TagType.Hub).ID;
  }
  private int selectOutpost() {

  return AprilTagManager.getClosestTagofTypeToRobotCenter(this.drivetrain.getState().Pose,TagType.Outpost).ID;

  }
  private int selectBump() {

  return AprilTagManager.getClosestTagofTypeToRobotCenter(this.drivetrain.getState().Pose,TagType.Bump).ID;

  }
  private int selectTrench() {

  return AprilTagManager.getClosestTagofTypeToRobotCenter(this.drivetrain.getState().Pose,TagType.Trench).ID;

  }

public final SelectCommand C_OutpostCommand(){
     return new SelectCommand<>(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(13, new PrintCommand("Red Outpost was selected!")
              .alongWith(new C_Align(AprilTagManager.getReverseStraightOutLoc(13,0.0)))),

              Map.entry(29, new PrintCommand("Blue Outpost was selected!")
              .alongWith(new C_Align(AprilTagManager.getReverseStraightOutLoc(29,0))))
              ),

          ()->{return selectOutpost();});

    }
    public final SelectCommand C_HubCommand(){
     return new SelectCommand<>(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(10, new PrintCommand("Red Hub was selected!")
              .alongWith(new C_Align(AprilTagManager.getReverseStraightOutLoc(10,HubCenterOffset)))),

              Map.entry(26, new PrintCommand("Blue Hub was selected!")
              .alongWith(new C_Align(AprilTagManager.getReverseStraightOutLoc(26,HubCenterOffset))))
              ),

          ()->{return selectHub();});

    }

      public final SelectCommand C_TowerCommand(){
     return new SelectCommand<>(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(16, new PrintCommand("Red Tower was selected!")
              .alongWith(new C_Align(AprilTagManager.getReverseStraightOutLoc(16,TowerOffset)))),

              Map.entry(32, new PrintCommand("Blue Tower was selected!")
              .alongWith(new C_Align(AprilTagManager.getReverseStraightOutLoc(32,TowerOffset))))
              ),

          ()->{return selectTower();});
    }
 
  }