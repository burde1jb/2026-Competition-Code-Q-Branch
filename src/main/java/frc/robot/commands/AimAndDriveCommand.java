package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.RobotContainer;
import frc.robot.drivetrainThings;
import frc.robot.AlphaBots.NT;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AimAndDriveCommand extends Command {
    
    public static class Driving {
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);

    }

    private final CommandSwerveDrivetrain swerve;
    private Pose2d PoseOffset;//This is how far we are from where we want to be. this is CurrentPose minus TargetPose.
    private final PIDController AlignXPid = new PIDController(drivetrainThings.k_PoseX_P,drivetrainThings.k_PoseX_I,drivetrainThings.k_PoseX_D);
    private final PIDController AlignYPid = new PIDController(drivetrainThings.k_PoseY_P,drivetrainThings.k_PoseY_I,drivetrainThings.k_PoseY_D);
    private final double wanteddistanceFromHub = Units.inchesToMeters(142); //CHANGE SHOOTING DISTANCE HERE
    private final StructPublisher<Pose2d> NTTargetPose = NT.getStructEntry_Pose2D("AimAndDriveCmd", "TargetPose", new Pose2d());
    private final BooleanEntry isAligned = NT.getBooleanEntry("AimAndDriveCmd", "isAligned", false);
    private final BooleanEntry alignXOK = NT.getBooleanEntry("AimAndDriveCmd", "alignXOK", false);
    private final BooleanEntry alignYOK = NT.getBooleanEntry("AimAndDriveCmd", "alignYOK", false);
    private final BooleanEntry alignZOK = NT.getBooleanEntry("AimAndDriveCmd", "alignZOK", false);

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        //.withDeadband(Driving.kMaxSpeed.times(0.2))    //no deadband we are using PID to correct our position so we dont want to just ignore small inputs, uncomment if robot is twitchy when it is close to the target, BUT you should just lower P gains to reduce twitching. 
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance) //everything here is in Blue alliance perspective. it points at locations on the field on its own so drivers perspective doesnt matter here. 
        .withHeadingPID(5, 0, 0);

    public AimAndDriveCommand( CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        return hubDirectionInBlueAlliancePerspective;
    }
    // returns the closest pose2d that is the specified distance from the hub on the arc around the hub at the specific distance, facing towards the hub
    private Pose2d getPoseAlongLineToHub(double distanceFromHub) {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Rotation2d directionToHub = getDirectionToHub(); //direction to hub in blue alliance perspective
        final Rotation2d RayCastFromHub = directionToHub.rotateBy(Rotation2d.fromDegrees(180)); //for target position we need to raycast from the hub outwards in the opposite direction of the hub to robot vector, so we rotate it by 180 degrees to get that opposite direction. this is still in blue alliance perspective which is what we want.
        final Translation2d targetPosition = hubPosition.plus(new Translation2d(RayCastFromHub.getCos() * distanceFromHub, RayCastFromHub.getSin() * distanceFromHub));
        return new Pose2d(targetPosition, directionToHub);
    }

    @Override
    public void execute() {
        //Step 1 get where we are and where we are going
        Pose2d CurrentPose = swerve.getState().Pose;
        Pose2d TargetPose = getPoseAlongLineToHub(wanteddistanceFromHub);
        NTTargetPose.set(TargetPose);
        
        //Step 2 get the difference between where we are and where we need to be. this will be our error for checking if we are close enough to the target.
        double Xpose_Offset = CurrentPose.getX() - TargetPose.getX();
        double Ypose_Offset = CurrentPose.getY() - TargetPose.getY();             
        Rotation2d RZ_Offset = CurrentPose.getRotation().minus(TargetPose.getRotation());
        PoseOffset = new Pose2d(Xpose_Offset, Ypose_Offset, RZ_Offset);

        //Step 3 tell PID where we want to be and where we are, and let it calculate the adjustment we need to make to get there.
        AlignXPid.setSetpoint(TargetPose.getX());
        AlignYPid.setSetpoint(TargetPose.getY());
        double xpose_adjust = AlignXPid.calculate(CurrentPose.getX());
        double Ypose_adjust = AlignYPid.calculate(CurrentPose.getY());   

        //Step 4 clamp all results to a max (and negative max) top speed
        Ypose_adjust = MathUtil.clamp(Ypose_adjust, -drivetrainThings.maxYvelocity, drivetrainThings.maxYvelocity);
        xpose_adjust = MathUtil.clamp(xpose_adjust, -drivetrainThings.maxXvelocity, drivetrainThings.maxXvelocity);
        
        //Step 5 send the request to the drivetrain
        swerve.setControl(
            fieldCentricFacingAngleRequest
             .withVelocityX(xpose_adjust) 
                .withVelocityY(Ypose_adjust)
                .withTargetDirection(TargetPose.getRotation())
        );
    }

    @Override
    public void initialize() {
        AlignXPid.reset();
        AlignYPid.reset();
        RobotContainer.aligned = false;
    }
    @Override
    public boolean isFinished() {
        if(PoseOffset == null){System.err.println("No pose OFFSET! Broken CODE?"); return false;}
        boolean Xok = IsXInTarget();
        boolean Yok = IsYInTarget();
        boolean Zok = isRotInTarget();
        boolean isatSetpos = Xok && Yok; //  && Zok; //uncomment to add rotation into the required parameters if shooting while not facing the hub is a problem. for now we are just going to require x and y to be in position, and not care about rotation as much since we can still shoot if we are a little off in rotation, but being too far away in x or y is a bigger problem.
    
        if(isatSetpos){RobotContainer.aligned = true;}else {RobotContainer.aligned = false;}
        isAligned.set(isatSetpos);
        alignXOK.set(Xok);
        alignYOK.set(Yok);
        alignZOK.set(Zok);
        return false;
    }
     private boolean IsXInTarget() {
        return Math.abs(PoseOffset.getX()) < drivetrainThings.minXposeErrorMetersToCorrect;
      }
    
      private boolean IsYInTarget() {
        return Math.abs(PoseOffset.getY()) < drivetrainThings.minYposeErrorMetersToCorrect;
      }
    
      private boolean isRotInTarget() {
        return Math.abs(PoseOffset.getRotation().getDegrees()) < drivetrainThings.minRZErrorToCorrect;
      }
}
