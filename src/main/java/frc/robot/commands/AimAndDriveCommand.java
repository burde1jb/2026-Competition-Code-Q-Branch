package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
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
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.AlphaBots.NT;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Util.DriveInputSmoother;
import frc.robot.Util.GeometryUtil;
import frc.robot.Util.ManualDriveInput;
import frc.robot.commands.C_Align.drivetrainThings;
import frc.robot.generated.TunerConstants;

public class AimAndDriveCommand extends Command {
    
    public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final double kAimingTranslationalSpeedFactor = 0.5;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
         //if we are really far away lets keep pid from going insane.
        public static final double maxYvelocity = 2.5;
        public static final double maxXvelocity = 2.5;
    }
    private static final Angle kAimTolerance = Degrees.of(5);

    private final CommandSwerveDrivetrain swerve;
    private final frc.robot.Util.DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(Driving.kMaxSpeed.times(0.2))    
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);

    public AimAndDriveCommand(
        CommandSwerveDrivetrain swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new frc.robot.Util.DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(CommandSwerveDrivetrain swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }
    public double distanceToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        double dist = hubPosition.minus(robotPosition).getNorm();
        SmartDashboard.putNumber("Distance to Hub", dist);
        return dist;
    }
    // returns the closest pose2d that is the specified distance from the hub on the arc around the hub at the specific distance, facing towards the hub
    private Pose2d getPoseAlongLineToHub(double distanceFromHub) {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Rotation2d directionToHub = getDirectionToHub().rotateBy(Rotation2d.fromDegrees(180));
        final Translation2d targetPosition = hubPosition.plus(new Translation2d(directionToHub.getCos() * distanceFromHub, directionToHub.getSin() * distanceFromHub));
        return new Pose2d(targetPosition, new Rotation2d());
    }
    
    

    private final PIDController AlignXPid = new PIDController(drivetrainThings.k_PoseX_P,drivetrainThings.k_PoseX_I,drivetrainThings.k_PoseX_D);
    private final PIDController AlignYPid = new PIDController(drivetrainThings.k_PoseY_P,drivetrainThings.k_PoseY_I,drivetrainThings.k_PoseY_D);
    private final double wanteddistanceFromHub = Units.inchesToMeters(160); //CHANGE SHOOTING DISTANCE HERE
    private final StructPublisher<Pose2d> drivePose = NT.getStructEntry_Pose2D("AimAndDriveCmd", "TargetPose", new Pose2d());
    private final DoubleEntry distanceToHubPublisher = NT.getDoubleEntry("AimAndDriveCmd", "DistanceToHub", 0);
    @Override
    public void execute() {
        double currentDistanceToHub = distanceToHub();
        distanceToHubPublisher.set(currentDistanceToHub);
        Pose2d CurrentPose = swerve.getState().Pose;
        Pose2d TargetPose = getPoseAlongLineToHub(wanteddistanceFromHub);
        TargetPose = new Pose2d(TargetPose.getTranslation(), getDirectionToHub());
        drivePose.set(TargetPose);
        //get offsets
        //SUBTRACT where we need to go, from where we are. this will give us the translations we need to make 
        //double Xpose_Offset = CurrentPose.getX() - TargetPose.getX();
        //double Ypose_Offset = CurrentPose.getY() - TargetPose.getY();             
     
        //var PoseOffset = new Pose2d(Xpose_Offset, Ypose_Offset, new Rotation2d(0));
        AlignXPid.setSetpoint(TargetPose.getX());
        AlignYPid.setSetpoint(TargetPose.getY());
        double xpose_adjust = AlignXPid.calculate(CurrentPose.getX());//GetXPoseAdjust(XP_buffer, min_xpose_command);
        double Ypose_adjust = AlignYPid.calculate(CurrentPose.getY());//GetYPoseAdjust(YP_buffer, min_Ypose_command );    
        //drive drive drivetrain with PID clamped something to not go crazy or something
        //clamp all results to a max (and negative max) top speed
        Ypose_adjust = MathUtil.clamp(Ypose_adjust, -Driving.maxYvelocity, Driving.maxYvelocity);
        xpose_adjust = MathUtil.clamp(xpose_adjust, -Driving.maxXvelocity, Driving.maxXvelocity);
        var xyMirrorRed = (DriverStation.getAlliance().get() == Alliance.Blue) ? 1.0:-1.0; //our drivetrain auto flips itself when we are on red. so we have to aswell. 


        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        swerve.setControl(
            fieldCentricFacingAngleRequest
             .withVelocityX(xpose_adjust  * xyMirrorRed) // Drive forward with // negative Y (forward)
                .withVelocityY(Ypose_adjust * xyMirrorRed) // Drive left with negative X (left)
                //.withVelocityX(Driving.kMaxSpeed.times(input.forward * Driving.kAimingTranslationalSpeedFactor))
                //.withVelocityY(Driving.kMaxSpeed.times(input.left * Driving.kAimingTranslationalSpeedFactor))
                .withTargetDirection(getDirectionToHub())
        );
    }

    @Override
    public void initialize() {
        AlignXPid.reset();
        AlignYPid.reset();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
