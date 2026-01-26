package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Reggie;


/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.3, 12/11/2025
 */
@Configurable
@Autonomous(name = "BLUE_Close_LM3_Tournament", group = "LM3/TOUR Reggie")
public class ReggieBlueCloseAuto_v2 extends OpMode {
    //private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
    //Pedro Pathing Variables
    private Follower follower;
    private Reggie Miller;
    public static Pose startingPose;
    public int motif;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int GShoot = 20;
    private int PShoot = 65;
    double sortPositionMiddle = 0.45;
    double sortPositionRight = 0.1;
    double sortPositionMidRight = 0.3;
    double sortPositionLeft = 0.9;
    double sortPositionMidLeft = 0.75;
    private enum Rows{
        TOP_ROW,
        MIDDLE_ROW,
        BOTTOM_ROW
    }
    Rows currentRow = Rows.TOP_ROW;

    /**
     *
     * Configurable Variables Start Here
     *
     */

    public static double yTopRowPurple = 90;
    public static double yTopRowGreen = 84;

    public static double yMidRowPurple = 64;
    public static double yMidRowPurpleGPP = 67;
    public static double yMidRowGreenGPP = 63;
    public static double yMidRowGreen = 60;
    public static double yBotRowPurple = 26;
    public static double yBotRowGreen = 34;
    public static double pickingAngle = 180;
    public static double shootingAngleFirst = 137;
    public static double shootingAngleSecond = 135;


    /**
     *
     * Paths Start here
     *
     */
    private Path Start;
    private PathChain ReadPath, ScorePath, aimingFirstBallTopRowPath, pickFirstBallTopRowPath, pickSecondBallTopRowPath,
                      pickThirdBallTopRowPath, grabPickup2,forward, scoreFromTopRowPath, scoreFromMiddleRowPath,
                      aimingFirstBallMiddleRowPath,aimingFirstBallMiddleRowPathGPP, pickFirstBallMiddleRowPath, pickFirstBallMiddleRowPathGPP,
                      pickSecondBallMiddleRowPath,pickSecondBallMiddleRowPathGPP, pickThirdBallMiddleRowPathGPP, scoreFromMiddleRowPathGPP,
                      pickThirdBallMiddleRowPath, leavePath, pickThirdBallMiddleRowPathPPG, scoreFromMiddleRowPathPPG,
                      pickSecondBallMiddleRowPathPPG;
    private final Pose startPose = new Pose(16, 113, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose readPose = new Pose(51, 95, Math.toRadians(80)); // Reading Obelisk Pose of our robot. It is facing the obelisk at 127 degree angle.
    private final Pose scorePose = new Pose(48, 90, Math.toRadians(shootingAngleFirst)); // Scoring Pose of our robot. It is facing the goal at 47 degree aScoring Pose ofngle.
    //    //private final Pose scorePoseSecond = new Pose(92, 93, Math.toRadians(40)); //  Working Pose
    private final Pose scorePoseSecond = new Pose(50, 95, Math.toRadians(shootingAngleSecond));
    private final Pose scorePoseSecondControl = new Pose(53, 80, Math.toRadians(113)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose scorePoseThirdControl = new Pose(54, 50, Math.toRadians(113)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose scorePoseThird = new Pose(53, 92, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose aimingFirstBallTopRowPose = new Pose(47, yTopRowPurple, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose aimingFirstBallTopRowControlPose = new Pose(60, yTopRowPurple+10, Math.toRadians(shootingAngleFirst)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickFirstBallTopRowPose = new Pose(35, yTopRowPurple, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickSecondBallTopRowPose = new Pose(31, yTopRowPurple, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallTopRowPose = new Pose(18, yTopRowGreen, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallTopRowControlPose = new Pose(40, yTopRowGreen, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose aimingFirstBallMiddleRowPose = new Pose(43, yMidRowPurple, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose aimingFirstBallMiddleRowControlPose = new Pose(50, yMidRowPurple+5, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickFirstBallMiddleRowPose = new Pose(33, yMidRowPurple, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.

    //GPP
    private final Pose aimingFirstBallMiddleRowPoseGPP = new Pose(43, yMidRowPurpleGPP, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose aimingFirstBallMiddleRowControlPoseGPP = new Pose(50, yMidRowPurpleGPP, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickFirstBallMiddleRowPoseGPP = new Pose(33, yMidRowPurpleGPP, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.

    private final Pose pickSecondBallMiddleRowPoseGPP = new Pose(23, yMidRowGreenGPP, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickSecondBallMiddleRowControlPoseGPP = new Pose(30, yMidRowGreenGPP, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallMiddleRowPoseGPP = new Pose(13, yMidRowGreenGPP-3, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallMiddleRowControlPoseGPP = new Pose(20, yMidRowGreenGPP-2, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.

    private final Pose pickSecondBallMiddleRowPose = new Pose(23, yMidRowGreen, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickSecondBallMiddleRowControlPose = new Pose(30, yMidRowGreen, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallMiddleRowPose = new Pose(13, yMidRowGreen-3, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallMiddleRowControlPose = new Pose(20, yMidRowGreen-2, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.

    //PPG Mid Row Paths
    private final Pose pickSecondBallMiddleRowPosePPG = new Pose(31, yMidRowPurple-2, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickSecondBallMiddleRowControlPosePPG = new Pose(35, yMidRowPurple-2, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallMiddleRowPosePPG = new Pose(18, yMidRowPurple+2, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallMiddleRowControlPosePPG = new Pose(20, yMidRowPurple+2, Math.toRadians(pickingAngle)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.

//    private final Pose pickThirdBallMiddleRowPosePPG = new Pose(115, yMidRowPurple-7, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose leavePose = new Pose(53, 75, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(82, 80, Math.toRadians(50)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1 = new Pose(90, 74, Math.toRadians(360)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2 = new Pose(115, 74, Math.toRadians(360)); // Highest (First Set) of Artifacts from the Spike Mark.






    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    //@Override


    @Override
    public void init() {
        //Pedro Pathing
//        Constants.setConstants(FConstants.class, LConstants.class);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        Miller = new Reggie();
        Miller.init(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        Miller.setStoppers(true, true);
        //motif = order(Miller.huskyLens);

        // playTime.reset();
    }

    public void buildPaths() {
        /* Drive to read the obelisk */
        ReadPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, readPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),readPose.getHeading())
                .build();

        /* Turn to score position */
        ScorePath = follower.pathBuilder()
                .addPath(new BezierLine(readPose, scorePose))
                .setLinearHeadingInterpolation(readPose.getHeading(),scorePose.getHeading())
                .build();

        /* Aim to get the first ball from the top row */
        aimingFirstBallTopRowPath = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,aimingFirstBallTopRowControlPose, aimingFirstBallTopRowPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),aimingFirstBallTopRowPose.getHeading())
                .build();

        /* Drive to pick the first ball then stop to sort */
        pickFirstBallTopRowPath = follower.pathBuilder()
                .addPath(new BezierLine(aimingFirstBallTopRowPose, pickFirstBallTopRowPose))
                .setLinearHeadingInterpolation(aimingFirstBallTopRowPose.getHeading(),pickFirstBallTopRowPose.getHeading())
                .build();

        /* Drive to pick the Second ball from top row */
        pickSecondBallTopRowPath = follower.pathBuilder()
                .addPath(new BezierLine(pickFirstBallTopRowPose, pickSecondBallTopRowPose))
                .setLinearHeadingInterpolation(pickFirstBallTopRowPose.getHeading(),pickSecondBallTopRowPose.getHeading())
                .build();

        /* Drive to pick the Third ball from top row */
        pickThirdBallTopRowPath = follower.pathBuilder()
                .addPath(new BezierCurve(pickSecondBallTopRowPose, pickThirdBallTopRowControlPose, pickThirdBallTopRowPose))
                .setLinearHeadingInterpolation(pickSecondBallTopRowPose.getHeading(),pickThirdBallTopRowPose.getHeading())
                .build();

        /* Drive to Score from the Third ball from top row */
        scoreFromTopRowPath = follower.pathBuilder()
                .addPath(new BezierCurve(pickThirdBallTopRowPose, scorePoseSecondControl ,scorePoseSecond))
                .setLinearHeadingInterpolation(pickThirdBallTopRowPose.getHeading(),scorePoseSecond.getHeading())
                .build();

        /* Drive to Score from the Third ball from Middle row */
        scoreFromMiddleRowPath = follower.pathBuilder()
                .addPath(new BezierCurve(pickThirdBallMiddleRowPose, scorePoseThirdControl, scorePoseThird))
                .setLinearHeadingInterpolation(pickThirdBallMiddleRowPose.getHeading(),scorePoseThird.getHeading())
                .build();

        /* Drive to Score from the Third ball from Middle row PPG */
        scoreFromMiddleRowPathGPP = follower.pathBuilder()
                .addPath(new BezierCurve(pickThirdBallMiddleRowPoseGPP, scorePoseThirdControl, scorePoseThird))
                .setLinearHeadingInterpolation(pickThirdBallMiddleRowPoseGPP.getHeading(),scorePoseThird.getHeading())
                .build();

        /* Drive to Score from the Third ball from Middle row PPG */
        scoreFromMiddleRowPathPPG = follower.pathBuilder()
                .addPath(new BezierCurve(pickThirdBallMiddleRowPosePPG, scorePoseThirdControl, scorePoseThird))
                .setLinearHeadingInterpolation(pickThirdBallMiddleRowPosePPG.getHeading(),scorePoseThird.getHeading())
                .build();

        /* Aim to get the first ball from the top row */
        aimingFirstBallMiddleRowPath = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoseSecond,aimingFirstBallMiddleRowControlPose, aimingFirstBallMiddleRowPose))
                .setLinearHeadingInterpolation(scorePoseSecond.getHeading(),aimingFirstBallMiddleRowPose.getHeading())
                .build();


        /* Aim to get the first ball from the top row */
        aimingFirstBallMiddleRowPathGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoseSecond,aimingFirstBallMiddleRowControlPoseGPP, aimingFirstBallMiddleRowPoseGPP))
                .setLinearHeadingInterpolation(scorePoseSecond.getHeading(),aimingFirstBallMiddleRowPoseGPP.getHeading())
                .build();



        /* Drive to pick the first ball then stop to sort */
        pickFirstBallMiddleRowPath = follower.pathBuilder()
                .addPath(new BezierLine(aimingFirstBallMiddleRowPose, pickFirstBallMiddleRowPose))
                .setLinearHeadingInterpolation(aimingFirstBallMiddleRowPose.getHeading(),pickFirstBallMiddleRowPose.getHeading())
                .build();



        /* Drive to pick the first ball then stop to sort */
        pickFirstBallMiddleRowPathGPP = follower.pathBuilder()
                .addPath(new BezierLine(aimingFirstBallMiddleRowPoseGPP, pickFirstBallMiddleRowPoseGPP))
                .setLinearHeadingInterpolation(aimingFirstBallMiddleRowPoseGPP.getHeading(),pickFirstBallMiddleRowPoseGPP.getHeading())
                .build();

        /* Drive to pick the Second ball from top row */
        pickSecondBallMiddleRowPath = follower.pathBuilder()
                .addPath(new BezierCurve(pickFirstBallMiddleRowPose,pickSecondBallMiddleRowControlPose, pickSecondBallMiddleRowPose))
                .setLinearHeadingInterpolation(pickFirstBallMiddleRowPose.getHeading(),pickSecondBallMiddleRowPose.getHeading())
                .build();

        /* Drive to pick the Second ball from top row */
        pickSecondBallMiddleRowPathGPP = follower.pathBuilder()
                .addPath(new BezierCurve(pickFirstBallMiddleRowPoseGPP,pickSecondBallMiddleRowControlPoseGPP, pickSecondBallMiddleRowPoseGPP))
                .setLinearHeadingInterpolation(pickFirstBallMiddleRowPoseGPP.getHeading(),pickSecondBallMiddleRowPoseGPP.getHeading())
                .build();

        /* Drive to pick the Second ball from Middle row PPG */
         pickSecondBallMiddleRowPathPPG = follower.pathBuilder()
                .addPath(new BezierCurve(pickFirstBallMiddleRowPose, pickSecondBallMiddleRowControlPosePPG, pickSecondBallMiddleRowPosePPG))
                .setLinearHeadingInterpolation(pickFirstBallMiddleRowPose.getHeading(), pickSecondBallMiddleRowPosePPG.getHeading())
                .build();

        /* Drive to pick the Third ball from top row */
        pickThirdBallMiddleRowPath = follower.pathBuilder()
                .addPath(new BezierLine(pickSecondBallMiddleRowPose, pickThirdBallMiddleRowPose))
                .setLinearHeadingInterpolation(pickSecondBallMiddleRowPose.getHeading(),pickThirdBallMiddleRowPose.getHeading())
                .build();

        /* Drive to pick the Third ball from top row */
        pickThirdBallMiddleRowPathPPG = follower.pathBuilder()
                .addPath(new BezierCurve(pickSecondBallMiddleRowPosePPG,pickThirdBallMiddleRowControlPosePPG, pickThirdBallMiddleRowPosePPG))
                .setLinearHeadingInterpolation(pickSecondBallMiddleRowPosePPG.getHeading(),pickThirdBallMiddleRowPosePPG.getHeading())
                .build();

        /* Drive to pick the Third ball from top row */
        pickThirdBallMiddleRowPathGPP = follower.pathBuilder()
                .addPath(new BezierCurve(pickSecondBallMiddleRowPoseGPP,pickThirdBallMiddleRowControlPoseGPP, pickThirdBallMiddleRowPoseGPP))
                .setLinearHeadingInterpolation(pickSecondBallMiddleRowPoseGPP.getHeading(),pickThirdBallMiddleRowPoseGPP.getHeading())
                .build();

        leavePath = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseThird,leavePose))
                .setLinearHeadingInterpolation(scorePoseThird.getHeading(), leavePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose,pickup1))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1.getHeading())
                .build();

        forward = follower.pathBuilder()
                .addPath(new BezierLine(pickup1,pickup2))
                .setConstantHeadingInterpolation(pickup1.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(ReadPath,0.9, true);
                setPathState(11);
                Miller.setShooterVelocity(Reggie.SIDES.RIGHT);

                //Miller.shootClose();
                break;
            case 11:
                motif = order(Miller.huskyLens);
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.75 && (motif==1 || motif==2 || motif==3)){
                    setPathState(1);
                }
                if(pathTimer.getElapsedTimeSeconds() >= 4){
                    Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                    follower.followPath(ScorePath,0.85,true);
                    setPathState(100);
                }
                break;
            case 1:
                follower.setMaxPower(.85);

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(ScorePath,0.85, true);
//                    Miller.setShooterPower(100);
                    //Miller.setShooterVelocity(Miller.TARGET_VELOCITY);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >=1.25 ){
                    if ( motif == 1) {
                        Miller.TARGET_MIN_VELOCITY_RIGHT = 1050;
                        Miller.TARGET_VELOCITY_RIGHT = 1150;
                        Miller.TARGET_VELOCITY_LEFT = 1200;
                        Miller.TARGET_MIN_VELOCITY_LEFT = 1100;
                        Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                        setPathState(100);
                    }else if ( motif == 2) {
                        Miller.TARGET_MIN_VELOCITY_RIGHT = 1050;
                        Miller.TARGET_VELOCITY_RIGHT = 1150;
                        Miller.TARGET_VELOCITY_LEFT = 1175;
                        Miller.TARGET_MIN_VELOCITY_LEFT = 1075;
                        Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                        setPathState(200);
                    }else if ( motif == 3) {
                        Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                        Miller.TARGET_MIN_VELOCITY_RIGHT = 1125;
                        Miller.TARGET_VELOCITY_RIGHT = 1175;
                        Miller.TARGET_VELOCITY_LEFT = 1175;
                        Miller.TARGET_MIN_VELOCITY_LEFT = 1075;

                        setPathState(300);
                    }else{
                        setPathState(100);
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() >= 15) {
                    break;
                }
                break;
            case 100:
                //PPG
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT) && !follower.isBusy()){
                    Miller.setStoppers(true,false);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    Miller.indexerPower(85, Miller.leftIndexerServo);

                    setPathState(101);
                }
                break;
            case 101:
                /**
                 * Shooting the Purple Balls
                 */
                if(pathTimer.getElapsedTimeSeconds()>0.65){
                    Miller.setIntakePower(75);
                    Miller.setSorterServoPosition(sortPositionLeft);
                }
                if(pathTimer.getElapsedTimeSeconds() >= 1.25){
                    Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                    setPathState(102);
                }
                break;
            //Former 32 State
            case 102:
                /**
                 * Shooting Green Artifact
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT) && pathTimer.getElapsedTimeSeconds()>0.35){
                    setPathState(103);
                    Miller.setStoppers(false,false);
                    Miller.setIntakePower(0);
                    Miller.setSorterServoPosition(sortPositionRight);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    Miller.indexerPower(60, Miller.rightIndexerServo);
                }
                break;
            case 103:
                if (pathTimer.getElapsedTimeSeconds() >= 0.75){
                        switch (currentRow){
                            case TOP_ROW:
                                follower.followPath(aimingFirstBallTopRowPath, 0.85,true);
                                Miller.setSorterServoPosition(sortPositionMiddle);
                                Miller.setStoppers(true, true);
                                setPathState(104);
                                break;
                            case MIDDLE_ROW:
                                follower.followPath(aimingFirstBallMiddleRowPath, 0.85,true);
                                Miller.setSorterServoPosition(sortPositionMiddle);
                                Miller.setIntakePower(75);
                                Miller.setStoppers(true, true);

                                setPathState(111);
                                break;
                            case BOTTOM_ROW:
                                setPathState(120);
                                break;
                        }



                }
                break;
            case 104:
                /**
                 * Picking up the First Purple Ball from Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickFirstBallTopRowPath, 0.3,true);
                    Miller.setIntakePower(75);
                    Miller.setSorterServoPosition(sortPositionRight);
                    Miller.setIntakePower(85);
                    Miller.setStoppers(true, true);
                    setPathState(105);
                }

                break;
            case 105:
                /**
                 * Pause a little to retrieve the first Purple Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.25){
                    setPathState(106);
                }
                break;
            case 106:
                /**
                 * Advance to the second Purple Ball from Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickSecondBallTopRowPath, 0.35,true);
                    Miller.setSorterServoPosition(sortPositionMidRight);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-5, Miller.leftIndexerServo);
                    //Miller.setStoppers(true, true);
                    setPathState(107);
                }
                break;
            case 107:
                /**
                 * Retrieve the Green Ball From Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.35){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    follower.followPath(pickThirdBallTopRowPath, 0.35,true);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-10, Miller.leftIndexerServo);
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                    setPathState(108);
                }
                break;
            case 108:
                /**
                 * Pause to retrieve the Green Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.750){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    //Miller.setStoppers(false, false);
                    setPathState(109);
                }
                break;
            case 109:
                /**
                 * Drive to Score Position after retrieving Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(scoreFromTopRowPath, 0.8,true);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    //Miller.stopEverything();

                    setPathState(110);
                }
                break;
            case 110:
                /**
                 * Wait for shooter to speed up then shoot
                 */
                if (!follower.isBusy() && Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    //Miller.stopEverything();
                    //Miller.setStoppers(false,false);
                    Miller.setIntakePower(75);
                    Miller.indexerPower(85, Miller.leftIndexerServo);
                    //Miller.setSorterServoPosition(Miller.sortPositionRight);
                    currentRow = Rows.MIDDLE_ROW;
                    setPathState(100);
                }
                break;
            case 111:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.25){
                    //Miller.stopEverything();
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                   // follower.followPath(aimingFirstBallMiddleRowPath, 0.8,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(75);
                    Miller.setStoppers(true, true);
                    setPathState(112);
                }
                break;
            case 112:
                /**
                 * Picking up the First Purple Ball from Middle Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickFirstBallMiddleRowPath, 0.3,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(85);
                    //Miller.setStoppers(true, true);
                    setPathState(113);
                }
                break;
            case 113:
                /**
                 * Pause a little to retrieve the first Purple Ball from Middle Row
                 */
                if(pathTimer.getElapsedTimeSeconds()>0.45){
                    Miller.setSorterServoPosition(sortPositionMidRight);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.55){
                    setPathState(114);
                    //follower.setMaxPower(.5);
                }
                break;
            case 114:
                /**
                 * Advance to the Green Ball from Middle Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickSecondBallMiddleRowPathPPG, 0.4,true);
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-5, Miller.leftIndexerServo);
                    //Miller.setStoppers(true, true);
                    setPathState(115);
                }
                break;
            case 115:
                /**
                 * Pause to retrieve the Green Ball from Middle Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.15){
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    setPathState(116);
                }
                break;
            case 116:
                /**
                 * Retrieve the Second Purple Ball From Middle Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.5){
                    Miller.setSorterServoPosition(sortPositionMidRight);
                    follower.followPath(pickThirdBallMiddleRowPathPPG, 0.5,true);
                    Miller.setIntakePower(95);
                    //Miller.indexerPower(-10, Miller.rightIndexerServo);
                    setPathState(117);
                }
                break;
            case 117:
                /**
                 * Pause to retrieve the Second Purple Ball from Middle Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.35){
                    // Miller.setSorterServoPosition(sortPositionMidLeft);
                    //Miller.setStoppers(false, false);
                    setPathState(118);
                }
                break;
            case 118:
                /**
                 * Drive to Score Position after retrieving Middle Row
                 */
                follower.setMaxPower(0.9);
                if (!follower.isBusy()){
                    follower.followPath(scoreFromMiddleRowPathPPG, 0.9,true);
                    //Miller.stopEverything();
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                    setPathState(119);
                }
                break;
            case 119:
                /**
                 * Wait for shooter to speed up then shoot
                 */
                if (!follower.isBusy() && Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    //Miller.stopEverything();
                    Miller.setStoppers(false,false);
                    Miller.setIntakePower(75);
                    Miller.indexerPower(85, Miller.rightIndexerServo);
                    //Miller.setSorterServoPosition(Miller.sortPositionRight);
                    currentRow = Rows.BOTTOM_ROW;
                    setPathState(100);
                }
                break;
            case 120:
                /**
                 * Shooting Green
                 */
                if(pathTimer.getElapsedTimeSeconds()>0.15){
                    follower.followPath(leavePath, 0.8,true);
                    Miller.stopEverything();
                    setPathState(-100);
                }
                break;

            case 200:
                /**
                 * Green - Purple - Purple
                 * Waiting for the shooter to speed up to shoot Green Artifact
                 */
                //GPP
                //Miller.setShooterPower(GShoot);
                if(Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT)){
                    Miller.setStoppers(false,true);
                    Miller.indexerPower(85, Miller.rightIndexerServo);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    Miller.setSorterServoPosition(sortPositionRight);
                    setPathState(201);
                }
                break;
            case 201:
                /**
                 * Shooting the Green Ball
                 */
                if(pathTimer.getElapsedTimeSeconds() >= 0.750){
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    setPathState(202);
                }
                break;
            case 202:
                /**
                 * Speeding up for the First Purple Artifact
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT) && pathTimer.getElapsedTimeSeconds() >= 0.75){

                    Miller.setStoppers(false,false);
                    Miller.setIntakePower(75);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.indexerPower(85, Miller.leftIndexerServo);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    setPathState(203);
                }
                break;
             case 203:
                 /**
                  * Shooting First Purple Artifact
                  */

                if(pathTimer.getElapsedTimeSeconds() >= 0.45){
                    //Miller.setIntakePower(0);
                    //Miller.setSorterServoPosition(sortPositionLeft);
                    //Miller.indexerPower(0, Miller.leftIndexerServo);
                    setPathState(204);
                }
                break;
            case 204:
                /**
                 * Speeding up for the Second Purple Artifact
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT) ){
                    Miller.setIntakePower(85);
                    Miller.setSorterServoPosition(Miller.sortPositionLeft);
                    Miller.indexerPower(85, Miller.leftIndexerServo);
                    setPathState(205);
                }
                break;
            case 205:
                /**
                 * Shooting Second Purple Artifact
                 */

                if(pathTimer.getElapsedTimeSeconds() >= 0.75){
                    switch (currentRow){
                        case TOP_ROW:
                            Miller.setSorterServoPosition(Miller.sortPositionLeft);
                            setPathState(206);
                            break;
                        case MIDDLE_ROW:
                            Miller.indexerPower(85, Miller.leftIndexerServo);
                            Miller.setSorterServoPosition(Miller.sortPositionLeft);
                            setPathState(216);
                            break;
                        case BOTTOM_ROW:

                            break;
                    }


                }
                break;
            case 206:
                /**
                 * Aiming for the Top Row of Artifacts
                 */
//                if(pathTimer.getElapsedTimeSeconds() > 0.25){
//                    Miller.stopEverything();
//                }
                if (pathTimer.getElapsedTimeSeconds() >= 0.30){
                    follower.followPath(aimingFirstBallTopRowPath, 0.85,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(75);
                    Miller.setStoppers(true, true);
                    setPathState(208);
                }
                break;
            case 207:
                /**
                 * Waiting for aimingFirstBallTopRowPath to finish
                 */
                if (!follower.isBusy()){
                    setPathState(208);
                }
                break;
            case 208:
                /**
                 * Picking up the First Purple Ball from Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickFirstBallTopRowPath, 0.3,true);
                    Miller.setSorterServoPosition(sortPositionRight);
                    Miller.setIntakePower(85);
                    //Miller.setStoppers(true, true);
                    setPathState(209);
                }
                break;
            case 209:
                /**
                 * Pause a little to retrieve the first Purple Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.35){
                    setPathState(210);
                }
                break;
            case 210:
                /**
                 * Advance to the second Purple Ball from Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickSecondBallTopRowPath, 0.3,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-5, Miller.leftIndexerServo);
                    Miller.setStoppers(true, true);
                    setPathState(211);
                }
                break;
            case 211:
                /**
                 * Pause to retrieve the second Purple Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.45){
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    setPathState(212);
                }
                break;
            case 212:
                /**
                 * Retrieve the Green Ball From Top Row
                 */
                if (!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    follower.followPath(pickThirdBallTopRowPath, 0.35,true);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-10, Miller.leftIndexerServo);
                    Miller.setShooterVelocity(Reggie.SIDES.RIGHT);

                    setPathState(213);
                }
                break;
            case 213:
                /**
                 * Pause to retrieve the Green Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1.0){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    //Miller.setStoppers(false, false);
                    setPathState(214);
                }
                break;
            case 214:
                /**
                 * Drive to Score Position after retrieving Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(scoreFromTopRowPath, 0.8,true);
                    //Miller.stopEverything();

                    setPathState(215);
                }
                break;
            case 215:
                /**
                 * Wait for shooter to speed up then shoot
                 */
                if (!follower.isBusy() && Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT)){
                   //Miller.stopEverything();
                    //Miller.setStoppers(false,false);
                    Miller.setIntakePower(75);
                    Miller.indexerPower(85, Miller.rightIndexerServo);
                    //Miller.setSorterServoPosition(Miller.sortPositionRight);
                    currentRow = Rows.MIDDLE_ROW;
                    setPathState(200);
                }
                break;
            case 216:
                if (pathTimer.getElapsedTimeSeconds() >= 0.80){
                    //Miller.stopEverything();
                    follower.followPath(aimingFirstBallMiddleRowPathGPP, 0.8,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(75);
                    Miller.setStoppers(true, true);
                    setPathState(218);
                }
                break;
            case 217:
                /**
                 * Waiting for aimingFirstBallMiddleRowPath to finish
                 */
                if (!follower.isBusy()){
                    setPathState(218);
                }
                break;
            case 218:
                /**
                 * Picking up the First Purple Ball from Middle Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickFirstBallMiddleRowPathGPP, 0.3,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(85);
                    //Miller.setStoppers(true, true);
                    setPathState(219);
                }
                break;
            case 219:
                /**
                 * Pause a little to retrieve the first Purple Ball from Middle Row
                 */
                if(pathTimer.getElapsedTimeSeconds()>0.45){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.55){
                    setPathState(220);
                    //follower.setMaxPower(.5);
                }
                break;
            case 220:
                /**
                 * Advance to the Green Ball from Middle Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickSecondBallMiddleRowPath, 0.4,true);
                    //Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-5, Miller.leftIndexerServo);
                    //Miller.setStoppers(true, true);
                    setPathState(221);
                }
                break;
            case 221:
                /**
                 * Pause to retrieve the second Green Ball from Middle Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.15){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    setPathState(222);
                }
                break;
            case 222:
                /**
                 * Retrieve the Second Purple Ball From Middle Row
                 */
                if (!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    follower.followPath(pickThirdBallMiddleRowPath, 0.45,true);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-10, Miller.rightIndexerServo);
                    setPathState(223);
                }
                break;
            case 223:
                /**
                 * Pause to retrieve the Second Purple Ball from Middle Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.35){
                   // Miller.setSorterServoPosition(sortPositionMidLeft);
                    //Miller.setStoppers(false, false);
                    setPathState(224);
                }
                break;
            case 224:
                /**
                 * Drive to Score Position after retrieving Middle Row
                 */
                follower.setMaxPower(0.9);
                if (!follower.isBusy()){
                    follower.followPath(scoreFromMiddleRowPath, 0.9,true);
                    //Miller.stopEverything();
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                    setPathState(225);
                }
                break;
            case 225:
                /**
                 * Wait for shooter to speed up then shoot
                 */
                if (!follower.isBusy() && Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    //Miller.stopEverything();
                    Miller.setStoppers(false,false);
                    Miller.setIntakePower(75);
                    Miller.indexerPower(85, Miller.rightIndexerServo);
                    //Miller.setSorterServoPosition(Miller.sortPositionRight);
                    currentRow = Rows.TOP_ROW;
                    setPathState(226);
                }
                break;
            case 226:
                /**
                 * Shooting Green from right shooter
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT) && pathTimer.getElapsedTimeSeconds()>0.55){
                    Miller.setStoppers(true,false);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    Miller.indexerPower(80, Miller.leftIndexerServo);
                    Miller.setSorterServoPosition(sortPositionRight);
                    setPathState(227);
                }
                break;
            case 227:
                /**
                 * Shooting Purple from right shooter
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT) && pathTimer.getElapsedTimeSeconds()>0.55){
                    Miller.setStoppers(false,false);
                    Miller.indexerPower(85, Miller.rightIndexerServo);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    Miller.setSorterServoPosition(sortPositionRight);
                    setPathState(229);
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                }
                break;
            case 228:
                /**
                 * Shooting Purple from Left shooter
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT) && pathTimer.getElapsedTimeSeconds()>0.75){
                    Miller.setStoppers(false,false);
                    Miller.indexerPower(90, Miller.leftIndexerServo);
                    Miller.indexerPower(85, Miller.rightIndexerServo);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    setPathState(229);
                }
                break;
            case 229:
                /**
                 * Shooting Purple from Left shooter
                 */
                if(pathTimer.getElapsedTimeSeconds()>0.25){
                    follower.followPath(leavePath, 0.8,true);
                    //Miller.stopEverything();
                    setPathState(230);
                }
                break;
            case 230:
                /**
                 * Shooting Purple from Left shooter
                 */
                if(!follower.isBusy()){
                    Reggie.poseFromAuto = follower.getPose();
                    Reggie.poseFromAuto = scorePose;
                    requestOpModeStop();
                }
                break;
            case 300:
                //PGP
                //Waiting for shooter to speed up
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    Miller.setStoppers(true, false);
                    Miller.indexerPower(85, Miller.leftIndexerServo);
                    setPathState(301);
                }
                break;
            case 301:
                //SHooting first purple
                Miller.setSorterServoPosition(sortPositionMiddle);
                if (pathTimer.getElapsedTimeSeconds() >= 0.75) {
                    Miller.setStoppers(true, true);
                    Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                    setPathState(302);
                }
                break;
            case 302:
                //Waiting for speeding up for the Green Artifact
                if(Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT) && pathTimer.getElapsedTimeSeconds()>=0.5){
                    Miller.setStoppers(false, true);
                    Miller.setSorterServoPosition(sortPositionRight);
                    Miller.indexerPower(80, Miller.rightIndexerServo);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    setPathState(303);
                }
                break;
            case 303:

                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    setPathState(304);
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    Miller.setIntakePower(75);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    Miller.indexerPower(80, Miller.leftIndexerServo);
                    setPathState(304);
                }
                break;
            case 304:
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    Miller.setStoppers(false, false);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    setPathState(305);
                }
                break;
            case 305:
                /**
                 * Shooting Second Purple Artifact
                 */

                if(pathTimer.getElapsedTimeSeconds() >= 1){
                    switch (currentRow){
                        case TOP_ROW:
                            Miller.setSorterServoPosition(Miller.sortPositionLeft);
                            setPathState(306);
                            break;
                        case MIDDLE_ROW:
                            Miller.setSorterServoPosition(Miller.sortPositionLeft);
                            setPathState(315);
                            break;
                        case BOTTOM_ROW:
                            setPathState(328);
                            break;
                    }


                }
                break;
            case 306:
                /**
                 * Aiming for the Top Row of Artifacts
                 */
//                if(pathTimer.getElapsedTimeSeconds() > 0.25){
//                    Miller.stopEverything();
//                }
                if (pathTimer.getElapsedTimeSeconds() >= 0.30){
                    follower.followPath(aimingFirstBallTopRowPath, 0.80,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(75);
                    Miller.setStoppers(true, true);
                    setPathState(307);
                }
                break;
            case 307:
                /**
                 * Picking up the First Purple Ball from Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickFirstBallTopRowPath, 0.3,true);
                    Miller.setSorterServoPosition(sortPositionRight);
                    Miller.setIntakePower(85);
                    //Miller.setStoppers(true, true);
                    setPathState(308);
                }
                break;
            case 308:
                /**
                 * Pause a little to retrieve the first Purple Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.35){
                    setPathState(309);
                }
                break;
            case 309:
                /**
                 * Advance to the second Purple Ball from Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickSecondBallTopRowPath, 0.3,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-5, Miller.leftIndexerServo);
                    Miller.setStoppers(true, true);
                    setPathState(310);
                }
                break;
            case 310:
                /**
                 * Pause to retrieve the second Purple Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.45){
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    setPathState(311);
                }
                break;
            case 311:
                /**
                 * Retrieve the Green Ball From Top Row
                 */
                if (!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    follower.followPath(pickThirdBallTopRowPath, 0.35,true);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-10, Miller.leftIndexerServo);
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                    setPathState(312);
                }
                break;
            case 312:
                /**
                 * Pause to retrieve the Green Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1.0){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    //Miller.setStoppers(false, false);
                    setPathState(313);
                }
                break;
            case 313:
                /**
                 * Drive to Score Position after retrieving Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(scoreFromTopRowPath, 0.8,true);
                    setPathState(314);
                }
                break;
            case 314:
                /**
                 * Wait for shooter to speed up then shoot
                 */
                if (!follower.isBusy() && Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    Miller.setIntakePower(0.25);
                    Miller.indexerPower(85, Miller.rightIndexerServo);
                    Miller.setSorterServoPosition(Miller.sortPositionLeft);
                    currentRow = Rows.MIDDLE_ROW;
                    setPathState(300);
                }
                break;
            case 315:
                if (pathTimer.getElapsedTimeSeconds() >= 0.10){
                   // Miller.stopEverything();
                    follower.followPath(aimingFirstBallMiddleRowPath, 0.8,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(75);
                    Miller.setStoppers(true, true);
                    setPathState(316);
                }
                break;
            case 316:
                /**
                 * Waiting for aimingFirstBallMiddleRowPath to finish
                 */
                if (!follower.isBusy()){
                    setPathState(317);
                }
                break;
            case 317:
                /**
                 * Picking up the First Purple Ball from Middle Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickFirstBallMiddleRowPath, 0.3,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(85);
                    //Miller.setStoppers(true, true);
                    setPathState(318);
                }
                break;
            case 318:
                /**
                 * Pause a little to retrieve the first Purple Ball from Middle Row
                 */
                if(pathTimer.getElapsedTimeSeconds()>0.45){
                    Miller.setSorterServoPosition(sortPositionMidRight);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.55){
                    setPathState(319);
                    //follower.setMaxPower(.5);
                }
                break;
            case 319:
                /**
                 * Advance to the Green Ball from Middle Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickSecondBallMiddleRowPath, 0.4,true);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-5, Miller.leftIndexerServo);
                    Miller.setStoppers(true, true);
                    setPathState(320);
                }
                break;
            case 320:
                /**
                 * Pause to retrieve the second Green Ball from Middle Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.15){
                    //Miller.setSorterServoPosition(sortPositionMidLeft);
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    setPathState(321);
                }
                break;
            case 321:
                /**
                 * Retrieve the Second Purple Ball From Middle Row
                 */
                if (!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionMidRight);
                    follower.followPath(pickThirdBallMiddleRowPath, 0.45,true);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-10, Miller.rightIndexerServo);
                    setPathState(322);
                }
                break;
            case 322:
                /**
                 * Pause to retrieve the Second Purple Ball from Middle Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.35){
                    // Miller.setSorterServoPosition(sortPositionMidLeft);
                    //Miller.setStoppers(false, false);
                    setPathState(323);
                }
                break;
            case 323:
                /**
                 * Drive to Score Position after retrieving Middle Row
                 */
                follower.setMaxPower(0.9);
                if (!follower.isBusy()){
                    follower.followPath(scoreFromMiddleRowPath, 0.9,true);
                    //Miller.stopEverything();
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                    setPathState(324);
                }
                break;
            case 324:
                /**
                 * Wait for shooter to speed up then shoot
                 */
                if (!follower.isBusy() && Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    //Miller.stopEverything();
                    Miller.setStoppers(true,false);
                    Miller.setIntakePower(75);
                    Miller.indexerPower(85, Miller.leftIndexerServo);
                    //Miller.setSorterServoPosition(Miller.sortPositionRight);
                    currentRow = Rows.BOTTOM_ROW;
                    setPathState(300);
                }
                break;
            case 325:
                /**
                 * Shooting Green from right shooter
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT) && pathTimer.getElapsedTimeSeconds()>0.55){
                    Miller.setStoppers(false,false);
                    Miller.indexerPower(80, Miller.rightIndexerServo);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    Miller.setSorterServoPosition(sortPositionRight);
                    setPathState(326);
                }
                break;
            case 326:
                /**
                 * Shooting Purple from right shooter
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT) && pathTimer.getElapsedTimeSeconds()>0.55){
                    Miller.setStoppers(false,false);
                    Miller.indexerPower(85, Miller.rightIndexerServo);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    Miller.setSorterServoPosition(sortPositionRight);
                    setPathState(327);
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                }
                break;
            case 327:
                /**
                 * Shooting Purple from Left shooter
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT) && pathTimer.getElapsedTimeSeconds()>0.75){
                    Miller.setStoppers(false,false);
                    Miller.indexerPower(90, Miller.leftIndexerServo);
                    Miller.indexerPower(85, Miller.rightIndexerServo);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    setPathState(328);
                }
                break;
            case 328:
                /**
                 * Shooting Purple from Left shooter
                 */
                if(pathTimer.getElapsedTimeSeconds()>0.1){
                    follower.followPath(leavePath, 1,true);
//                    Miller.stopEverything();
                    setPathState(62);
                }
                break;
            case 3066:
                if (!follower.isBusy()){
                    follower.setMaxPower(.7);
                    follower.followPath(grabPickup2);
                    setPathState(62);
                }
                break;
            case 3077:
                if(!follower.isBusy()){
                    follower.followPath(forward);
                    setPathState(62);
                }
                break;
            case 62:
                if(!follower.isBusy()){
                    requestOpModeStop();
                }
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /**
     * This method is called once at the start of the OpMode.
     **/
    @Override
    public void start() {



    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        /**Pedro Pathing Driving
         *
         */

            follower.update();
            autonomousPathUpdate();
            // Feedback to Driver Hub for debugging
            telemetry.addData("motif", motif);
            telemetry.addData("left indexer", Miller.leftIndexerServo.getPower());
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("shooter power Left Error", ((Miller.leftShooterMotor.getVelocity() + Miller.rightShooterMotor.getVelocity())/2) - Miller.TARGET_MIN_VELOCITY_LEFT);
            telemetry.addData("shooter power Right Error", ((Miller.leftShooterMotor.getVelocity() + Miller.rightShooterMotor.getVelocity())/2) - Miller.TARGET_MIN_VELOCITY_RIGHT );
            telemetry.addData("Shooter velocity:", (Miller.rightShooterMotor.getVelocity() + Miller.leftShooterMotor.getVelocity()) / 2);
            telemetry.update();

        }

    public int order(HuskyLens hl) {
        int orden = 0;
        HuskyLens.Block[] blocks = Miller.huskyLens.blocks();
        for (int i = 0; i < blocks.length; i++) {
            if (blocks[i].id == 3) {
                //PPG
                orden = 1;
            }
            if (blocks[i].id == 4) {
                //GPP
                orden = 2;
            }
            if (blocks[i].id == 5){
                //PGP
                orden = 3;

            }
            else if (blocks[i].width != 44 && blocks[i].height != 44){
            }
        }

        return orden;
    }
    }