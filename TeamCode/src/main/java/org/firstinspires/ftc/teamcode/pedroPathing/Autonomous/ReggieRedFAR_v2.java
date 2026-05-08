package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Reggie;



/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 02/11/2024
 */
@Disabled
@Autonomous(name = "RED_FAR_LM3_Tournament", group = "LM3/TOUR Reggie")
public class ReggieRedFAR_v2 extends OpMode {
    //private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
    //Pedro Pathing Variables

    private enum Rows{
        TOP_ROW,
        MIDDLE_ROW,
        BOTTOM_ROW,
        BASE
    }
    private Follower follower;
    private Reggie Miller;
    public static Pose startingPose;
    public int motif;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public static double yTopRowPurple = 81;
    public static double yTopRowGreen = 85;

    public static double yMidRowPurple = 60;
    public static double yMidRowGreen = 62;
    public static double yBotRowPurple = 33;
    public static double yBotRowGreen = 36;
    int shooterPower;
    int GshooterPower;
    int artifact;
    public static double shootingAngleFirst = 66;
    public static double shootingAngleSecond = 69;
    double sortPositionMiddle = 0.45;
    double sortPositionRight = 0.1;
    double sortPositionMidRight = 0.25;
    double sortPositionLeft = 0.9;
    double sortPositionMidLeft = 0.75;

    private Path Start;
    public PathChain ReadPos, Score, Park, aimingFirstBallBotRowPath, pickFirstBallBotRowPath, pickThirdBallBotRowPath,
            pickSecondBallBotRowPath, scoreFromBotRowPath;
    private final Pose startPose = new Pose(87, 9, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose readPos = new Pose(87, 40, Math.toRadians(88)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose = new Pose(83, 28, Math.toRadians(shootingAngleFirst)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose secondScorePose = new Pose(85, 26, Math.toRadians(shootingAngleSecond)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scoreSecondControlPose = new Pose(110, 25, Math.toRadians(shootingAngleFirst)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose park = new Pose(109, 14, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose aimingFirstBallBotRowPose = new Pose(90, yBotRowGreen, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickFirstBallBotRowPose = new Pose(105, yBotRowGreen, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickSecondBallBotRowPose = new Pose(109, yBotRowPurple, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallBotRowPose = new Pose(122, yBotRowPurple, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickSecondBallBotRowControlPose = new Pose(96, yBotRowPurple, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    public Rows currentRow = Rows.BOTTOM_ROW;


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
        shooterPower = 70;
        GshooterPower = 70;
        artifact = 0;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        Miller.TARGET_MIN_VELOCITY_RIGHT = 1350;
        Miller.TARGET_VELOCITY_RIGHT = 1400;
        Miller.TARGET_MIN_VELOCITY_LEFT = 1300;
        Miller.TARGET_VELOCITY_LEFT = 1350;
        //Rows currentRow = Rows.BOTTOM_ROW;
        //motif = order(Miller.huskyLens);

        // playTime.reset();
    }

    public void buildPaths() {
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        ReadPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, readPos))
                .setLinearHeadingInterpolation(startPose.getHeading(),readPos.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Score = follower.pathBuilder()
                .addPath(new BezierLine(readPos, scorePose))
                .setLinearHeadingInterpolation(readPos.getHeading(),scorePose.getHeading())
                .build();
        Park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,park))
                .setLinearHeadingInterpolation(readPos.getHeading(), park.getHeading())
                .build();

        /* Aim to get the first ball from the top row */
        aimingFirstBallBotRowPath = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,new Pose(93, 30), aimingFirstBallBotRowPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),aimingFirstBallBotRowPose.getHeading())
                .build();

        /* Drive to pick the first ball then stop to sort */
        pickFirstBallBotRowPath = follower.pathBuilder()
                .addPath(new BezierLine(aimingFirstBallBotRowPose, pickFirstBallBotRowPose))
                .setLinearHeadingInterpolation(aimingFirstBallBotRowPose.getHeading(),pickFirstBallBotRowPose.getHeading())
                .build();

        /* Drive to pick the Second ball from top row */
        pickSecondBallBotRowPath = follower.pathBuilder()
                .addPath(new BezierCurve(pickFirstBallBotRowPose,pickSecondBallBotRowControlPose, pickSecondBallBotRowPose))
                .setLinearHeadingInterpolation(pickFirstBallBotRowPose.getHeading(),pickSecondBallBotRowPose.getHeading())
                .build();

        /* Drive to pick the Third ball from top row */
        pickThirdBallBotRowPath = follower.pathBuilder()
                .addPath(new BezierCurve(pickSecondBallBotRowPose, pickThirdBallBotRowPose))
                .setLinearHeadingInterpolation(pickSecondBallBotRowPose.getHeading(),pickThirdBallBotRowPose.getHeading())
                .build();

        /* Drive to Score from the Third ball from top row */
        scoreFromBotRowPath = follower.pathBuilder()
                .addPath(new BezierCurve(pickThirdBallBotRowPose, scoreSecondControlPose ,secondScorePose))
                .setLinearHeadingInterpolation(pickThirdBallBotRowPose.getHeading(),secondScorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(.9);
                follower.followPath(ReadPos,true);
                setPathState(11);

                //Miller.shootClose();

                break;
            case 11:
                //motif = order(Miller.huskyLens);

                if(pathTimer.getElapsedTimeSeconds() >= 1 && (motif==1 || motif==2 || motif==3)){
                    Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                    setPathState(1);
                }
                break;
            case 1:
                //follower.setMaxPower(.8);

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.75) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Score,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >=0.5 ){
                    if ( motif == 1) {
                        setPathState(100);

                        Miller.TARGET_MIN_VELOCITY_RIGHT = 1350;
                        Miller.TARGET_VELOCITY_RIGHT = 1450;
                        Miller.TARGET_MIN_VELOCITY_LEFT = 1350;
                        Miller.TARGET_VELOCITY_LEFT = 1450;
                        Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    }else if ( motif == 2) {
                        setPathState(200);
                    }else if ( motif == 3) {
                        setPathState(300);
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() >= 5) {
                    setPathState(100);
                    break;
                }
                break;
            case 100:
                //PPG
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    Miller.setStoppers(true, false);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    Miller.indexerPower(80, Miller.leftIndexerServo);
                    setPathState(101);
                }
                break;
            case 101:
                //Wait to shoot first purple
                if(pathTimer.getElapsedTimeSeconds()>1.250 && Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    Miller.setIntakePower(80);
                    setPathState(102);
                }
                break;
            case 102:
                //shoot second purple
                if(pathTimer.getElapsedTimeSeconds()>0.650){
                    Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                    Miller.setIntakePower(0);
                    setPathState(103);

                }
                break;
            case 103:
                if(pathTimer.getElapsedTimeSeconds()>0.250 && Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT)){
                    Miller.setStoppers(false, false);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(0);
                    Miller.indexerPower(80, Miller.rightIndexerServo);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    setPathState(104);
                }
                break;
            case 104:
                if(pathTimer.getElapsedTimeSeconds()>=0.850) {
                    follower.followPath(aimingFirstBallBotRowPath, 0.85, true);
                    Miller.setShooterPower(0);
                    if(currentRow==Rows.BOTTOM_ROW) {
                        setPathState(105);
                    }else{
                        setPathState(67);
                    }
                }
                break;
            case 105:
                if(!follower.isBusy()){
                    Miller.setIntakePower(85);
                    Miller.setStoppers(true, true);
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    follower.followPath(pickFirstBallBotRowPath, 0.45,true);
                    setPathState(106);
                }
                break;
            case 106:
                if(!follower.isBusy()){
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                    follower.followPath(pickSecondBallBotRowPath, 0.35, true);
                    Miller.setSorterServoPosition(sortPositionRight);
                    setPathState(107);
                }
                break;
            case 107:
                if(!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionMidRight);
                    follower.followPath(pickThirdBallBotRowPath, 0.45, true);
                    setPathState(108);
                }
                break;
            case 108:
                if(!follower.isBusy()){
                    follower.followPath(scoreFromBotRowPath, 0.75, true);
                    setPathState(109);
                }
                break;
            case 109:
                if(!follower.isBusy()){
                    currentRow = Rows.BASE;
                    setPathState(100);
                }
                break;
            case 200:
                //GPP
                if(Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT)){
                    Miller.setStoppers(false, true);
                    Miller.setSorterServoPosition(sortPositionRight);
                    Miller.indexerPower(80, Miller.rightIndexerServo);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    setPathState(201);
                }

                break;
            case 201:
                if(pathTimer.getElapsedTimeSeconds()>0.650){
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    setPathState(202);
                }
                break;
            case 202:
                if(pathTimer.getElapsedTimeSeconds()>0.850 && Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    Miller.setStoppers(false, false);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(0);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    Miller.indexerPower(80, Miller.leftIndexerServo);
                    setPathState(203);
                }
                break;
            case 203:
                if (pathTimer.getElapsedTimeSeconds() >= 1.250 && Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    Miller.setIntakePower(80);
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    setPathState(204);
                }
                break;
            case 204:
                if(pathTimer.getElapsedTimeSeconds()>=0.850) {
                    follower.followPath(aimingFirstBallBotRowPath, 0.85, true);
                    Miller.setShooterPower(0);
                    if(currentRow==Rows.BOTTOM_ROW) {
                        setPathState(205);
                    }else{
                        setPathState(67);
                    }
                }
                break;
            case 205:
                if(!follower.isBusy()){
                    Miller.setIntakePower(85);
                    Miller.setStoppers(true, true);
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    follower.followPath(pickFirstBallBotRowPath, 0.45,true);
                    setPathState(206);
                }
                break;
            case 206:
                if(!follower.isBusy()){
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                    follower.followPath(pickSecondBallBotRowPath, 0.35, true);
                    Miller.setSorterServoPosition(sortPositionRight);
                    setPathState(207);
                }
                break;
            case 207:
                if(!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionMidRight);
                    follower.followPath(pickThirdBallBotRowPath, 0.45, true);
                    setPathState(208);
                }
                break;
            case 208:
                if(!follower.isBusy()){
                    follower.followPath(scoreFromBotRowPath, 0.75, true);
                    setPathState(209);
                }
                break;
            case 209:
                if(!follower.isBusy()){
                    currentRow = Rows.BASE;
                    setPathState(200);
                }
                break;
            case 300:
                //PGP
                Miller.setIntakePower(0);
                Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                if((Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT) && (pathTimer.getElapsedTimeSeconds() >= 2))){
                    Miller.setStoppers(true, false);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    if (pathTimer.getElapsedTimeSeconds() >= 3) {
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    Miller.indexerPower(80, Miller.leftIndexerServo);
                    setPathState(301);
                    }
                }

                break;
            case 301:
                if(pathTimer.getElapsedTimeSeconds()>0.650){
                    Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                    Miller.setStoppers(false, true);
                    setPathState(302);
                }
                break;
            case 302:
                if(pathTimer.getElapsedTimeSeconds()>0.850 && Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT)){
                    Miller.setStoppers(false, false);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(0);
                    Miller.indexerPower(80, Miller.rightIndexerServo);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    setPathState(303);
                }
                break;
            case 303:
                if (pathTimer.getElapsedTimeSeconds() >= 1.250 && Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    Miller.indexerPower(80, Miller.leftIndexerServo);
                    Miller.setIntakePower(80);
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    setPathState(304);
                }
                break;
            case 304:
                if(pathTimer.getElapsedTimeSeconds()>=0.850) {
                    follower.followPath(aimingFirstBallBotRowPath, 0.85, true);
                    Miller.setShooterPower(0);
                    if(currentRow==Rows.BOTTOM_ROW) {
                        Miller.indexerPower(0, Miller.leftIndexerServo);
                        Miller.indexerPower(0, Miller.rightIndexerServo);
                        setPathState(305);
                    }else{
                        setPathState(67);
                    }
                }
                break;
            case 305:
                if(!follower.isBusy()){
                    Miller.setIntakePower(85);
                    Miller.setStoppers(true, true);
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    follower.followPath(pickFirstBallBotRowPath, 0.45,true);
                    setPathState(306);
                }
                break;
            case 306:
                if(!follower.isBusy()){
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                    follower.followPath(pickSecondBallBotRowPath, 0.35, true);
                    Miller.setSorterServoPosition(sortPositionRight);
                    setPathState(307);
                }
                break;
            case 307:
                if(!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionMidRight);
                    follower.followPath(pickThirdBallBotRowPath, 0.45, true);
                    setPathState(308);
                }
                break;
            case 308:
                if(!follower.isBusy()){
                    follower.followPath(scoreFromBotRowPath, 0.75, true);
                    setPathState(309);
                }
                break;
            case 309:
                if(!follower.isBusy()){
                    currentRow = Rows.BASE;
                    setPathState(300);
                }
                break;
            case 67:
                follower.followPath(Park);
                Miller.stopEverything();
                break;
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
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();

        }


    }