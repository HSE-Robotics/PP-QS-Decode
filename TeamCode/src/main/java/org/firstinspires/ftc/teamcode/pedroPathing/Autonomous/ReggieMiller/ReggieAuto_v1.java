package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.ReggieMiller;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Reggie;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

@Disabled
/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 02/11/2024
 */
@Autonomous(name = "Reggie_Auto_RED", group = "LM1 Reggie")
public class ReggieAuto_v1 extends OpMode {
    //private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
    //Pedro Pathing Variables
    private Follower follower;
    private Reggie Miller;
    public static Pose startingPose;
    public int motif;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final int GShoot = 20;
    private final int PShoot = 65;
    double sortPositionMiddle = 0.45;
    double sortPositionRight = 0.1;
    double sortPositionLeft = 0.9;
    private Path Start;
    private PathChain ReadPos, Score, grabPickup2,forward;
    private final Pose startPose = new Pose(110, 122, Math.toRadians(135)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(83, 95, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose1 = new Pose(84, 90, Math.toRadians(90)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
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
        //motif = order(Miller.huskyLens);

        // playTime.reset();
    }

    public void buildPaths() {
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        ReadPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),Math.toRadians(105))
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Score = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1Pose.getHeading())
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
                follower.setMaxPower(.9);
                follower.followPath(ReadPos,true);
                setPathState(11);

                //Miller.shootClose();
                if(pathTimer.getElapsedTimeSeconds() >= 6){

                    break;
                }
                break;
            case 11:
                motif = order(Miller.huskyLens);
                if(pathTimer.getElapsedTimeSeconds() >= 2 && (motif==1 || motif==2 || motif==3)){

                    setPathState(1);
                }
                break;
            case 1:
                follower.setMaxPower(.8);

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Score,true);
//                    Miller.setShooterPower(100);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >=1.5 ){
                    if ( motif == 1) {
                        setPathState(3);
                    }else if ( motif == 2) {
                        Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                        setPathState(4);
                    }else if ( motif == 3) {
                        setPathState(5);
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() >= 15) {
                    break;
                }
                break;
            case 3:
                //PPG
                Miller.setShooterPower(PShoot);
                Miller.indexerPower(65, Miller.leftIndexerServo);
                Miller.setIntakePower(5);
                Miller.setSorterServoPosition(sortPositionLeft);
                if(pathTimer.getElapsedTimeSeconds() >= 2){
                    setPathState(31);
                }
                break;
            case 31:
                Miller.setShooterPower(GShoot);
                Miller.setSorterServoPosition(sortPositionRight);
                Miller.indexerPower(80, Miller.rightIndexerServo);
                if (pathTimer.getElapsedTimeSeconds() >= 2){
                    setPathState(32);
                }
                break;
            case 32:
                Miller.indexerPower(0, Miller.rightIndexerServo);
                Miller.indexerPower(0, Miller.leftIndexerServo);
                Miller.setShooterPower(0);
                Miller.setIntakePower(50);
                setPathState(6);
                break;
            case 4:
                //GPP
                //Miller.setShooterPower(GShoot);
                if(Miller.leftShooterMotor.getVelocity() > Miller.TARGET_MIN_VELOCITY_RIGHT && Miller.rightShooterMotor.getVelocity() > Miller.TARGET_MIN_VELOCITY_RIGHT ){
                    setPathState(401);
                }
                break;
            case 401:
                //GPP OLD PathState4
                //Miller.setShooterPower(GShoot);
                Miller.indexerPower(85, Miller.rightIndexerServo);
                Miller.indexerPower(0, Miller.leftIndexerServo);
                Miller.setSorterServoPosition(sortPositionRight);
                if(pathTimer.getElapsedTimeSeconds() >= 1.05){
                    setPathState(402);
                    //Miller.setShooterPower(100);
                }
                break;
            case 402:
                if(Miller.leftShooterMotor.getVelocity() > Miller.TARGET_MIN_VELOCITY_LEFT && Miller.rightShooterMotor.getVelocity() > Miller.TARGET_MIN_VELOCITY_LEFT ){
                    setPathState(41);
                }
                break;
             case 41:

                Miller.setIntakePower(70);
                Miller.setSorterServoPosition(sortPositionLeft);
                Miller.indexerPower(85, Miller.leftIndexerServo);
                if(pathTimer.getElapsedTimeSeconds() >= 0.75){
                    Miller.setIntakePower(0);
                    //Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    setPathState(411);
                }
                break;
            case 411:
                if(Miller.leftShooterMotor.getVelocity() > Miller.TARGET_MIN_VELOCITY_LEFT && Miller.rightShooterMotor.getVelocity() > Miller.TARGET_MIN_VELOCITY_LEFT ){
                    setPathState(412);
                }
                break;
            case 412:
                Miller.setIntakePower(75);
                Miller.setSorterServoPosition(sortPositionLeft);
                Miller.indexerPower(85, Miller.leftIndexerServo);
                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    Miller.setSorterServoPosition(Miller.sortPositionLeft);
                    setPathState(42);
                }
                break;
            case 42:
                if (pathTimer.getElapsedTimeSeconds() >= 0.75){
                    setPathState(32);
                }
                break;
            case 5:
                //PGP
                Miller.setShooterPower(PShoot);
                Miller.indexerPower(15, Miller.leftIndexerServo);
                //Miller.setIntakePower(100, Miller.intakeMotor);
                Miller.setSorterServoPosition(sortPositionLeft);
                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    setPathState(51);
                }
                break;
            case 51:
                Miller.setShooterPower(GShoot);
                Miller.setSorterServoPosition(sortPositionRight);
                Miller.indexerPower(80, Miller.rightIndexerServo);
                Miller.indexerPower(0, Miller.leftIndexerServo);
                if (pathTimer.getElapsedTimeSeconds() >= 3) {
                    setPathState(52);
                    Miller.setIntakePower(100);
                }
                break;
            case 52:
                Miller.setShooterPower(PShoot);
                Miller.setSorterServoPosition(sortPositionLeft);
                Miller.indexerPower(0, Miller.rightIndexerServo);
                Miller.indexerPower(80, Miller.leftIndexerServo);
                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    setPathState(32);
                }
                break;
            case 6:
                if (!follower.isBusy()){
                    follower.setMaxPower(.7);
                    follower.followPath(grabPickup2);
                    setPathState(61);
                }
                break;
            case 61:
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
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("shooter power Left", Miller.leftShooterMotor.getPower());
            telemetry.addData("shooter power Right", Miller.rightShooterMotor.getPower());
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