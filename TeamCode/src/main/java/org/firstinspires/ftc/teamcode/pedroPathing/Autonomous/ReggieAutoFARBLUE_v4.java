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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Reggie;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.DataStorage;


/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 02/11/2024
 */
@Autonomous(name = "BLUE_FAR_STATE", group = "State Reggie Far")
public class ReggieAutoFARBLUE_v4 extends OpMode {
    //private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
    //Pedro Pathing Variables
    private Follower follower;
    public DcMotorEx leftShooterMotor = null;
    public DcMotorEx rightShooterMotor = null;
    private Reggie Miller;
    public static Pose startingPose;
    public int motif;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    int longDistanceVelocity = 1400;
    int shooterPower;
    int GshooterPower;
    int artifact;
    double sortPositionMiddle = 0.45;
    double sortPositionRight = 0.1;
    double sortPositionLeft = 0.65;
    public double TARGET_VELOCITY_LEFT = 1200;
    public double TARGET_MIN_VELOCITY_LEFT = 1190;
    public double TARGET_VELOCITY_RIGHT = 1300;
    public double TARGET_MIN_VELOCITY_RIGHT = 1290;
    private Path Start;
    public boolean lastShot = false;
    public enum ShooterSide{
        LEFT,
        RIGHT
    }
    public PathChain ReadPos, Score, aimRow1, pickupGreen, pickupPurple, Park,startToLeave, score2leave, purple2score;
    private final Pose startPose = new Pose(58, 9, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose readPos = new Pose(58, 46, Math.toRadians(92)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose = new Pose(58, 21, Math.toRadians(113)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose aimRow = new Pose(44,44,Math.toRadians(180));
    private final Pose aimRowControl = new Pose(54,32.6,Math.toRadians(180));
    private final Pose rowGreen = new Pose(40,50,Math.toRadians(180));
    private final Pose rowPurple = new Pose(26,50,Math.toRadians(180));
    private final Pose park = new Pose(14, 9.6, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.



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
        artifact = 0;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "LS");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "RS");


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
        aimRow1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,aimRowControl,aimRow))
                .setLinearHeadingInterpolation(scorePose.getHeading(), aimRow.getHeading())
                .build();
        pickupGreen = follower.pathBuilder()
                .addPath(new BezierLine(aimRow,rowGreen))
                .setConstantHeadingInterpolation(aimRow.getHeading())
                .build();
        pickupPurple = follower.pathBuilder()
                .addPath(new BezierLine(rowGreen,rowPurple))
                .setConstantHeadingInterpolation(rowPurple.getHeading())
                .build();
        Park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,park))
                .setLinearHeadingInterpolation(readPos.getHeading(), park.getHeading())
                .build();
        startToLeave = follower.pathBuilder()
                .addPath(new BezierLine(startPose,park))
                .setLinearHeadingInterpolation(startPose.getHeading(), park.getHeading())
                .build();
        score2leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,park))
                .setConstantHeadingInterpolation(park.getHeading())
                .build();
        purple2score = follower.pathBuilder()
                .addPath(new BezierLine(rowPurple,scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(.9);
                //follower.followPath(ReadPos,true);
                //Miller.shootClose();
                setPathState(11);

                break;
            case 11:
                //motif = order(Miller.huskyLens);
                motif = 2;
                if(pathTimer.getElapsedTimeSeconds() >= 0.5 && (motif==1 || motif==2 || motif==3)){

                    setPathState(1);
                }
                break;
            case 1:
                //follower.setMaxPower(.8);

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                ///if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 10) {
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Score,true);
                    //Miller.setStoppers(false,false);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >=1 ){
                    if ( motif == 1) {
                        Miller.TARGET_MIN_VELOCITY_RIGHT = 1150;
                        Miller.TARGET_VELOCITY_RIGHT = 1100;
                        Miller.TARGET_MIN_VELOCITY_LEFT = 1100;
                        Miller.TARGET_VELOCITY_LEFT = 1150;
                        setShooterSide(ShooterSide.LEFT);
                        setPathState(300);
                    }else if ( motif == 2) {
                        Miller.TARGET_MIN_VELOCITY_RIGHT = 1360;
                        Miller.TARGET_VELOCITY_RIGHT = 1440;
                        Miller.TARGET_MIN_VELOCITY_LEFT = 1300;
                        Miller.TARGET_VELOCITY_LEFT = 1450;
                        setShooterSide(ShooterSide.RIGHT);
                        setPathState(4);
                    }else if ( motif == 3) {
                        Miller.TARGET_MIN_VELOCITY_RIGHT = 1350;
                        Miller.TARGET_VELOCITY_RIGHT = 1400;
                        Miller.TARGET_MIN_VELOCITY_LEFT = 1300;
                        Miller.TARGET_VELOCITY_LEFT = 1350;
                        setShooterSide(ShooterSide.LEFT);
                        setPathState(500);
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() >= 15) {
                    break;
                }
                break;
            case 300:
                //PPG
                Miller.indexerPower(25, Miller.leftIndexerServo);
                Miller.setIntakePower(65);
                Miller.setSorterServoPosition(sortPositionLeft);
                setShooterSide(ShooterSide.RIGHT);
                if(targetVelocityAcquired(ShooterSide.RIGHT )&& (pathTimer.getElapsedTimeSeconds() >= 2)){
                    Miller.setSorterServoPosition(sortPositionRight);
                    setPathState(-301);
                }
                break;
            case 301:
                Miller.indexerPower(80, Miller.rightIndexerServo);
                if (pathTimer.getElapsedTimeSeconds() >= 2 && artifact == 0){
                    setPathState(32);
                }
                break;
            case 32:
                follower.setMaxPower(1);
                follower.followPath(Park);
                Miller.stopEverything();
                break;
            case 4:
                //GPP
                if(!follower.isBusy() && targetVelocityAcquired(ShooterSide.LEFT) && pathTimer.getElapsedTimeSeconds() >= 0.8){
                    Miller.setStoppers(true,true);
                    Miller.indexerPower(0,Miller.leftIndexerServo);
                    Miller.indexerPower(80, Miller.rightIndexerServo);
                    Miller.setSorterServoPosition(sortPositionRight);
                    if(targetVelocityAcquired(ShooterSide.LEFT) && pathTimer.getElapsedTimeSeconds() >= 1){
                        Miller.setStoppers(false,true);
                    setPathState(41);

                        Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                    }
                }
                break;
            case 41:
                if (targetVelocityAcquired(ShooterSide.RIGHT) && pathTimer.getElapsedTimeSeconds() > 1){
                    Miller.setIntakePower(58);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.indexerPower(30, Miller.leftIndexerServo);
                    if(pathTimer.getElapsedTimeSeconds() >= 3){
                        Miller.setStoppers(false,false);
                        setPathState(42);
                    }
                }
                break;
            case 42:
                if (pathTimer.getElapsedTimeSeconds() >= 2){
                    Miller.indexerPower(0,Miller.leftIndexerServo);
                    Miller.indexerPower(0,Miller.rightIndexerServo);
                    Miller.setStoppers(true,true);
                    if (!lastShot){
                        setPathState(43);
                }else{
                    setPathState(47);
                }
        }
                break;
            case 43 :
                if (!follower.isBusy()){
                    follower.followPath(aimRow1,.45,true);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    setPathState(44);
                }
                break;
            case 44:
                if (!follower.isBusy()){
                    Miller.setIntakePower(50);
                    follower.followPath(pickupGreen,.45,true);
                    setPathState(45);
                }
                break;
            case 45:
                if (!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionRight);
                    follower.followPath(pickupPurple,.25,true);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    setPathState(46);
                }
                break;
            case 46:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1){
                    follower.followPath(purple2score,.9,true);
                    lastShot = true;
                    setPathState(4);
                }
                break;
            case 47:
                if (!follower.isBusy()){
                    follower.followPath(score2leave);
                    setPathState(48);
                }
                break;
            case 48:
                if(!follower.isBusy()){
                    DataStorage.currentPose = follower.getPose();
                    DataStorage.Blue = true;
                    requestOpModeStop();
                }
                break;
            case 5:
                //PGP
                Miller.indexerPower(0,Miller.leftIndexerServo);
                setShooterSide(ShooterSide.LEFT);
                Miller.indexerPower(65, Miller.leftIndexerServo);
                //Miller.setIntakePower(100, Miller.intakeMotor);
                Miller.setSorterServoPosition(sortPositionLeft);
                setShooterSide(ShooterSide.RIGHT);
                if (targetVelocityAcquired(ShooterSide.RIGHT)) {
                    setPathState(51);
                }
                break;
            case 51:
                Miller.setSorterServoPosition(sortPositionRight);
                Miller.indexerPower(80, Miller.rightIndexerServo);
                Miller.indexerPower(0, Miller.leftIndexerServo);
                setShooterSide(ShooterSide.LEFT);
                if (targetVelocityAcquired(ShooterSide.LEFT)) {
                    setPathState(52);
                    Miller.setIntakePower(80);
                }
                break;
            case 52:
                Miller.setSorterServoPosition(sortPositionLeft);
                Miller.indexerPower(0, Miller.rightIndexerServo);
                Miller.indexerPower(80, Miller.leftIndexerServo);
                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    setPathState(32);
                }
                break;
            case 900:
                if(!follower.isBusy()){
                    follower.followPath(startToLeave);
                    setPathState(901);
                }
                break;
            case 901:
                if(!follower.isBusy()){
                    requestOpModeStop();
                }
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
    public void setShooterSide(ShooterSide side) {
        if (side == ShooterSide.LEFT) {
            leftShooterMotor.setVelocity(TARGET_VELOCITY_LEFT);
            rightShooterMotor.setVelocity(TARGET_VELOCITY_LEFT);
        } else {
            leftShooterMotor.setVelocity(TARGET_VELOCITY_RIGHT);
            rightShooterMotor.setVelocity(TARGET_VELOCITY_RIGHT);
        }
    }
    public boolean targetVelocityAcquired(ShooterSide side){
        if (side == ShooterSide.LEFT) {
            return leftShooterMotor.getVelocity() > TARGET_MIN_VELOCITY_LEFT && rightShooterMotor.getVelocity() > TARGET_MIN_VELOCITY_LEFT;
        }
        return leftShooterMotor.getVelocity() > TARGET_MIN_VELOCITY_RIGHT && rightShooterMotor.getVelocity() > TARGET_MIN_VELOCITY_RIGHT;
    }
    }