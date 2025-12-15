package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
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
 * @version 1.0, 02/11/2024
 */
@Autonomous(name = "Reggie_Auto_BLUE", group = "LM1 Reggie")
public class ReggieAuto_v1BLUE extends OpMode {
    //private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
    //Pedro Pathing Variables
    private Follower follower;
    private Reggie Miller;
    public static Pose startingPose;
    public int motif;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    int shooterPower;
    int GshooterPower;
    int artifact;
    double sortPositionMiddle = 0.45;
    double sortPositionRight = 0.1;
    double sortPositionLeft = 0.7;
    private Path Start;
    public PathChain ReadPos, Score,Score2, grab2Pickup1,grab2Pickup12,grab2Pickup12R,grab2Pickup13,grab2Pickup13R,grab2Pickup14,grab2Pickup14R,pickup2Score,grabToPickup1;
    private final Pose startPose = new Pose(21, 124, Math.toRadians(54)); // Start Pose of our robot.
    private final Pose readPos = new Pose(41, 95, Math.toRadians(60)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose = new Pose(53, 87, Math.toRadians(133)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose scorePose2 = new Pose(53, 87, Math.toRadians(125)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupRow1 = new Pose(42, 87, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupRow1R = new Pose(35.5, 87, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupRow12 = new Pose(35, 87, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupRow12R = new Pose(30.5, 87, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupRow13 = new Pose(30, 87, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupRow13R = new Pose(23.5, 87, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupRow14 = new Pose(23, 87, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupRow2 = new Pose(42, 66, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.






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
        shooterPower = 50;
        GshooterPower = 60;
        artifact = 0;
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
                .addPath(new BezierLine(startPose, readPos))
                .setLinearHeadingInterpolation(startPose.getHeading(),Math.toRadians(60))
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Score = follower.pathBuilder()
                .addPath(new BezierLine(readPos, scorePose))
                .setLinearHeadingInterpolation(readPos.getHeading(),scorePose.getHeading())
                .build();
        Score2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupRow13, scorePose2))
                .setLinearHeadingInterpolation(pickupRow13.getHeading(),scorePose2.getHeading())
                .build();
        grab2Pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pickupRow1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupRow1.getHeading())
                .build();
        grab2Pickup12 = follower.pathBuilder()
                .addPath(new BezierLine(pickupRow1,pickupRow12))
                .setLinearHeadingInterpolation(pickupRow1.getHeading(),pickupRow12.getHeading())
                .build();
        grab2Pickup12R = follower.pathBuilder()
                .addPath(new BezierLine(pickupRow12,pickupRow1R))
                .setConstantHeadingInterpolation(pickupRow1R.getHeading())
                .build();
        grab2Pickup13 = follower.pathBuilder()
                .addPath(new BezierLine(pickupRow12,pickupRow13))
                .setLinearHeadingInterpolation(pickupRow12.getHeading(),pickupRow13.getHeading())
                .build();
        grab2Pickup13R = follower.pathBuilder()
                .addPath(new BezierLine(pickupRow13,pickupRow12R))
                .setLinearHeadingInterpolation(pickupRow12.getHeading(),pickupRow13.getHeading())
                .build();
        grab2Pickup14 = follower.pathBuilder()
                .addPath(new BezierLine(pickupRow13,pickupRow14))
                .setLinearHeadingInterpolation(pickupRow13.getHeading(),pickupRow14.getHeading())
                .build();
        grab2Pickup14R = follower.pathBuilder()
                .addPath(new BezierLine(pickupRow14,pickupRow13R))
                .setLinearHeadingInterpolation(pickupRow14.getHeading(),pickupRow13.getHeading())
                .build();
        pickup2Score = follower.pathBuilder()
                .addPath(new BezierLine(pickupRow13,scorePose))
                .setLinearHeadingInterpolation(pickupRow13.getHeading(),scorePose.getHeading())
                .build();
        grabToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2,pickupRow2))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickupRow2.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(.7);
                follower.followPath(ReadPos,true);
                setPathState(11);

                //Miller.shootClose();

                break;
            case 11:
                motif = order(Miller.huskyLens);
                if(pathTimer.getElapsedTimeSeconds() >= 2 && (motif==1 || motif==2 || motif==3)){

                    setPathState(1);
                }
                break;
            case 1:
                //follower.setMaxPower(.8);

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Score,true);
                    Miller.setShooterPower(70);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >=3 ){
                    if ( motif == 1) {
                        setPathState(3);
                    }else if ( motif == 2) {
                        setPathState(4);
                    }else if ( motif == 3) {
                        setPathState(5);
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() >= 15) {
                    break;
                }
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds() >= 2){
                    if ( motif == 1) {
                        setPathState(3);
                    }if ( motif == 2) {
                        setPathState(4);
                    }if ( motif == 3) {
                        setPathState(5);
                    }
                }
                break;
            case 3:
                //PPG
                Miller.indexerPower(0,Miller.leftIndexerServo);
                Miller.setShooterPower(shooterPower);
                Miller.indexerPower(65, Miller.leftIndexerServo);
                Miller.setIntakePower(65,Miller.intakeMotor);
                Miller.setSorterServoPosition(sortPositionLeft);
                if(pathTimer.getElapsedTimeSeconds() >= 5){
                    setPathState(31);
                }
                break;
            case 31:
                Miller.setShooterPower(GshooterPower);
                Miller.setSorterServoPosition(sortPositionRight);
                Miller.indexerPower(80, Miller.rightIndexerServo);
                if (pathTimer.getElapsedTimeSeconds() >= 2 && artifact == 0){
                    setPathState(32);
                }
                break;
            case 32:
                Miller.stopEverything();
                if (artifact == 0) {
                    setPathState(6);
                    shooterPower = 70;
                    GshooterPower = 60;
                }if (artifact == 1){
                    setPathState(7);
            }
                break;
            case 4:
                //GPP
                Miller.indexerPower(0,Miller.leftIndexerServo);
                Miller.setShooterPower(GshooterPower);
                Miller.indexerPower(80, Miller.rightIndexerServo);
                Miller.indexerPower(0, Miller.leftIndexerServo);
                Miller.setSorterServoPosition(sortPositionRight);
                if(pathTimer.getElapsedTimeSeconds() >= 2){
                    setPathState(41);
                    Miller.setShooterPower(65);
                }
                break;
            case 41:
                Miller.setShooterPower(shooterPower);
                Miller.setIntakePower(58, Miller.intakeMotor);
                Miller.setSorterServoPosition(sortPositionLeft);
                Miller.indexerPower(65, Miller.leftIndexerServo);
                if(pathTimer.getElapsedTimeSeconds() >= 5){
                    setPathState(42);
                }
                break;
            case 42:
                if (pathTimer.getElapsedTimeSeconds() >= 2){
                    setPathState(32);
                }
                break;
            case 5:
                //PGP
                Miller.indexerPower(0,Miller.leftIndexerServo);
                Miller.setShooterPower(shooterPower);
                Miller.indexerPower(50, Miller.leftIndexerServo);
                //Miller.setIntakePower(100, Miller.intakeMotor);
                Miller.setSorterServoPosition(sortPositionLeft);
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    setPathState(51);
                }
                break;
            case 51:
                Miller.setShooterPower(GshooterPower);
                Miller.setSorterServoPosition(sortPositionRight);
                Miller.indexerPower(80, Miller.rightIndexerServo);
                Miller.indexerPower(0, Miller.leftIndexerServo);
                if (pathTimer.getElapsedTimeSeconds() >= 1.75) {
                    setPathState(52);
                    Miller.setIntakePower(80, Miller.intakeMotor);
                }
                break;
            case 52:
                Miller.setShooterPower(shooterPower);
                Miller.setSorterServoPosition(sortPositionLeft);
                Miller.indexerPower(0, Miller.rightIndexerServo);
                Miller.indexerPower(80, Miller.leftIndexerServo);
                if (pathTimer.getElapsedTimeSeconds() >= 3) {
                    setPathState(32);
                }
                break;
            case 6:
                if (!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionRight);
                    follower.followPath(grab2Pickup1,true);

                    setPathState(61);
                    Miller.setShooterPower(0);
                    Miller.setIntakePower(0.65, Miller.intakeMotor);
                    Miller.indexerPower(-100, Miller.rightIndexerServo);
                }
                break;
            case 61:
                if (!follower.isBusy()){
                    follower.setMaxPower(.45);
                    //Miller.setSorterServoPosition(0.8);
                    follower.followPath(grab2Pickup12,true);
                    Miller.indexerPower(-10, Miller.rightIndexerServo);
                    setPathState(62);

                }
                break;
            case 62:
                if (!follower.isBusy()){
                    follower.followPath(grab2Pickup12R,true);
                    setPathState(63);
                }
                break;
            case 63:
                if (!follower.isBusy()){
                    //Miller.setSorterServoPosition(sortPositionLeft);
                    follower.followPath(grab2Pickup13,true);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    setPathState(64);
                }
                break;
            case 64:
                if (!follower.isBusy()){
                    Miller.indexerPower(-60, Miller.leftIndexerServo);
                    follower.followPath(grab2Pickup13R);
                    setPathState(65);
                }
                break;
            case 65:
                if (!follower.isBusy()){
                    follower.followPath(grab2Pickup14);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    //Miller.setShooterPower(-100);
                    setPathState(66);
                }
            case 66:
                if (!follower.isBusy()){
                    follower.followPath(Score2);
                    Miller.setSorterServoPosition(sortPositionRight);
                    Miller.indexerPower(0,Miller.rightIndexerServo);
                    setPathState(67);
                }
                break;
            case 67:
                if (pathTimer.getElapsedTimeSeconds() == 4 || !follower.isBusy()){
                    artifact = 1;
                    Miller.indexerPower(-50,Miller.leftIndexerServo);
                    Miller.setShooterPower(50);
                    if (pathTimer.getElapsedTimeSeconds() >= 3) {
                        setPathState(22);
                        follower.setMaxPower(1);
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    follower.followPath(grabToPickup1);
                    setPathState(71);
                }
                break;
            case 71:
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
    }