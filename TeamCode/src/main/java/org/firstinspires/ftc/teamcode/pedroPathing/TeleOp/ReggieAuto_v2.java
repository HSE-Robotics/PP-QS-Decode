package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;
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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Reggie;


/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 02/11/2024
 */
@Autonomous(name = "Reggie_Auto_RED_LM3_Tournament", group = "LM3/TOUR Reggie")
public class ReggieAuto_v2 extends OpMode {
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
    double sortPositionLeft = 0.9;
    private enum Rows{
        TOP_ROW,
        MIDDLE_ROW,
        BOTTOM_ROW
    }
    Rows currentRow = Rows.TOP_ROW;
    private Path Start;
    private PathChain ReadPath, ScorePath, aimingFirstBallTopRowPath, pickFirstBallTopRowPath, pickSecondBallTopRowPath,
                      pickThirdBallTopRowPath, grabPickup2,forward, scoreFromTopRowPath, scoreFromMiddleRowPath;
    private final Pose startPose = new Pose(125, 118, Math.toRadians(127)); // Start Pose of our robot.
    private final Pose readPose = new Pose(93, 95, Math.toRadians(105)); // Reading Obelisk Pose of our robot. It is facing the obelisk at 127 degree angle.
    private final Pose scorePose = new Pose(90, 95, Math.toRadians(28)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose scorePoseSecond = new Pose(92, 97, Math.toRadians(38)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose scorePoseThird = new Pose(90, 95, Math.toRadians(30)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose aimingFirstBallTopRowPose = new Pose(100, 74, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickFirstBallTopRowPose = new Pose(108, 74, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickSecondBallTopRowPose = new Pose(113, 74, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallTopRowPose = new Pose(123, 85, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallTopRowControlPose = new Pose(92, 85, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose aimingFirstBallMiddleRowPose = new Pose(100, 59, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickFirstBallMiddleRowPose = new Pose(108, 59, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickSecondBallMiddleRowPose = new Pose(113, 59, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose pickThirdBallMiddleRowPose = new Pose(122, 59, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
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
                .addPath(new BezierCurve(scorePose,new Pose(90.000, 83.000), aimingFirstBallTopRowPose))
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
                .addPath(new BezierLine(pickThirdBallTopRowPose, scorePoseSecond))
                .setLinearHeadingInterpolation(pickThirdBallTopRowPose.getHeading(),scorePoseSecond.getHeading())
                .build();

        /* Drive to Score from the Third ball from Middle row */
        scoreFromMiddleRowPath = follower.pathBuilder()
                .addPath(new BezierLine(pickThirdBallTopRowPose, scorePoseSecond))
                .setLinearHeadingInterpolation(pickThirdBallTopRowPose.getHeading(),scorePoseSecond.getHeading())
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
                follower.followPath(ReadPath,0.8, true);
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
                    follower.followPath(ScorePath,0.8, true);
//                    Miller.setShooterPower(100);
                    //Miller.setShooterVelocity(Miller.TARGET_VELOCITY);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >=1.5 ){
                    if ( motif == 1) {
                        setPathState(100);
                    }else if ( motif == 2) {
                        Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                        setPathState(200);
                    }else if ( motif == 3) {
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
                Miller.setShooterPower(PShoot);
                Miller.indexerPower(65, Miller.leftIndexerServo);
                Miller.setIntakePower(5,Miller.intakeMotor);
                Miller.setSorterServoPosition(sortPositionLeft);
                if(pathTimer.getElapsedTimeSeconds() >= 2){
                    setPathState(101);
                }
                break;
            case 101:
                Miller.setShooterPower(GShoot);
                Miller.setSorterServoPosition(sortPositionRight);
                Miller.indexerPower(80, Miller.rightIndexerServo);
                if (pathTimer.getElapsedTimeSeconds() >= 2){
                    setPathState(102);
                }
                break;
            //Former 32 State
            case 102:
                Miller.indexerPower(0, Miller.rightIndexerServo);
                Miller.indexerPower(0, Miller.leftIndexerServo);
                Miller.setShooterPower(0);
                Miller.setIntakePower(50,Miller.intakeMotor);
                setPathState(6);
                break;
            case 200:
                /**
                 * Green - Purple - Purple
                 * Waiting for the shooter to speed up to shoot Green Artifact
                 */
                //GPP
                //Miller.setShooterPower(GShoot);
                if(Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT) ){
                    Miller.setStoppers(false,true);
                    setPathState(201);
                }
                break;
            case 201:
                /**
                 * Shooting the Green Ball
                 */
                Miller.indexerPower(85, Miller.rightIndexerServo);
                Miller.indexerPower(0, Miller.leftIndexerServo);
                Miller.setSorterServoPosition(sortPositionRight);
                if(pathTimer.getElapsedTimeSeconds() >= 1.0){
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    setPathState(202);
                    //Miller.setShooterPower(100);
                }
                break;
            case 202:
                /**
                 * Speeding up for the First Purple Artifact
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT) && pathTimer.getElapsedTimeSeconds() >= 2){
                    setPathState(203);
                    Miller.setStoppers(false,false);
                }
                break;
             case 203:
                 /**
                  * Shooting First Purple Artifact
                  */
                Miller.setIntakePower(70, Miller.intakeMotor);
                Miller.setSorterServoPosition(sortPositionLeft);
                Miller.indexerPower(85, Miller.leftIndexerServo);
                Miller.indexerPower(0, Miller.rightIndexerServo);
                if(pathTimer.getElapsedTimeSeconds() >= 1.05){
                    Miller.setIntakePower(0, Miller.intakeMotor);
                    //Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    setPathState(204);
                }
                break;
            case 204:
                /**
                 * Speeding up for the Second Purple Artifact
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT) ){
                    setPathState(205);
                }
                break;
            case 205:
                /**
                 * Shooting Second Purple Artifact
                 */
                Miller.setIntakePower(75, Miller.intakeMotor);
                Miller.setSorterServoPosition(sortPositionLeft);
                Miller.indexerPower(85, Miller.leftIndexerServo);
                if(pathTimer.getElapsedTimeSeconds() >= 1.25){
                    switch (currentRow){
                        case TOP_ROW:
                            Miller.setSorterServoPosition(Miller.sortPositionLeft);
                            setPathState(206);
                            break;
                        case MIDDLE_ROW:
                            Miller.setSorterServoPosition(Miller.sortPositionLeft);
                            setPathState(217);
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
                if(pathTimer.getElapsedTimeSeconds() > 0.25){
                    Miller.stopEverything();
                }
                if (pathTimer.getElapsedTimeSeconds() >= 1.25){
                    follower.followPath(aimingFirstBallTopRowPath, 0.8,true);
                    Miller.setSorterServoPosition(sortPositionRight);
                    Miller.setIntakePower(75,Miller.intakeMotor);
                    //Miller.setStoppers(true, true);
                    setPathState(207);
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
                    Miller.setIntakePower(75,Miller.intakeMotor);
                    //Miller.setStoppers(true, true);
                    setPathState(209);
                }
                break;
            case 209:
                /**
                 * Pause a little to retrieve the first Purple Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.75){
                    setPathState(210);
                }
                break;
            case 210:
                /**
                 * Advance to the second Purple Ball from Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickSecondBallTopRowPath, 0.35,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(75,Miller.intakeMotor);
                    Miller.indexerPower(-5, Miller.leftIndexerServo);
                    Miller.setStoppers(false, true);
                    setPathState(211);
                }
                break;
            case 211:
                /**
                 * Pause to retrieve the second Purple Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.75){
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    setPathState(212);
                }
                break;
            case 212:
                /**
                 * Retrieve the Green Ball From Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickThirdBallTopRowPath, 0.45,true);
                    Miller.setIntakePower(75,Miller.intakeMotor);
                    Miller.indexerPower(-10, Miller.leftIndexerServo);
                    setPathState(213);
                }
                break;
            case 213:
                /**
                 * Pause to retrieve the Green Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1.25){
                    Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.setStoppers(false, false);
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
                    Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                    setPathState(215);
                }
                break;
            case 215:
                /**
                 * Wait for shooter to speed up then shoot
                 */
                if (!follower.isBusy() && Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT)){
                    Miller.stopEverything();
                    //Miller.setStoppers(false,false);
                    Miller.setIntakePower(75, Miller.intakeMotor);
                    Miller.indexerPower(85, Miller.rightIndexerServo);
                    Miller.setSorterServoPosition(Miller.sortPositionRight);
                    setPathState(200);
                }
                break;
            case 300:
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
                    Miller.setIntakePower(100, Miller.intakeMotor);
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