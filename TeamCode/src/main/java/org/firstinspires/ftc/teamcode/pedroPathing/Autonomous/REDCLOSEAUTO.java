package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

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
@Autonomous(name = "RedCloseAutoWorlds", group = "Worlds Reggie")
public class REDCLOSEAUTO extends OpMode {
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
    double sortPositionLeft = 0.75;
    double sortPositionMidLeft = 0.75;
    double sortPositionMidRight = 0.25;


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

    public static double yTopRowPurple = 78;
    public static double yTopRowGreen = 87;

    public static double yMidRow = 75;
    public static double yBotRowPurple = 26;
    public static double yBotRowGreen = 34;




    /**
     *
     * Paths Start here
     *
     */
    private Path Start;
    private PathChain StarttoShoot, ScorePath, aimingFirstBallTopRowPath, pickFirstBallTopRowPath, pickSecondBallTopRowPath,
            ShoottoMiddleRowStart, MiddleRowStarttoMiddleRowEnd,MiddleRowEndtoShoot2, MiddleRowEnd1toMiddleRowStart2,
            MiddleRowStart2toMiddleRowEnd2,OpenGateForward1toShoot3,TopRowEndtoShoot4, Shoot2toOpenGate1, MiddleRowStart1toMiddleRowEnd1,ShoottoMiddleRowStart1,MiddleRowEnd2toShoot2, OpenGate1toOpenGateForward1,
            pickThirdBallTopRowPath,MiddleRowEnd1toShoot2,TopRowStarttoTopRowEnd, grabPickup2,forward, scoreFromTopRowPath, scoreFromMiddleRowPath, Shoot3toTopRowStart,
            aimingFirstBallMiddleRowPath, pickFirstBallMiddleRowPath, pickSecondBallMiddleRowPath,
            pickThirdBallMiddleRowPath, leavePath, pickThirdBallMiddleRowPathPPG, scoreFromMiddleRowPathPPG, pickSecondBallMiddleRowPathPPG, Shoot2;
    private final Pose startPose = new Pose(100, 136, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose shootPose = new Pose(84.5, 104.5, Math.toRadians(52));
    private final Pose shootPosecontrolpoint = new Pose(80, 80.5, Math.toRadians(50));
    private final Pose shootPose3controlpoint = new Pose(35, 82.5, Math.toRadians(50));

    // Scoring Pose of our robot. It is facing the goal at 47 degree aScoring Pose ofngle.
    private final Pose shootPose2 = new Pose(84, 105, Math.toRadians(38.5)); // Scoring Pose of our robot. It is facing the goal at 47 degree aScoring Pose ofngle.
    private final Pose shootPose3 = new Pose(83, 113, Math.toRadians(47.5));
    private final Pose shootPose4 = new Pose(76, 120, Math.toRadians(35)); // Scoring Pose of our robot. It is facing the goal at 47 degree aScoring Pose ofngle.
    private final Pose middleRowStart1 = new Pose(79, yMidRow-9, Math.toRadians(0));
    private final Pose middleRowEnd1=new Pose(95,yMidRow-9, Math.toRadians(0));
    private final Pose middleRowStart2=new Pose(93 ,yMidRow-2.5, Math.toRadians(0));
    private final Pose middleRowEnd2=new Pose(115,yMidRow-2.5, Math.toRadians(0));
    private final Pose middleRowEnd2controlpoint =new Pose(60,yMidRow-6.5, Math.toRadians(0));

    private final Pose opengate1=new Pose(101,64.5, Math.toRadians(19.5));
    private final Pose opengateforward1=new Pose(110,64.5, Math.toRadians(26));

    private final Pose opengate1curve =new Pose(45,75);
    private final Pose topRowStart =new Pose(86,89, Math.toRadians(0) );
    private final Pose topRowEnd =new Pose(100,89, Math.toRadians(0));
    private final Pose toprowendcurve =new Pose(65,96, Math.toRadians(0));



    private int gate = 0;


    // Scoring Pose of our robot. It is facing the goal at 47 degree angle.

    //    private final Pose pickThirdBallMiddleRowPosePPG = new Pose(115, yMidRowPurple-7, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
    private final Pose leavePose = new Pose(100, 75, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
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

    //    public void buildPathsPPG() {
//
//        ///* Drive to Score from the Third ball from Middle row PPG */
//        scoreFromMiddleRowPathPPG = follower.pathBuilder()
//                .addPath(new BezierCurve(pickThirdBallMiddleRowPosePPG, scorePoseThirdControl, scorePoseThird))
//                .setLinearHeadingInterpolation(pickThirdBallMiddleRowPosePPG.getHeading(),scorePoseThird.getHeading())
//                .build();
//
//        ///* Drive to pick the Second ball from Middle row PPG */
//        pickSecondBallMiddleRowPathPPG = follower.pathBuilder()
//                .addPath(new BezierCurve(pickFirstBallMiddleRowPose, pickSecondBallMiddleRowControlPosePPG, pickSecondBallMiddleRowPosePPG))
//                .setLinearHeadingInterpolation(pickFirstBallMiddleRowPose.getHeading(), pickSecondBallMiddleRowPosePPG.getHeading())
//                .build();
//
//        /* Drive to pick the Third ball from top row */
//        pickThirdBallMiddleRowPathPPG = follower.pathBuilder()
//                .addPath(new BezierCurve(pickSecondBallMiddleRowPosePPG,pickThirdBallMiddleRowControlPosePPG, pickThirdBallMiddleRowPosePPG))
//                .setLinearHeadingInterpolation(pickSecondBallMiddleRowPosePPG.getHeading(),pickThirdBallMiddleRowPosePPG.getHeading())
//                .build();
//    }
    public void buildPaths() {
        ///* Drive to read the obelisk */
        StarttoShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();

        ///* Turn to score position */
        ShoottoMiddleRowStart1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,shootPosecontrolpoint, middleRowStart1))
                .setLinearHeadingInterpolation(shootPose.getHeading(),middleRowStart1.getHeading())
                .build();

        MiddleRowStart1toMiddleRowEnd1 = follower.pathBuilder()
                .addPath(new BezierLine(middleRowStart1, middleRowEnd1))
                .setLinearHeadingInterpolation(middleRowStart1.getHeading(),middleRowEnd1.getHeading())
                .build();
        OpenGate1toOpenGateForward1 = follower.pathBuilder()
                .addPath(new BezierLine(opengate1,opengateforward1))
                .setLinearHeadingInterpolation(opengate1.getHeading(),opengateforward1.getHeading())
                .build();

        MiddleRowEnd1toMiddleRowStart2 = follower.pathBuilder()
                .addPath(new BezierLine(middleRowEnd1, middleRowStart2))
                .setLinearHeadingInterpolation(middleRowEnd1.getHeading(),middleRowStart2.getHeading())
                .build();

        MiddleRowStart2toMiddleRowEnd2 = follower.pathBuilder()
                .addPath(new BezierLine(middleRowStart2, middleRowEnd2))
                .setLinearHeadingInterpolation(middleRowStart2.getHeading(),middleRowEnd2.getHeading())
                .build();

        MiddleRowEnd2toShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(middleRowEnd2,middleRowEnd2controlpoint, shootPose2))
                .setLinearHeadingInterpolation(middleRowEnd2.getHeading(),shootPose2.getHeading())
                .build();
        Shoot2toOpenGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2,opengate1curve,opengate1))
                .setLinearHeadingInterpolation(shootPose2.getHeading(),opengate1.getHeading())
                .build();

        OpenGate1toOpenGateForward1 = follower.pathBuilder()
                .addPath(new BezierLine(opengate1,opengateforward1))
                .setLinearHeadingInterpolation(opengate1.getHeading(),opengateforward1.getHeading())
                .build();
        MiddleRowEnd1toShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(middleRowEnd1,middleRowEnd2controlpoint, shootPose2))
                .setLinearHeadingInterpolation(middleRowEnd1.getHeading(),shootPose2.getHeading())
                .build();
        OpenGateForward1toShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(opengateforward1,shootPose3controlpoint, shootPose3))
                .setLinearHeadingInterpolation(opengateforward1.getHeading(),shootPose3.getHeading())
                .build();
        Shoot3toTopRowStart = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3,topRowStart))
                .setLinearHeadingInterpolation(shootPose3.getHeading(),topRowStart.getHeading())
                .build();
        TopRowStarttoTopRowEnd = follower.pathBuilder()
                .addPath(new BezierLine(topRowStart,topRowEnd))
                .setLinearHeadingInterpolation(topRowStart.getHeading(),topRowEnd.getHeading())
                .build();
        TopRowEndtoShoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(topRowEnd,toprowendcurve,shootPose4))
                .setLinearHeadingInterpolation(topRowEnd.getHeading(),shootPose4.getHeading())
                .build();



        ///* Aim to get the first ball from the top row */
//
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //This is following the path to begin shooting, while also having our flywheels on
                follower.followPath(StarttoShoot, 0.9, true);
                setPathState(1);
                Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                //Miller.shootClose();

                break;

            case 1:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    //our indexers turn on
                    Miller.setStoppers(false, false);
                    Miller.setIntakePower(20);
                    Miller.indexerPower(60, Miller.rightIndexerServo);
                    Miller.indexerPower(60, Miller.leftIndexerServo);
                    //Miller.setShooterVelocity(Miller.TARGET_VELOCITY);
                    Miller.setStoppers(false, false);
                    setPathState(2);

                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.85) {
                    follower.followPath(ShoottoMiddleRowStart1, true);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    Miller.setStoppers(true, true);
                    setPathState(3);
                }

                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    follower.followPath(MiddleRowStart1toMiddleRowEnd1, 0.35, true);
                    setPathState(6);

                }
                break;


            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.75) {
                    follower.followPath(MiddleRowEnd1toShoot2, 0.9, true);
                    setPathState(7);


                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.75) {
                    Miller.indexerPower(60, Miller.rightIndexerServo);
                    Miller.indexerPower(60, Miller.leftIndexerServo);
                    Miller.setStoppers(false, false);

                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0 && gate < 1) {
                    follower.followPath(Shoot2toOpenGate1);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    Miller.setStoppers(true, true);
                    gate = gate + 1;


                    setPathState(9);
                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.25 && gate == 1) {
                    setPathState(12);

                }
                break;
            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.25 || pathTimer.getElapsedTimeSeconds() >= 5) {
                    follower.followPath(OpenGate1toOpenGateForward1, 0.4, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.75) {
                    follower.followPath(OpenGateForward1toShoot3, 0.9, true);
                    setPathState(11);

                }
                break;

            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    Miller.indexerPower(60, Miller.rightIndexerServo);
                    Miller.indexerPower(60, Miller.leftIndexerServo);
                    Miller.setStoppers(false, false);
                    setPathState(8);


                }
                break;
            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    follower.followPath(Shoot3toTopRowStart, true);
                    setPathState(13);
                    Miller.setStoppers(true, true);


                }
                break;

            case 13:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    follower.followPath(TopRowStarttoTopRowEnd, 0.5, true);
                    setPathState(14);

                }
                break;

            case 14:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    follower.followPath(TopRowEndtoShoot4, 0.7, true);
                    setPathState(15);

                }
                break;

            case 15:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    Miller.indexerPower(60, Miller.rightIndexerServo);
                    Miller.indexerPower(60, Miller.leftIndexerServo);
                    Miller.setStoppers(false, false);
                    setPathState(16);


                }
                break;

            case 16:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    requestOpModeStop();


                }
                break;


            case 100:
                //PPG
                if(!follower.isBusy() && Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    Miller.setStoppers(true,false);

                    setPathState(101);
                }
                break;
            case 101:
                /**
                 * Shooting the Purple Balls
                 */
                if(pathTimer.getElapsedTimeSeconds() >= 0.25 && pathTimer.getElapsedTimeSeconds()<0.55){
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    Miller.indexerPower(35, Miller.leftIndexerServo);
                }
                if(pathTimer.getElapsedTimeSeconds()>0.55 && pathTimer.getElapsedTimeSeconds()<1.0){
                    Miller.setIntakePower(55);
                    Miller.setSorterServoPosition(sortPositionLeft);
                }

                if(pathTimer.getElapsedTimeSeconds() >= 1.10){
                    Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                    setPathState(102);
                }
                break;
            //Former 32 State
            case 102:
                /**
                 * Shooting Green Artifact
                 */
                if(Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT) && pathTimer.getElapsedTimeSeconds()>0.25){
                    setPathState(103);
                    Miller.setStoppers(false,false);
                    Miller.setIntakePower(0);
                    Miller.setSorterServoPosition(sortPositionRight);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    Miller.indexerPower(60, Miller.rightIndexerServo);
                }
                break;
            case 103:
                if (pathTimer.getElapsedTimeSeconds() >= 1.25){

                    switch (currentRow){
                        case TOP_ROW:
                            follower.followPath(aimingFirstBallTopRowPath, 0.85,true);
                            Miller.setSorterServoPosition(sortPositionMiddle);
                            Miller.setIntakePower(75);
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
                    Miller.setSorterServoPosition(sortPositionMidRight);
                    Miller.setIntakePower(85);
                    //Miller.setStoppers(true, true);
                    setPathState(105);
                }

                break;
            case 105:
                /**
                 * Pause a little to retrieve the first Purple Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.35){
                    setPathState(106);
                }
                break;
            case 106:
                /**
                 * Advance to the second Purple Ball from Top Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickSecondBallTopRowPath, 0.3,true);
                    Miller.setSorterServoPosition(sortPositionMiddle);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-5, Miller.leftIndexerServo);
                    Miller.setStoppers(true, true);
                    setPathState(107);
                }
                break;
            case 107:
                /**
                 * Retrieve the Green Ball From Top Row
                 */
                if (!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    follower.followPath(pickThirdBallTopRowPath, 0.35,true);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-10, Miller.leftIndexerServo);
                    if(motif==2){
                        Miller.setShooterVelocity(Reggie.SIDES.RIGHT);
                    }else{
                        Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    }
                    setPathState(108);
                }
                break;
            case 108:
                /**
                 * Pause to retrieve the Green Ball from Top Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1.0){
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
                if(motif==2){
                    if (!follower.isBusy() && Miller.targetVelocityAcquired(Reggie.SIDES.RIGHT)){
                        Miller.setIntakePower(75);
                        Miller.indexerPower(60, Miller.rightIndexerServo);
                        currentRow = Rows.MIDDLE_ROW;
                        setPathState(200);
                    }

                }else{
                    if (!follower.isBusy() && Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                        Miller.setIntakePower(75);
                        Miller.indexerPower(85, Miller.leftIndexerServo);
                        currentRow = Rows.MIDDLE_ROW;
                        setPathState(100);
                    }
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
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.55){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    setPathState(114);
                    //follower.setMaxPower(.5);
                }
                break;
            case 114:
                /**
                 * Advance to the Green Ball from Middle Row
                 */
                if (!follower.isBusy()){
                    follower.followPath(pickSecondBallMiddleRowPath, 0.4,true);
//                    follower.followPath(pickSecondBallMiddleRowPathPPG, 0.4,true);
                    // Miller.setSorterServoPosition(sortPositionMiddle);
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
                    //Miller.setSorterServoPosition(sortPositionMidRight);
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
                    follower.followPath(pickThirdBallMiddleRowPath, 0.3,true);
//                    follower.followPath(pickThirdBallMiddleRowPathPPG, 0.3,true);
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
                    follower.followPath(scoreFromMiddleRowPath, 0.9,true);
//                    follower.followPath(scoreFromMiddleRowPathPPG, 0.9,true);
                    //Miller.stopEverything();
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                    setPathState(119);
                }
                break;
            case 119:
                //**
                // * Wait for shooter to speed up then shoot
                // */
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
                    Miller.indexerPower(60, Miller.rightIndexerServo);
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
                    setPathState(203);
                    Miller.setStoppers(false,false);
                    Miller.setIntakePower(70);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.indexerPower(60, Miller.leftIndexerServo);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                }
                break;
            case 203:
                /**
                 * Shooting First Purple Artifact
                 */

                if(pathTimer.getElapsedTimeSeconds() >= 0.45){
                    Miller.setIntakePower(0);
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
                    Miller.setIntakePower(75);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.indexerPower(60, Miller.leftIndexerServo);
                    setPathState(205);
                }
                break;
            case 205:
                /**
                 * Shooting Second Purple Artifact
                 */

                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
                    switch (currentRow){
                        case TOP_ROW:
                            Miller.setSorterServoPosition(Miller.sortPositionLeft);
                            setPathState(206);
                            break;
                        case MIDDLE_ROW:
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
                    Miller.indexerPower(60, Miller.rightIndexerServo);
                    //Miller.setSorterServoPosition(Miller.sortPositionRight);
                    currentRow = Rows.MIDDLE_ROW;
                    setPathState(200);
                }
                break;
            case 216:
                if (pathTimer.getElapsedTimeSeconds() >= 0.10){
                    Miller.stopEverything();
                    follower.followPath(aimingFirstBallMiddleRowPath, 0.8,true);
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
                    follower.followPath(pickFirstBallMiddleRowPath, 0.3,true);
                    Miller.setSorterServoPosition(sortPositionMidRight);
                    Miller.setIntakePower(85);
                    //Miller.setStoppers(true, true);
                    setPathState(219);
                }
                break;
            case 219:
                /**
                 * Pause a little to retrieve the first Purple Ball from Middle Row
                 */
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
                    Miller.setSorterServoPosition(sortPositionMidLeft);
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
                    //Miller.setSorterServoPosition(sortPositionMidLeft);
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    setPathState(222);
                }
                break;
            case 222:
                /**
                 * Retrieve the Second Purple Ball From Middle Row
                 */
                if (!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionMidRight);
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
                    Miller.stopEverything();
                    setPathState(230);
                }
                break;
            case 230:
                /**
                 * Shooting Purple from Left shooter
                 */
                if(!follower.isBusy()){
                    requestOpModeStop();
                }
                break;
            case 300:
                //PGP
                //Waiting for shooter to speed up
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    Miller.setStoppers(true, false);
                    Miller.indexerPower(60, Miller.leftIndexerServo);
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
                    Miller.indexerPower(60, Miller.rightIndexerServo);
                    Miller.indexerPower(0, Miller.leftIndexerServo);
                    setPathState(303);
                }
                break;
            case 303:

                if (pathTimer.getElapsedTimeSeconds() >= 1.05) {
                    setPathState(304);
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    Miller.setIntakePower(75);
                }
                break;
            case 304:
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)){
                    Miller.setStoppers(false, false);
                    Miller.setIntakePower(75);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.indexerPower(0, Miller.rightIndexerServo);
                    Miller.indexerPower(60, Miller.leftIndexerServo);
                    setPathState(305);
                }
                break;
            case 305:
                /**
                 * Shooting Second Purple Artifact
                 */

                if(pathTimer.getElapsedTimeSeconds() >= 0.5){
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
                    follower.followPath(aimingFirstBallTopRowPath, 0.85,true);
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
                    //Miller.setSorterServoPosition(Miller.sortPositionRight);
                    currentRow = Rows.MIDDLE_ROW;
                    setPathState(300);
                }
                break;
            case 315:
                if (pathTimer.getElapsedTimeSeconds() >= 0.10){
                    Miller.stopEverything();
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
                    Miller.setSorterServoPosition(sortPositionMidLeft);
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
                    //Miller.setSorterServoPosition(sortPositionLeft);
                    Miller.setIntakePower(85);
                    //Miller.indexerPower(-5, Miller.leftIndexerServo);
                    //Miller.setStoppers(true, true);
                    setPathState(320);
                }
                break;
            case 320:
                /**
                 * Pause to retrieve the second Green Ball from Middle Row
                 */
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.15){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
                    Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                    setPathState(321);
                }
                break;
            case 321:
                /**
                 * Retrieve the Second Purple Ball From Middle Row
                 */
                if (!follower.isBusy()){
                    Miller.setSorterServoPosition(sortPositionMidLeft);
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
                    Miller.indexerPower(60, Miller.leftIndexerServo);
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
                    Miller.indexerPower(60, Miller.rightIndexerServo);
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
                    Miller.setIntakePower(75);
                    Miller.indexerPower(60, Miller.rightIndexerServo);
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
                    Miller.indexerPower(60, Miller.leftIndexerServo);
                    Miller.indexerPower(60, Miller.rightIndexerServo);
                    Miller.setSorterServoPosition(sortPositionLeft);
                    setPathState(328);
                }
                break;
            case 328:
                /**
                 * Shooting Purple from Left shooter
                 */
                if(pathTimer.getElapsedTimeSeconds()>0.25){
                    follower.followPath(leavePath, 0.8,true);
                    Miller.stopEverything();
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
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooter power Left Error", ((Miller.leftShooterMotor.getVelocity() + Miller.rightShooterMotor.getVelocity())/2) - Miller.TARGET_MIN_VELOCITY_LEFT);
        telemetry.addData("shooter power Right Error", ((Miller.leftShooterMotor.getVelocity() + Miller.rightShooterMotor.getVelocity())/2) - Miller.TARGET_MIN_VELOCITY_RIGHT );
        telemetry.addData("Shooter velocity:", (Miller.rightShooterMotor.getVelocity() + Miller.leftShooterMotor.getVelocity()) / 2);
        telemetry.update();

    }




}
