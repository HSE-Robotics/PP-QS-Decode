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
@Autonomous(name = "BlueFarAutoWorlds", group = "Worlds Reggie")
public class BLUEFARAUTO extends OpMode {
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
    private PathChain StarttoShoot, ShoottoPickup, ShoottoLeave, PickuptoShoot;
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose shootPose = new Pose(60, 24, Math.toRadians(117));
    private final Pose pickup = new Pose(18, 26,Math.toRadians(193)); // Scoring Pose of our robot. It is facing the goal at 47 degree aScoring Pose ofngle.
    private final Pose pickupControl = new Pose(40, 29);
    private final Pose leavingPose = new Pose(18, 26, Math.toRadians(190)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    private final Pose shootPosecontrolpoint = new Pose(50, 86.5, Math.toRadians(130));
    private final Pose shootPose3controlpoint = new Pose(57, 86.5, Math.toRadians(130));

    // Scoring Pose of our robot. It is facing the goal at 47 degree aScoring Pose ofngle.
    private final Pose shootPose2 = new Pose(49, 105, Math.toRadians(130)); // Scoring Pose of our robot. It is facing the goal at 47 degree aScoring Pose ofngle.
    private final Pose shootPose3 = new Pose(50, 113, Math.toRadians(134));
    private final Pose shootPose4 = new Pose(57, 120, Math.toRadians(142)); // Scoring Pose of our robot. It is facing the goal at 47 degree aScoring Pose ofngle.



    private int gate = 0;
    private int intake = 0;


    // Scoring Pose of our robot. It is facing the goal at 47 degree angle.

    //    private final Pose pickThirdBallMiddleRowPosePPG = new Pose(115, yMidRowPurple-7, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at 47 degree angle.
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

        ShoottoPickup = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupControl, pickup))
                .setConstantHeadingInterpolation(pickup.getHeading())
                .build();

        PickuptoShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickup, shootPose))
                .setLinearHeadingInterpolation(pickup.getHeading(),shootPose.getHeading())
                .build();

        ShoottoLeave = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavingPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),leavingPose.getHeading())
                .build();

        ///* Turn to score position */



        ///* Aim to get the first ball from the top row */
//
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //This is following the path to begin shooting, while also having our flywheels on
                follower.followPath(StarttoShoot, 0.9, true);
                Miller.setStoppers(true, true);
                Miller.TARGET_MIN_VELOCITY_LEFT = 1300;
                Miller.TARGET_VELOCITY_LEFT = 1400;
                Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                setPathState(1);

                break;
            case 1:
                //This is following the path to begin shooting, while also having our flywheels on
                if(Miller.targetVelocityAcquired(Reggie.SIDES.LEFT)) {
                    Miller.setStoppers(false, false);
                    Miller.indexerPower(60, Miller.rightIndexerServo);
                    Miller.indexerPower(60, Miller.leftIndexerServo);
                    Miller.setIntakePower(20);
                    setPathState(2);
                }

                break;




            case 2:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 3.0 && intake < 3) {
                        follower.followPath(ShoottoPickup);
                        Miller.setIntakePower(85);
                        intake = intake + 1;
                        Miller.setStoppers(true, true);

                        setPathState(3);

                    }
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.25 && intake == 3) {
                        setPathState(5);

                    }
                    break;



            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.85) {
                    follower.followPath(PickuptoShoot, 0.8, true);

                    setPathState(4);

                }

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.85) {
                    Miller.indexerPower(60, Miller.rightIndexerServo);
                    Miller.indexerPower(60, Miller.leftIndexerServo);
                    Miller.setStoppers(false, false);
                    setPathState(2);



                }

                break;




            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.85) {
                    follower.followPath(ShoottoLeave);
                    Miller.setStoppers(true, true);
                    setPathState(6);
                }

                break;

                case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.85) {
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
