package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Reggie;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;



/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 02/11/2024
 */
@Autonomous(name = "Reggie_Auto", group = "LM1 Reggie")
public class ReggieAuto_v1 extends OpMode {
    //private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
    //Pedro Pathing Variables
    private Follower follower;
    private Reggie Miller;
    public static Pose startingPose;
    public int motif;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Path Start;
    private PathChain ReadPos, Score, grabPickup2;
    private final Pose startPose = new Pose(110, 122, Math.toRadians(135)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(86, 95, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose1 = new Pose(84, 90, Math.toRadians(90)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(86, 85, Math.toRadians(50)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1 = new Pose(90, 71, Math.toRadians(60)); // Highest (First Set) of Artifacts from the Spike Mark.






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
        motif = order(Miller.huskyLens);

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
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(.7);
                follower.followPath(ReadPos,true);
                setPathState(11);

                //Miller.shootClose();
                if(pathTimer.getElapsedTimeSeconds() >= 6){

                    break;
                }
            case 11:
                //motif = order(Miller.huskyLens);
                if(pathTimer.getElapsedTimeSeconds() >= 2){

                    setPathState(1);
                }
                break;
            case 1:
                follower.setMaxPower(1);

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Score,true);
                    Miller.setShooterPower(60);
                    if(pathTimer.getElapsedTimeSeconds() >=3) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy() ){
                    if ( motif == 1) {

                    Miller.indexerPower(65, Miller.leftIndexerServo);
                    Miller.setIntakePower(100,Miller.intakeMotor);
                    Miller.setSorterServoPosition(1);
                    if(pathTimer.getElapsedTimeSeconds() >= 3){
                        Miller.setSorterServoPosition(0);
                        Miller.indexerPower(80, Miller.rightIndexerServo);
                            if (pathTimer.getElapsedTimeSeconds() >= 7){
                                Miller.indexerPower(0, Miller.rightIndexerServo);
                                Miller.indexerPower(0, Miller.leftIndexerServo);
                                Miller.setShooterPower(0);
                                Miller.setIntakePower(0,Miller.intakeMotor);
                            }
                        }
                    }
//                    Miller.shootClose();
                }
                if (pathTimer.getElapsedTimeSeconds() >= 15) {
                    break;
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