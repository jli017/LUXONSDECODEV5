package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths2 {

    public Pose startPose;
    public PathChain ScorePre;

    public PathChain collect1;
    public PathChain openGate;
    public PathChain Score1;

    public PathChain collect2;
    public PathChain score2;

    public PathChain collect3;
    public PathChain score3;

    public PathChain collect4;
    public PathChain score4;

    public PathChain collect5;
    public PathChain score5;
    public PathChain leave;

    public Pose shootpre = new Pose(55.5, 85, Math.toRadians(180));
    public Pose intake1 = new Pose(19, 60, Math.toRadians(180));
    public Pose controlIntake1 = new Pose(73.2, 54.5);
    //public Pose opengate = new Pose(17.3, 69, Math.toRadians(270));
    public Pose shootload1 = new Pose(53, 80, Math.toRadians(180));
    public Pose collectgate1 = new Pose(13, 63, Math.toRadians(155));
    //public Pose collectcontrol1 = new Pose(52, 43);
    public Pose shootgate1 = new Pose(53, 80, Math.toRadians(180));
    public Pose collectgate2 = new Pose(13, 63.5,Math.toRadians(155));
    //public Pose collectcontrol2 = new Pose(52, 43);
    public Pose shootgate2 = new Pose(53, 80, Math.toRadians(180));
    public Pose intake2 = new Pose(20, 35, Math.toRadians(180));
    public Pose controlIntake2 = new Pose(72, 33);
    public Pose shootload2 = new Pose(53, 84, Math.toRadians(180));
    public Pose intake3 = new Pose(20, 87, Math.toRadians(180));
    public Pose shootload3 = new Pose(39, 92.5, Math.toRadians(180));
    public Pose leave1 = new Pose(26,92.5, Math.toRadians(180));





    //tangent auto
    public PathChain Scoreuno;
    public PathChain CollectLastRow;
    public PathChain ScoreLastRow;
    public PathChain CollectHP1;
    public PathChain HP1_to_HP2;
    public PathChain HP2_to_HP1;
    public PathChain HP1_to_Score;
    public PathChain CollectOffHP;
    public PathChain OffHP1_to_OffHP2;
    public PathChain OffHP2_to_OffHP1;


    public Pose shoot = new Pose(56, 8, Math.toRadians(90));
    public Pose lastrow = new Pose(18, 35.5, Math.toRadians(180));
    public Pose con1 = new Pose(72, 36);
    public Pose shoot2 = new Pose(56, 9, Math.toRadians(180));
    public Pose hp1 = new Pose(9, 9, Math.toRadians(180));
    public Pose hp2 = new Pose(16, 9, Math.toRadians(180));
    public Pose offhp1 = new Pose(10, 21, Math.toRadians(180));
    public Pose offhp2 = new Pose(15, 21, Math.toRadians(180));

    public Paths2(Follower follower, Lebruxon.Alliance alliance) {
        if (alliance == Lebruxon.Alliance.RED || alliance == Lebruxon.Alliance.REDCLOSE || alliance == Lebruxon.Alliance.REDSQ) {

            shootpre = shootpre.mirror();
            intake1 = intake1.mirror();
            controlIntake1 = controlIntake1.mirror();
            //opengate = opengate.mirror();
            shootload1 = shootload1.mirror();
            collectgate1 = collectgate1.mirror();
            //collectcontrol1 = collectcontrol1.mirror();
            shootgate1 = shootgate1.mirror();
            collectgate2 = collectgate2.mirror();
            //collectcontrol2 = collectcontrol2.mirror();
            shootgate2 = shootgate2.mirror();
            intake2 = intake2.mirror();
            controlIntake2 = controlIntake2.mirror();
            shootload2 = shootload2.mirror();
            intake3 = intake3.mirror();
            shootload3 = shootload3.mirror();
            leave1 = leave1.mirror();


            shoot = shoot.mirror();
            lastrow = lastrow.mirror();
            con1 = con1.mirror();
            shoot2 = shoot2.mirror();
            hp1 = hp1.mirror();
            hp2 = hp2.mirror();
            offhp1 = offhp1.mirror();
            offhp2 = offhp2.mirror();
        }


        startPose = Lebruxon.startPose;
       // if (alliance == Mosby.Alliance.RED || alliance == Mosby.Alliance.REDCLOSE || alliance == Mosby.Alliance.REDSQ) {


        ScorePre = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, shootpre)
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), shootpre.getHeading())
                .setBrakingStrength(1.22)
                .build();

        collect1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(shootpre, controlIntake1, intake1 )
                )
                .setLinearHeadingInterpolation(shootpre.getHeading(), intake1.getHeading())
                .setBrakingStrength(1.22)
                .build();


        Score1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intake1, shootload1)
                )
                .setLinearHeadingInterpolation(intake1.getHeading(), shootload1.getHeading())
                .setBrakingStrength(1.22)
                .build();

        collect2 = follower
                .pathBuilder() 
                .addPath(
                        new BezierLine(shootload1, collectgate1)
                )
                .setLinearHeadingInterpolation(shootload1.getHeading(), collectgate1.getHeading())
                .setBrakingStrength(1.5)
                .build();

        score2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(collectgate1, shootgate1)
                )
                .setLinearHeadingInterpolation(collectgate1.getHeading(), shootgate1.getHeading())
                .setBrakingStrength(1.22)
                .build();

        collect3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootgate1, collectgate2)
                )
                .setLinearHeadingInterpolation(shootgate1.getHeading(), collectgate2.getHeading())
                .setBrakingStrength(1.5)
                .build();

        score3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(collectgate2, shootgate2)
                )
                .setLinearHeadingInterpolation(collectgate2.getHeading(), shootgate2.getHeading())
                .setBrakingStrength(1.22)
                .build();

        collect4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(shootgate2, controlIntake2, intake2)
                )
                .setLinearHeadingInterpolation(shootgate2.getHeading(), intake2.getHeading())
                .setBrakingStrength(1.22)
                .build();

        score4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intake2, shootload2)
                )
                .setLinearHeadingInterpolation(intake2.getHeading(), shootload2.getHeading())
                .setBrakingStrength(1.22)
                .build();

        collect5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootload2, intake3)
                )
                .setLinearHeadingInterpolation(shootload2.getHeading(), intake3.getHeading())
                .setBrakingStrength(1.22)
                .build();

        score5= follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intake3, shootload3)
                )
                .setLinearHeadingInterpolation(intake3.getHeading(), shootload3.getHeading())
                .setBrakingStrength(1.22)
                .build();

        leave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootload3, leave1)
                )
                .setLinearHeadingInterpolation(shootload3.getHeading(), leave1.getHeading())
                .setBrakingStrength(1.22)
                .build();








            // tangent
            Scoreuno = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shoot))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shoot.getHeading())
                    .build();


            CollectLastRow = follower.pathBuilder()
                    .addPath(new BezierCurve(shoot, con1, lastrow))
                    .setLinearHeadingInterpolation(shoot.getHeading(), lastrow.getHeading())
                    .build();


            ScoreLastRow = follower.pathBuilder()
                    .addPath(new BezierLine(lastrow, shoot2))
                    .setLinearHeadingInterpolation(lastrow.getHeading(), shoot2.getHeading())
                    .build();


            CollectHP1 = follower.pathBuilder()
                    .addPath(new BezierLine(shoot2, hp1))
                    .setLinearHeadingInterpolation(shoot2.getHeading(), hp1.getHeading())
                    .build();

            HP1_to_HP2 = follower.pathBuilder()
                    .addPath(new BezierLine(hp1, hp2))
                    .setLinearHeadingInterpolation(hp1.getHeading(), hp2.getHeading())
                    .build();

            HP2_to_HP1 = follower.pathBuilder()
                    .addPath(new BezierLine(hp2, hp1))
                    .setLinearHeadingInterpolation(hp2.getHeading(), hp1.getHeading())
                    .build();

            HP1_to_Score = follower.pathBuilder()
                    .addPath(new BezierLine(hp1, shoot2))
                    .setLinearHeadingInterpolation(hp1.getHeading(), shoot2.getHeading())
                    .build();

            CollectOffHP = follower.pathBuilder()
                    .addPath(new BezierLine(shoot2, offhp1))
                    .setLinearHeadingInterpolation(shoot2.getHeading(), offhp1.getHeading())
                    .build();

            OffHP1_to_OffHP2 = follower.pathBuilder()
                    .addPath(new BezierLine(offhp1, offhp2))
                    .setLinearHeadingInterpolation(offhp1.getHeading(), offhp2.getHeading())
                    .build();

            OffHP2_to_OffHP1 = follower.pathBuilder()
                    .addPath(new BezierLine(offhp2, offhp1))
                    .setLinearHeadingInterpolation(offhp2.getHeading(), offhp1.getHeading())
                    .build();
        //}

    }

}