// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import edu.wpi.first.math.util.Units;

public class VisionLocking extends SubsystemBase {

    public enum Team{
        RED, BLUE
    }
    public enum Level {
        HIGH, MID, LOW
    }
    public enum Side {
        LEFT, RIGHT
    }

    public enum PieceType {
        CONES, CUBES
    }
    private Team m_team;
    private Level m_level;
    private Side m_side;
    private int m_grid;
    private PieceType m_pieceType;
    private final int[] blueTags = {8,7,6};
    private final int[] redTags = {3,2,1};







    /** Creates a new VisionLocking. */
    public VisionLocking() {
        m_team = Team.BLUE;
        m_pieceType = PieceType.CONES;
        m_grid = 1;
        m_side = Side.LEFT;
        m_level = Level.HIGH;


    }

    public void setTeam(Team setTo){
        m_team = setTo;
    }
    public void setLevel(Level setTo){
        m_level = setTo ;
    }
    public void setSide(Side setTo){
        m_side = setTo;
    }
    public void setGrid(int setTo){
        m_grid = setTo;
    }
    public void setPieceType(PieceType setTo){
        m_pieceType = setTo;
    }

    public Team getTeam(){
        return m_team;
    }
    public PieceType getPieceType(){
        return m_pieceType;
    }
    public int setGrid(){
        return m_grid;
    }
    public Side setSide(){
        return m_side;
    }
    public Level getLevel(){
        return m_level;
    }
    public void toggleGrid(){
        if (m_grid < 3) {
            m_grid += 1;
        }else {
            m_grid = 1;
        }
    }


    public void toggleLevel(){
        if (m_level.equals(Level.LOW)){
            m_level = Level.MID;
        } else if(m_level.equals(Level.MID)){
            m_level = Level.HIGH;
        } else {
            m_level = Level.LOW;
        }
    }

    public void toggleSide(){
        if (m_side.equals(Side.LEFT)) {
            m_side = Side.RIGHT;
        } else {
            m_side = Side.LEFT;
        }
    }
    public void togglePiece(){
        if (m_pieceType.equals(PieceType.CONES)) {
            m_pieceType = PieceType.CUBES;
        } else {
            m_pieceType = PieceType.CONES;
        }
    }

    /**
     *
     * Returns the position the robot must be within a certain degree of error of to score on the Grid
     *
     * TODO fix to apply for both alliance colors
     *
     * TODO Add more shit for substation loading
     *
     * @return position
     */

    public Pose2d getLockedPosition() {
        Pose2d position;

        if (m_team.equals(Team.BLUE)){
            position = FieldConstants.aprilTags.get(blueTags[m_grid - 1]).toPose2d();
            position.transformBy(new Transform2d(new Translation2d(Units.inchesToMeters(50), Units.inchesToMeters(0)),new Rotation2d()));
        } else {
            position = FieldConstants.aprilTags.get(redTags[m_grid - 1]).toPose2d();
            position.transformBy(new Transform2d(new Translation2d(Units.inchesToMeters(0),0),new Rotation2d()));
        }
        //return position;
        return new Pose2d(new Translation2d(2.1, 1), new Rotation2d(179));
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
