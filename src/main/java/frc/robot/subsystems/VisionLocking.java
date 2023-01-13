// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;

public class VisionLocking extends SubsystemBase {

    public enum Level {
        HIGH, BETWEEN, LOW
    }
        public enum Side {
        LEFT, MIDDLE, RIGHT
    }

    public enum Grid {
        GRID1, GRID2, GRID3
    }

    public enum PieceType {
        CONES, CUBES
    }
    private Level m_Level;
    private Side m_Side;
    private Grid m_Grid;
    private PieceType m_pieceType;



    /** Creates a new VisionLocking. */
    public VisionLocking() {
        m_pieceType = PieceType.CONES;
        m_Grid = Grid.GRID1;
        m_Side = Side.LEFT;
        m_Level = Level.HIGH;


    }


    public void setLeval(Level setTo){
        m_Level = setTo ;
    }
    public void setSide(Side setTo){
        m_Side = setTo;
    }
    public void setGrid(Grid setTo){
        m_Grid = setTo;
    }
    public void setPieceType(PieceType setTo){
        m_pieceType = setTo;
    }


    public PieceType getPieceType(){
        return m_pieceType;
    }
    public Grid setGrid(){
        return m_Grid;
    }
    public Side setSide(){
        return m_Side;
    }
    public Level getLeval(){
        return m_Level;
    }

    public Pose2d getLockedPositon(){



        //return new Pose2d();
        /*
        if(SmartDashboard.getString("Team Color", "Blue").equals("Blue")){
            if(m_Grid == Grid.GRID1){
                if(m_Side == Side.LEFT){
                    if(m_Level == Level.HIGH){
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else if(m_Side == Side.MIDDLE){
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else {
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                }

            } else if(m_Grid == Grid.GRID2){
                if(m_Side == Side.LEFT){
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else if(m_Side == Side.MIDDLE){
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else {
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                }

            } else {
                if(m_Side == Side.LEFT){
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else if(m_Side == Side.MIDDLE){
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else {
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                }

            }
        } else {
            if(m_Grid == Grid.GRID1){
                if(m_Side == Side.LEFT){
                    if(m_Level == Level.HIGH){
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else if(m_Side == Side.MIDDLE){
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else {
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                }

            } else if(m_Grid == Grid.GRID2){
                if(m_Side == Side.LEFT){
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else if(m_Side == Side.MIDDLE){
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else {
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                }

            } else {
                if(m_Side == Side.LEFT){
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else if(m_Side == Side.MIDDLE){
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                } else {
                    if(m_Level == Level.HIGH){
                        return new Pose2d();
                    } else if(m_Level == Level.BETWEEN){
                        return new Pose2d();
                    } else {
                        return new Pose2d();
                    }
                }

            }
        }
        return new Pose2d();

        */

         Pose2d position; 
         switch(m_Grid) {
            case GRID1:
            //initialize 
            position = FieldConstants.aprilTags.get(1).toPose2d()
            position.transformBy( new Transform2d(new Translation2d(0, Units.inchesToMeters(-6)), new Rotation2d(180)));
         
         case GRID2:
         //initialize 
         position = FieldConstants.aprilTags.get(2).toPose2d()
         position.transformBy( new Transform2d(new Translation2d(0, Units.inchesToMeters(-6)), new Rotation2d(180)));
      
        case GRID3:
      //initialize 
      position = FieldConstants.aprilTags.get(3).toPose2d()
      position.transformBy( new Transform2d(new Translation2d(0, Units.inchesToMeters(-6)), new Rotation2d(180)));
         }
         switch(m_Side) {
            case LEFT:
            //initialize 
        
            position.transformBy( new Transform2d(new Translation2d(-6, Units.inchesToMeters(0)), new Rotation2d(0)));
         
         case MIDDLE:
         //initialize 

         position.transformBy( new Transform2d(new Translation2d(0, Units.inchesToMeters(0)), new Rotation2d(0)));
      
        case RIGHT:
      //initialize 
 
      position.transformBy( new Transform2d(new Translation2d(6, Units.inchesToMeters(0)), new Rotation2d(0)));
         }
         return position;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
} 
