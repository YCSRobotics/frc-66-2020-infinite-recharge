/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class ShooterLookup {
    public static final double[][][] shooterLookupTable = {
        /*{[0] - Angle, [1] - Speed, [3] - FF Gain}*/
        { /*0 - Front of Goal*/
            {-60, 0.40,.0263},
        },
        {/*1*/
            {-40, 0.50,.0263},
        },
        {/*2 ~ Auto Line*/
            {-50, 0.47,.0263}, 
        },
        {/*3*/
            {-45, 0.85,.0263},
        }, 
        {/*4*/
            {-45, 0.85,.0263},
        },
        {/*5*/
            {-45, 0.95,.0263},
        },
        {/*6*/
            {-30, 0.95,.0263},
        },
        {/*7*/
            {-25, 0.75,.0263}
        }  
    };

}
