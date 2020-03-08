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
            {-65, 0.43,.0263},//CL 3/2/2020
        },
        {/*1 - 5 to 10 ft*/
            {-45, 0.52,.0263},//CL 3/2/2020
        },
        {/*2 - 10 to 12 ft*/
            {-40, 0.55,.0263},//CL 3/2/2020 
        },
        {/*3 - 12 to 14 ft*/
            {-38, 0.55,.0263},//CL 3/2/2020
        },
        {/*4 - 14 to 16 ft*/
            {-33, 0.58,.0263},//CL 3/2/2020
        }, 
        {/*5 - 16 to 18 ft*/
            {-30, 0.65,.0263},//CL 3/2/2020
        },
        {/*6 - 18 to 20 ft*/
            {-30, 0.70,.0263},
        },
        {/*7 - 20 to 22 ft */
            {-30, 0.80,.0263},
        },
        {/*8 - > 22 ft*/
            {-25, 0.85,.0263}
        }  
    };

}
