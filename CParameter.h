#ifndef _CPARAMETER_H
#define _CPARAMETER_H

//training parameter
#define TrailNum 50
#define EpisodeNum 2000
#define StepNum 500
//**************

//state space
#define DimensionNum 2
#define POSX_BINS 20
#define POSY_BINS 20
#define ActionNum 4
//**********

//agent walking parameter
#define SHitWall 0                                 //flag of hit wall
#define SHitGoal 1                                 //flag of hit goal
#define SNoHit   2                                 //flag of normal state
//****************

//Q-Learning parameter
#define GAMMA 0.9
#define BETA 0.7
//*************

//Reward value
//#define GoalReward 10.0                              //Reward of hit goal
//#define WallReward -10.0                             //Reward of hit wall
//#define NormalReward -0.000001                       //Reward of normal state
//*********
#define act20 31
#define act10 12 
#define act37 60 
#define actob 33


#endif