#include <fstream>
#include <ctime>
#include <iostream>

#include "CQLearning.h"
using namespace std;

using std::clock;
using std::clock_t; // CPU time


int main()
{
	clock_t Start,End;

	int i;
	int j;
	int k;
	int m;
	int t;
	int SeAction,ctrStep=0;
	int ThisState[DimensionNum];	
	int NextState[DimensionNum];
	//int action[actob]={2,0,0,0,3,3,3,0,3,0,3,3,3,3,3,0,0,0,0,0,3,3,0,3,0,3,0,3,3,3,0,0,0};//expert action
	int action[actstop]={3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
						 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	//0:up 1:down 2:left 3:right
	//int action[act60]={3,3,3,0,0,3,0,3,0,0,0,0,3,3,3,0,0,3,0,3,0,0,3,3,0,3,0,0,0,3,3,3,0,0,0,0,3,3,3,3,3,3,0,0,0,0,0,0,0,0,3,0,3,0,0,3,3,3,3,3};
	//int action[act20]={0,0,0,3,3,3,0,0,0,0,0,0,3,3,3,3,3,3,0,0,0,0,0,0,3,3,3,3,3,3,0};
	//int action[act12]={3,3,0,0,3,3,0,0,0,3,3,0};
	double Reward,error,situation=0;

	

	CQLearning m_Qlearning;
	srand((unsigned)time(NULL));
	for(t=0; t< 1; t++)
	{
		Start = clock();

	    m_Qlearning.MueReset();
		m_Qlearning.Initial_Position();//initial position 17,2 left_down     goal:2,17 right_up
		for( k = 0; k <actstop; k++ )
		{
		   m_Qlearning.GetState(ThisState);//use index of this_state to get robot's position now
		   //cout<<"Expert move to: "<<ThisState[0]<<" ,"<<ThisState[1]<<"\n";
		   m_Qlearning.GetMue(ThisState,k);
		  
		 
		   SeAction=action[k];
		   situation = m_Qlearning.TakeAction(SeAction);//hit_wall:0,hit_goal:1,no_hit:2
		   if(situation==SHitGoal)
		   {   
			   cout<<"Expert(Mue) take "<<k+1<<" step to goal"<<"\n";
			   cout<<"According to expert's action[actob] transform into Mue :finish"<<endl;
			   //m_Qlearning.terminal = false;
			   break;
		   }
		   m_Qlearning.GetState(NextState);//use index of NextState to get robot's position now
		}
		m_Qlearning.PrintfMue();

    //***********Update W**************//
	m_Qlearning.OMGReset();
	m_Qlearning.QReset();
	m_Qlearning.FReset();

	cout<<"Start to training:"<<endl;
	

	for( i = 0; i < 2000; i++ )//TrailNum:1000
	{ 
		//cout<<"Trial: "<<i<<endl;
		m_Qlearning.Mu1Reset();//Initial Mul to zero
		m_Qlearning.Initial_Position();
		m_Qlearning.GetState(ThisState);//First step
		m_Qlearning.GetMu1(ThisState,0);
		
		for( k = 1; k < actstop; k++ )
		{
			SeAction = m_Qlearning.EvluateAction(ThisState,0);//return Max q table's action
			situation = m_Qlearning.TakeAction(SeAction);
			if(situation==SHitWall){
			   m_Qlearning.GetMu1W(k);
			}
			if(situation==SHitGoal){   
			   cout<<"Mul take "<<k<<" step to goal"<<"\n";
			   m_Qlearning.terminal = false;
			   //m_Qlearning.GetMu1(ThisState,k);
			   break;
		    }
			else{
				m_Qlearning.GetState(ThisState);
				m_Qlearning.GetMu1(ThisState,k);
			}
				m_Qlearning.PrintfMul();
		//cout<<k<<"\n";
		}
		
			error=m_Qlearning.Error();//caculate square error and obtain a set of Mul
		    ofstream itfile("Error.txt", ios::app);
			itfile << error << "\n";
			itfile.close();
		cout<<"Trial: "<<i<<"  Square error-(Ue_Ul) "<<error<<endl;
		if(i ==20){
			//m_Qlearning.PrintfOmega();
		}
		if(error<0.000001)
		{
			m_Qlearning.terminal = true;
			//m_Qlearning.Initial_Position();
			/*for( k = 0; k < 31; k++ )//caculate MU1
			{
				m_Qlearning.GetState(ThisState);
			 cout<<ThisState[0]<<"   "<<ThisState[1]<<"\n";
			SeAction = m_Qlearning.EvluateAction(ThisState,0);
			Reward = m_Qlearning.TakeAction(SeAction);
			
			}*/
			m_Qlearning.PrintfOmega();
			ofstream itfile("Iter.txt", ios::app);
			//which trial is success
			itfile << i << "\n";	
			itfile.close();
			m_Qlearning.PrintfQ();
			

			//system("PAUSE");
			cout<<"Congratulations !! You succeed at "<<i<<" trial."<<endl;
			break;//break trial loop
			
		}
		else
		{
			//cout<<error<<"\n";
			m_Qlearning.Rule_without_bad();//Accordingerror to Mul and Mue to caculate for F   f:walk  fw:hit wall
			m_Qlearning.Omega();//According to F ,obtaining reward function(OMEGA)
			//m_Qlearning.QReset();//Reset Q table to zero for preparation of next Q_learning
		}
	
//*******************************RL***************************/use what the reward function what we just get throw into RL
		

		for( j = 0; j < 500; j++ )//EpisodeNum:2000 
		{
			m_Qlearning.Initial_Position();
					//cout<<j<<"\n";
			for( k = 0; k < 100; k++ )//StepNum:500
			{	
				m_Qlearning.GetState(ThisState);

				SeAction = m_Qlearning.EvluateAction(ThisState,10);//10% random

			    Reward=m_Qlearning.IRLAction(SeAction);//take action  ex: state0->state1(this reward)  inside has a index of omega

				m_Qlearning.GetState(NextState);

				m_Qlearning.UpdateValue(ThisState, NextState, SeAction, Reward);//update Q value with Q_learning

				if(m_Qlearning.terminal == true)
				{
					m_Qlearning.terminal = false;
					break;
				}
			}//step
			//cout<<"ALL trial: "<<i<<"  RL Episode: "<<j<<"  RL step: "<<k<<endl;
		}//episode
		
		
	}//iteration
    End = clock();
	cout <<"ALL running time is : "<< (End-Start)<<endl;
	
	ofstream itfile("running time.txt", ios::app);
			//which trial is success
			itfile <<End-Start << "\n";	
			itfile.close();
}//Experiment_times
 
	
    system("pause");
	return 0;
}//main

