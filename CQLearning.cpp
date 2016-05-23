#include <fstream>
#include <iostream>
#include <math.h>
#include "CQLearning.h"
#include "CMaze20.h"
#include <iomanip> //setprecision library
using namespace std;
ofstream mfile("Mue.txt", ios::out);

CQLearning::CQLearning()
{
	int i;
	int j;

	QTable = new double **[POSX_BINS];
	for(i = 0; i < POSX_BINS; i++)
	{
	    QTable[i] = new double *[POSY_BINS];
		
		for(j = 0; j < POSY_BINS; j++)
		{
			QTable[i][j] = new double [ActionNum];
				
		}
	}	

	Mue = new double *[POSX_BINS];
	Mu1 = new double *[POSX_BINS];
	OMG = new double *[POSX_BINS];
	F = new double *[POSX_BINS];
	for(i = 0; i < POSX_BINS; i++)
	{
	     Mue[i] = new double [POSY_BINS];
		 Mu1[i] = new double [POSY_BINS];
		 OMG[i] = new double [POSY_BINS];
		   F[i] = new double [POSY_BINS];
	}

}
CQLearning::~CQLearning()
{ 
	int i;
	int j;

	for(i = 0; i < POSX_BINS; i++)
	{	
		for(j = 0; j < POSY_BINS; j++)
		{	
			delete [] QTable[i][j];
		}
		delete [] QTable[i];
	}
	delete [] QTable;


	for(i = 0; i < POSX_BINS; i++)
	{	
		delete [] Mue[i];
		delete [] Mu1[i];
		delete [] OMG[i];
		delete [] F[i];
	}
	delete [] Mue;
	delete [] Mu1;
	delete [] OMG;
	delete [] F;


}
void CQLearning::QReset()
{
	int i;
	int j;
	int k;

	for(i = 0; i < POSX_BINS; i++)
	{
		for(j = 0 ; j < POSY_BINS; j++)
		{
			for(k = 0 ; k < ActionNum; k++)
			{
				QTable[i][j][k] = 0.0;
			}
		}
	}
	terminal = false;
}
void CQLearning::Initial_Position()
{
	agentX = 17;
	agentY = 2;
}
void CQLearning::GetState(int *State)
{
	State[0] = agentX;
	State[1] = agentY;
}
int CQLearning::EvluateAction(int *State,int Epsilon)
{
	int Action;
	short Random;
	double MaxQ;
	int i;
	Random = 1 + rand() % 100;//1~100
	
	Action = rand() % ActionNum;//0~3
	MaxQ = QTable[State[0]][State[1]][Action];
	//cout<<"MaxQ:" <<MaxQ<<endl;
	if(Random <= Epsilon)
	{
		Action = rand() % ActionNum;
	}
	else
	{
		for(i = 0; i < ActionNum; i++)
		{
			if(QTable[State[0]][State[1]][i] > MaxQ)
			{
				MaxQ = QTable[State[0]][State[1]][i];
				Action = i;
			}
		}
	}

	return Action;
}
double CQLearning::TakeAction(int Action)
{
	int agentXtemp;    //store temp x position
	int agentYtemp;    //store temp y position

	short Result;   

	switch(Action)
	{
	   case 0:
		   agentXtemp = agentX-1;
		   agentYtemp = agentY;
		   break;

	   case 1:
		   agentXtemp = agentX+1;
		   agentYtemp = agentY;
		   break;

	   case 2:
		   agentXtemp = agentX;
		   agentYtemp = agentY-1;
		   break;

	   case 3:
		   agentXtemp = agentX;
		   agentYtemp = agentY+1;
		   break;

	   default:
		   break;
	}

	Result = CheckPos(agentXtemp, agentYtemp);
	if(Result==SNoHit||Result==SHitGoal)
	{
		agentX = agentXtemp;                          
		agentY = agentYtemp;
	}
	return Result;
}
double CQLearning::IRLAction(int Action)
{
	int agentXtemp;    //store temp x position
	int agentYtemp;    //store temp y position

	short Result;   

	switch(Action)
	{
	   case 0://up
		   agentXtemp = agentX-1;
		   agentYtemp = agentY;
		   break;

	   case 1://down
		   agentXtemp = agentX+1;
		   agentYtemp = agentY;
		   break;

	   case 2://left
		   agentXtemp = agentX;
		   agentYtemp = agentY-1;
		   break;

	   case 3://right
		   agentXtemp = agentX;
		   agentYtemp = agentY+1;
		   break;

	   default:
		   break;
	}		

		/*if(agentXtemp<=0)
		agentXtemp=1;
		else if(agentXtemp>=(POSX_BINS-1))
		agentXtemp=(POSX_BINS-2);
		if(agentYtemp=<0)
		agentYtemp=1;
		else if(agentYtemp>=(POSY_BINS-1))
		agentYtemp=(POSY_BINS-2);*/

	Result = CheckPos(agentXtemp, agentYtemp);

	if(Result==SHitWall)
	{
		return OMGW;
	}
	else if(Result==SHitGoal)
	{
		terminal = true;
		agentX = agentXtemp;                          
		agentY = agentYtemp;
	}
	else
	{
		agentX = agentXtemp;                          
		agentY = agentYtemp;
	}
		
	return OMG[agentX][agentY];

}
short CQLearning::CheckPos(int x, int y)  
{ 
//-1:Wall or obstacle
// 0:free space
// 1:start point
// 2:Goal


	//whether hit wall or goal
	if(Maze[x][y] == -1)        
	{
		return SHitWall;//return 0
	}
	else if(Maze[x][y] == 2)
	{
		return SHitGoal;//return 1
	}

	 //Normal situation
	return SNoHit;//return 2
}
void CQLearning::UpdateValue( int *ThisState, int *NextState, int Action, double Reward )
{
	double MaxQ;
	double OldQ;
	double TempQ;
	int i;

	OldQ = QTable[ThisState[0]][ThisState[1]][Action];
	MaxQ = QTable[NextState[0]][NextState[1]][0];

	for(i = 0; i < ActionNum; i++)
	{
 		if(QTable[NextState[0]][NextState[1]][i] > MaxQ)
		{
			MaxQ = QTable[NextState[0]][NextState[1]][i];
		}
	}

	TempQ = OldQ + (BETA * (Reward + (GAMMA*MaxQ - OldQ)));

	QTable[ThisState[0]][ThisState[1]][Action] = TempQ;
}
void CQLearning::PrintfQ()
{
	int i,j,k;
	 
	double MaxQ ;
	ofstream Qfile("Q.csv", ios::out);	
			
	for(i = 0; i < POSX_BINS; i++)
	{
		for(j = 0 ; j < POSY_BINS; j++)
		{
			MaxQ=QTable[i][j][0];
			for(k = 0 ; k < ActionNum; k++)
			{		
				if(QTable[i][j][k]>MaxQ)
					MaxQ=QTable[i][j][k];
				
			}
			Qfile <<MaxQ<<",";
		}Qfile<< "\n";
	}
	Qfile.close();
}




/*******************************/
void CQLearning::Rule()
{
	int i;
	int j;
	double a=0.1;
	int correct[POSX_BINS][POSY_BINS]={0};
    for(i=0;i<POSX_BINS;i++)
	{
      for(j=0;j<POSY_BINS;j++)
	  {
        if(Mue[i][j]==Mu1[i][j]){
			correct[i][j]=1;
        }
		else{
			if(Mue[i][j]==0&&Mu1[i][j]>0&&Bad[i][j]==1)
				correct[i][j]=2;
			else
			correct[i][j]=0;
		}
	  }
	}

	for(i=0;i<POSX_BINS;i++)
	{
      for(j=0;j<POSY_BINS;j++)
	  {
        if(correct[i][j]==0){//wrong
            F[i][j]=F[i][j]+(1-F[i][j])*a;
        }else if(correct[i][j]==2){
			F[i][j]=1;
		}
		else{
            F[i][j]=(1-a)*F[i][j];
        }
	  }
	}
	if(MueW==Mu1W)
		FW=FW*(1-a);
	else if(MueW==0&&Mu1W>0)
		FW=1;
	else
	FW=FW+(1-FW)*a;

}
double CQLearning::Error()
{
	int i;
	int j;
	double error=0;
	for(i=0;i<POSX_BINS;i++)
	{
      for(j=0;j<POSY_BINS;j++)
	  {
		error=error+pow(Mue[i][j]-Mu1[i][j],2);
	  }
	}
	error=error+pow(MueW-Mu1W,2);
	error=pow(error,0.5);
    //cout<<"error"<<error<< "\n";
	//ofstream efile("error.txt", ios::app);
	//efile << error << "\n";	
	//efile.close();

	return error;
}
void CQLearning::Omega()
{
	int i;
	int j;
	
	for(i=0;i<POSX_BINS;i++)
	{
      for(j=0;j<POSY_BINS;j++)
	  {
			  OMG[i][j]=OMG[i][j]+(Mue[i][j]-Mu1[i][j])*F[i][j];
	  } 
	}
	OMGW=OMGW+(MueW-Mu1W)*FW;
}
void CQLearning::PrintfOmega()
{
	int i;
	int j;
	ofstream Ofile("OMG.csv", ios::out);
	for(i=0;i<POSX_BINS;i++)
	{
      for(j=0;j<POSY_BINS;j++)
	  {
			 if(Maze[i][j]==-1)
				  Ofile <<OMGW<<",";
			 else
			  Ofile <<OMG[i][j]<<",";
	  } 
	  Ofile <<"\n";
	}
	Ofile.close();
}
void CQLearning::MueReset()
{
	int i;
	int j;
	for(i = 0; i < POSX_BINS; i++)
	{
		for(j = 0 ; j < POSY_BINS; j++)
		{		
			Mue[i][j] = 0.0;
		}
	}
  MueW=0.0;
}
void CQLearning::Mu1Reset()
{
	int i;
	int j;
	for(i = 0; i < POSX_BINS; i++)
	{
		for(j = 0 ; j < POSY_BINS; j++)
		{		
			Mu1[i][j] = 0.0;
		}
	}
	Mu1W=0.0;
}
void CQLearning::OMGReset()
{
	int i;
	int j;
	for(i = 0; i < POSX_BINS; i++)
	{
		for(j = 0 ; j < POSY_BINS; j++)
		{		
			OMG[i][j] = 0.0;
		}
	}
	OMGW=0.0;
}
void CQLearning::FReset()
{
	int i;
	int j;
	for(i = 0; i < POSX_BINS; i++)
	{
		for(j = 0 ; j < POSY_BINS; j++)
		{		
			F[i][j] = 0.9;
		}
	}
	FW=0.9;
}
void CQLearning::GetMue(int *State,int t)
{
	Mue[State[0]][State[1]]=Mue[State[0]][State[1]]+pow(GAMMA,t);
	
}
void CQLearning::GetMu1(int *State,int t)
{
	Mu1[State[0]][State[1]]=Mu1[State[0]][State[1]]+pow(GAMMA,t);
	//cout<<"Mu1["<<State[0]<<"]["<<State[1]<<"]="<<Mu1[State[0]][State[1]]<<"\n";
}
void CQLearning::GetMu1W(int t)
{
	Mu1W=Mu1W+pow(GAMMA,t);
}
void CQLearning::PrintfMue()
{
	int i;
	int j;
	fstream mfile("Mue.txt", ios::out);
	for( i = 0; i < POSX_BINS; i++ )
	{
		for( j = 0; j < POSY_BINS; j++ )
		{
			//mfile << Mue[i][j] << ",";
			mfile << fixed << setprecision(2) << Mue[i][j] << ",  ";
		}
		mfile << "\n";
	}
	mfile.close();
	
}

