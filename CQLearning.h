#ifndef _CQLEARNING_H
#define _CQLEARNING_H

#include <stdlib.h>

#include "CParameter.h"

class CQLearning
{
   private:
	   int agentX;                                  //initial x postion
	   int agentY;                                  //initial y position

   public:
	   double ***QTable;
	   double **OMG;
	   double **Mue;
	   double **Mu1;
	   double **F;
	   double MueW,Mu1W,OMGW,FW;
	   
	  
	   CQLearning();
		~CQLearning();
	   bool terminal;
	   void QReset();
	   void MueReset();
	   void Mu1Reset();
	   void OMGReset();
	   void FReset();
	   void Initial_Position();
	   void GetState(int *);
	   int EvluateAction(int *,int);
	   double TakeAction(int);
	   double IRLAction(int);
	   short CheckPos(int, int);
	   void UpdateValue(int *, int *, int, double);
	   void GetMue(int *,int );
	   void GetMu1(int *,int );
	   void GetMu1W(int );
	   void PrintfMue();
	   void PrintfMul();
	   //void Rule();
	   void Rule_without_bad();
	   double Error();
	   void Omega();
	   void PrintfOmega();
	   void PrintfQ();

};



#endif