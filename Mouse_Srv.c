#include "Mouse_Srv.h"
#include "stdio.h"

#define MAZETYPE        16                                              /*  8: 四分之一迷宫；16: 全迷宫 */

extern uint8    GucMapStep[MAZETYPE][MAZETYPE];           /*  保存各坐标的等高值          */
extern uint8    GucXStart;                /*  起点横坐标                  */
extern uint8    GucYStart;                /*  起点纵坐标                  */
extern uint8    GucMapBlock[MAZETYPE][MAZETYPE];                        /*  GucMapBlock[x][y]           */

unsigned char 	MapDiagonalway[16][16]={0xff};
unsigned char   DiagonalwayCount=0;
unsigned char	DiagonalwayAction[255]={0};
unsigned char actionsequence[500]={0};
extern uint8 STRAIGHT90L;
extern uint8 LV45L;
extern uint8 LT45L;
extern uint8 LV90L;
extern uint8 LV135L;
extern uint8 RV45L;
extern uint8 RT45L;
extern uint8 RV90L;
extern uint8 RV135L;
extern uint8 RT135L;
extern uint8 STRAIGHT45L;
extern uint8 L180L;
extern uint8 R180L;
extern uint8 RT90L;
extern uint8 LT90L;
extern uint8 LT135L;
extern uint8 hh;

void Diagonalway(unsigned char cXdst,unsigned char cYdst)
{
	unsigned char i,j,cX,cY;
	unsigned char ucStep;
	unsigned char cDirTemp;
	unsigned char saveDir;
	unsigned char nowDir;
	unsigned char mouseDir=0;

	cX=GucXStart;
	cY=GucYStart;

	for(i=0;i<16;i++)
	for(j=0;j<16;j++)
	MapDiagonalway[i][j]=0xff;
	DiagonalwayCount=0;
	ucStep = GucMapStep[cX][cY];     //从（0，0）开始
	if((GucMapBlock[cX][cY] & 0x01) &&(GucMapStep[cX][cY + 1] < (ucStep)))//因为等高图是从终点往起点做的
	{
		cDirTemp = 0;   //目标绝对方向
		if(mouseDir==cDirTemp) //mouseDir电脑鼠当前绝对方向，如果等于目标绝对方向
		{
			MapDiagonalway[cX][cY]=cDirTemp;//将方向值付给数组，也就是起点的方向值为0
			saveDir=cDirTemp;
			DiagonalwayAction[DiagonalwayCount]=0;//对角动作为向上
			DiagonalwayCount++;   //0-255个坐标格 
			cY++;
			goto Sec;  //前进直行吧
                        //此时：cDirTemp = 0  mouseDir= 0 MapDiagonalway[0][0] =0  saveDir=0
                        //DiagonalwayAction[DiagonalwayCount=0]=0  DiagonalwayCount=1
                        //cX=0 cY=1
		}
	}
	if((GucMapBlock[cX][cY] & 0x02) &&(GucMapStep[cX+1][cY] < (ucStep)))
	{
		cDirTemp = 2;
		if(mouseDir==cDirTemp)
		{
			MapDiagonalway[cX][cY]=cDirTemp;
			saveDir=cDirTemp;
			DiagonalwayAction[DiagonalwayCount]=0;  //需要转弯方向，需要行动的方向（电脑鼠直行，不需要转弯？？）
			DiagonalwayCount++;
			cX++;
			goto Sec;
		}
	}
	if((GucMapBlock[cX][cY] & 0x04) &&(GucMapStep[cX][cY-1] < (ucStep)))
	{
		cDirTemp = 4;    //目标绝对方向
		if(mouseDir==cDirTemp)
		{
			MapDiagonalway[cX][cY]=cDirTemp;
			saveDir=cDirTemp;
			DiagonalwayAction[DiagonalwayCount]=0;
			DiagonalwayCount++;
			cY--;
			goto Sec;
		}
	}
	if((GucMapBlock[cX][cY] & 0x08) &&(GucMapStep[cX-1][cY] < (ucStep)))
	{
		cDirTemp = 6;
		if(mouseDir==cDirTemp)
		{
			MapDiagonalway[cX][cY]=cDirTemp;
			saveDir=cDirTemp;
			DiagonalwayAction[DiagonalwayCount]=0;
			DiagonalwayCount++;
			cX--;
			goto Sec;
		}
	}
	DiagonalwayAction[DiagonalwayCount]=(8+cDirTemp-mouseDir)%8;  //转弯方向=（目标绝对方向+8-现在绝对方向）%8
	DiagonalwayCount++;
	MapDiagonalway[cX][cY]=cDirTemp;   // ？？保存目标的绝对方向值？
	saveDir=cDirTemp;    //保存的目标绝对方向
	mouseDir=cDirTemp;   //将电脑鼠目标方向付给当前方向。
	switch(cDirTemp)	
	{ 
		case 0:cY++;
			   break;
		case 2:cX++;
			   break;
		case 4:cY--;
			   break;
		case 6:cX--;
			   break;
		default:break; 
	}
Sec:while((cX != cXdst)||(cY != cYdst))
	{       
//		printf(" ni hao \n");
//                printf("cX=%d, cY=%d, DiagonalwayAction[%d]=%d\n",cX,cY,DiagonalwayCount,DiagonalwayAction[DiagonalwayCount]);
		ucStep = GucMapStep[cX][cY];
		mouseDir=cDirTemp;//先将cDirTemp赋值给mouseDir,此时mouseDir=0
		if((GucMapBlock[cX][cY] & 0x01) && (GucMapStep[cX][cY + 1] < (ucStep)))	
		{//如果上方有出口
			nowDir=0;  
			switch(saveDir)//保存的目标绝对方向
			{
				case 0:
					cDirTemp=0;
					break;
				case 2:              //如果右方有出口
					cDirTemp=1;  //斜右上方方向
					break;
				case 6:
					cDirTemp=7;
					break;	
				default:
					break;
                        //第一次循环在（0，1）点时，nowDir=0; saveDir=0；cDirTemp=0；
			}
			if(mouseDir == cDirTemp)  
			{
				MapDiagonalway[cX][cY]=cDirTemp;//第一次循环在（0，1）点MapDiagonalway[0][1]=0
				saveDir=nowDir;
				DiagonalwayAction[DiagonalwayCount]=0;
				DiagonalwayCount++;
				cY++;  //y纵坐标+1
				continue;////结束本次while循环
                         //第一次循环（0，1），nowDir=0; saveDir=0；cDirTemp=0；
                         //MapDiagonalway[0][1]=0  DiagonalwayAction[DiagonalwayCount=1]=0;
                         // DiagonalwayCount=2；cY=2
			}	
		}
                /*LQ如果当前电脑鼠方向右，方向值为2，那么，如果绝对方向上方有路case0，那么cDirTemp=1*/
		if ((GucMapBlock[cX][cY] & 0x02) && (GucMapStep[cX+1][cY] < (ucStep)))	
		{//如果右方有出口
			nowDir=2;//
			switch(saveDir)//保存的目标绝对方向
			{
				case 0:
					cDirTemp=1;
					break;
				case 2:
					cDirTemp=2;
					break;
				case 4:
					cDirTemp=3;
					break;	
				default:
					break;
			}
			if(mouseDir == cDirTemp)
			{
				MapDiagonalway[cX][cY]=cDirTemp;
				saveDir=nowDir;
				DiagonalwayAction[DiagonalwayCount]=0;
				DiagonalwayCount++;
				cX++;
				continue;//结束本次循环
                                // continue语句只用在for、while、do-while等循环体中，常与if条件语句一起使用，用来加速循环。
			}
		}
		if((GucMapBlock[cX][cY] & 0x04) && (GucMapStep[cX][cY -1] < (ucStep)))	
		{//如果下方有路口
			nowDir=4;
			switch(saveDir)
			{
				case 2:
					cDirTemp=3;
					break;
				case 4:
					cDirTemp=4;
					break;
				case 6:
					cDirTemp=5;
					break;	
				default:
					break;
			}
			if(mouseDir == cDirTemp)
			{
				MapDiagonalway[cX][cY]=cDirTemp;
				saveDir=nowDir;
				DiagonalwayAction[DiagonalwayCount]=0;
				DiagonalwayCount++;
				cY--;
				continue;
			}
		}
		if((GucMapBlock[cX][cY] & 0x08) && (GucMapStep[cX-1][cY ] < (ucStep)))	  //缺乏case2选项
		{//如果左方有路口
			nowDir=6;
			switch(saveDir)
			{
				case 0:
					cDirTemp=7;
					break;
				case 4:
					cDirTemp=5;
					break;
				case 6:
					cDirTemp=6;
					break;	
				default:
					break;
			}
                        //nowDir=6;saveDir=0;cDirTemp=7
			if(mouseDir == cDirTemp)
			{
				MapDiagonalway[cX][cY]=cDirTemp;
				saveDir=nowDir;
				DiagonalwayAction[DiagonalwayCount]=0;
				DiagonalwayCount++;
				cX--;
				continue;
			}
		}
		MapDiagonalway[cX][cY]=cDirTemp;
		DiagonalwayAction[DiagonalwayCount]=(8+MapDiagonalway[cX][cY]-mouseDir)%8;
		DiagonalwayCount++; 
		mouseDir=cDirTemp;
		saveDir=nowDir; 
		switch(nowDir)
		{ 
                    case 0:cY++;  break;
                    case 2:cX++; break;
                    case 4:cY--; break;
                    case 6:cX--;  break;
                    default:break; //如表达式的值与所有case后的常量表达式均不相同时，则执行default后的语句
		} 
                //(0,3)nowDir=6;saveDir=6;cDirTemp=7,mouseDir=7
                //MapDiagonalway[0][3]=7;DiagonalwayAction[DiagonalwayCount]=7
	}//到此while循环
    
    
	DiagonalwayAction[DiagonalwayCount]=(8+saveDir-mouseDir)%8;
	DiagonalwayCount++; 
}

int getsum(int num)
{
	int ret=0,i=0;
	for(i=0;i<=num;i++) ret+=DiagonalwayAction[i];
	return ret;
}

void actiongenerate()//DiagonalwayAction[DiagonalwayCount]
{
	int sum=0,current=0;
	int i=0,j=0;
            
              actionsequence[j++]=STRAIGHT90L;
                      // USART_SendData(USART3, actionsequence[j-1]);
                      // while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);
		for(i=0;i<DiagonalwayCount-1;i++)//
		{	
			if(i<current)continue;
		     if(DiagonalwayAction[i+1]==1  &&DiagonalwayAction[i+2]==2 && DiagonalwayAction[i+3]==1 && getsum(i)%2==0){actionsequence[j++]=R180L;current+=3;}
			else if(DiagonalwayAction[i+1]==7 && DiagonalwayAction[i+2]==6 && DiagonalwayAction[i+3]==7 && getsum(i)%2==0){actionsequence[j++]=L180L;current+=3;}
			else if(DiagonalwayAction[i+1]==2 && DiagonalwayAction[i+2]==1 && getsum(i)%2==1){actionsequence[j++]=RV135L;current+=2;}
			else if(DiagonalwayAction[i+1]==6 && DiagonalwayAction[i+2]==7 && getsum(i)%2==1){actionsequence[j++]=LV135L;current+=2;}
			else if(DiagonalwayAction[i+1]==1 && DiagonalwayAction[i+2]==2 && getsum(i)%2==0){actionsequence[j++]=RT135L;current+=3;}
			else if(DiagonalwayAction[i+1]==7 && DiagonalwayAction[i+2]==6 && getsum(i)%2==0){actionsequence[j++]=LT135L;current+=3;}
			else if(DiagonalwayAction[i+1]==7 && DiagonalwayAction[i+2]==7 && getsum(i)%2==0){actionsequence[j++]=LT90L;current+=2;}
                     //else if(DiagonalwayAction[i+1]==0 && DiagonalwayAction[i+2]==7 && DiagonalwayAction[i+3]==7&&getsum(i)%2==0){actionsequence[j++]=LT90L;current+=3;}
			else if(DiagonalwayAction[i+1]==1 && DiagonalwayAction[i+2]==1 && getsum(i)%2==0){actionsequence[j++]=RT90L;current+=2;}
			else if(DiagonalwayAction[i+1]==2 && getsum(i)%2==1){actionsequence[j++]=RV90L;current+=2;}
			else if(DiagonalwayAction[i+1]==6 && getsum(i)%2==1){actionsequence[j++]=LV90L;current+=2;}
			else if(DiagonalwayAction[i+1]==1 && getsum(i)%2==0){actionsequence[j++]=RT45L;current+=2;}
			else if(DiagonalwayAction[i+1]==7 && getsum(i)%2==0){actionsequence[j++]=LT45L;current+=2;}
			else if(DiagonalwayAction[i+1]==1 && getsum(i)%2==1){actionsequence[j++]=RV45L;current+=1;}
			else if(DiagonalwayAction[i+1]==7 && getsum(i)%2==1){actionsequence[j++]=LV45L;current+=1;}
			else if(DiagonalwayAction[i+1]==0 && getsum(i)%2==0){actionsequence[j++]=STRAIGHT90L;current+=1;}
			else if(DiagonalwayAction[i+1]==0 && getsum(i)%2==1){actionsequence[j++]=STRAIGHT45L ;current+=1;}
 
		}
             hh= DiagonalwayCount;
}