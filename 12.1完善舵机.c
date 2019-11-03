#include "picture.h"
#include "pic_calculate.h"
#include "g_value.h"





//...............................................................................
//...............................................................................
 int pow[8]={1,2,4,8,16,32,64,128};                    //个人全局变量开始处

    #define BOTTOM 127  //仿真算法128行 结束行的位置
	#define WIDTH 320   //图像宽度
    #define LEFTLINE 0   //图像左边界
    #define RIGHTLINE 319 //图像有边界
    #define TOP 0        //仿真算法128行 开始行的位置
	#define HEIGHT 128    //图像的行数
	#define VIEWROWUP 40   //舵机控制行上界
	#define VIEWROWDOWN 90  //舵机控制行下界
	
	
int rLine[HEIGHT],lLine[HEIGHT];           //存储左右边界
//int uLine[WIDTH+20];                         //  存储上边界
int MidLine[140];							//存储 中线 的数组
int lSum=BOTTOM;							// 记录 中线 数组 的 元素个数
int ex[4],ey[4];                            // 记录十字4个点坐标用的

//.............特殊图像判断 所用的变量
  int Left_Lost_Flag=0,Right_Lost_Flag=0;   //原版掉线 包含 左右边界的马蹄形弯道掉线
  int Left_Lost_Flag2=0,Right_Lost_Flag2=0;   //跳跃式掉线，专注于十字的掉线 即明显的 边界坑洞
    int Left_Lost_Flag3=0,Right_Lost_Flag3=0; //针对十字的全白 flag判断 
	int Left_Lost_End3=-1,Right_Lost_End3=-1;   //针对十字的全白flag 判断
  int Left_Lost_Start2=-1,Left_Lost_End2=-1,Right_Lost_Start2=-1,Right_Lost_End2=-1;
  
  int Left_Lost_Start=-1,Left_Lost_End=-1,Right_Lost_Start=-1,Right_Lost_End=-1;
  int Left_Continuous_Flag=1,Right_Continuous_Flag=1,Left_Continuous_Delta=0,Right_Continuous_Delta=0;
  int Left_Bend_Flag=0,Right_Bend_Flag=0;
  int Left_Grad[250],Right_Grad[250];    // 一阶导数
  int Left_Grad_Sum=0,Right_Grad_Sum=0;        //判断 十字是向左 还是向右
  int Left_Grad2[250],Right_Grad2[250];  //二阶导数
  int Left_Straint_Flag=1,Right_Straint_Flag=1;  // 边沿是否是直线判断
  int Cross_Flag=0,Left_Cross_Flag=0,Right_Cross_Flag=0,Strait_Cross_Flag=0;  //十字类型判断
  int Half_Cross_Flag=0;   //进入十字后的半十字判断
  int All_White_Flag=0;   //彻底进入十字后的全白十字判断
  int Mid_Deep=0;          //中线以上竖直距离记录
  int lSum2=20;
//控制舵机使用的变量  
  int Servo_Bend_90_Flag=0; //90度弯道发现flag
  int Servo_Bend_Flag=0;    //进入弯道flag
  int Servo_Strait_Flag=0;           //进入直到flag
  int Servo_Control1=160;             //控制舵机用的 中点值 当前
  int Servo_Control2=160;             //控制舵机用的 中点值 之前
  


void Pic_Calculate(unsigned char imgbuff[])
{
int Mid=160;
int i,j,k;


void ArrayClear(int buff[],int x1,int x2);
void DrawLine2(unsigned char buff[],int x1,int y1,int x2,int y2);  //在原图像上补线
void LineSearch(unsigned char buff[]);                             //边界 和中点 的搜索
void Strait_Cross_Left(unsigned char buff[]);                       //直十字 左边二点
void Strait_Cross_Right(unsigned char buff[]);                   //直十字 右边 的二个点
void Right_Cross_Right(unsigned char buff[]);  //启发条件：十字非常向右，且右边 掉线
void Left_Cross_Left(unsigned char buff[]);    // 启发条件：十字非常向左，且左边 掉线
void Right_Cross_All_White(unsigned char buff[]);  //十字右下边 全白的情况
void Left_All_White(unsigned char buff[]);   // 十字左下边全白的情况
void DrawLine(int buff[],int x1,int y1,int x2,int y2);
void DrawLine3(unsigned char buff[],int x1,int y1,float K,float B); //在原图像上向下补射线
void All_White(unsigned char buff[]);            //左右全白新算法
void Variable_Clear(void);                        //变量初始化
void Servo_Control(void);                         //舵机偏转控制程序

//...........边界平滑处理..............//
//未完成
		Variable_Clear();                  //变量清零

                LineSearch(imgbuff);                             //边界搜索


//..........................特殊图像的提前判断............................//

  //求主点向上的高度
  for(i=BOTTOM;i>TOP;i--)
	  if((imgbuff[i*40+160/8]&pow[7-160%8])!=0 || i==TOP)
	  {
		  Mid_Deep=BOTTOM-i;
		  break;
	  }
  if(lSum>TOP)lSum2=lSum;

  ArrayClear(Left_Grad,BOTTOM,TOP);
  ArrayClear(Left_Grad2,BOTTOM,TOP);
  ArrayClear(Right_Grad,BOTTOM,TOP);
  ArrayClear(Right_Grad2,BOTTOM,TOP);
  
  int Left_Uncontinuous_Flag=0,Right_Uncontinuous_Flag=0;
  Left_Continuous_Delta=lLine[BOTTOM-5]-lLine[BOTTOM];
  Right_Continuous_Delta=rLine[BOTTOM-5]-rLine[BOTTOM];
  
  
  if(Left_Continuous_Delta==0)Left_Continuous_Flag=0;
  if(Right_Continuous_Delta==0)Right_Continuous_Flag=0;
  for(i=BOTTOM;i>lSum+4;i--)
  {
	  //............判断 左右边界是否  单调连续
	  if(((lLine[i-3]-lLine[i])*Left_Continuous_Delta)<0 && Left_Continuous_Flag)
	  {Left_Continuous_Flag=0;}
      if(((rLine[i-3]-rLine[i])*Right_Continuous_Delta)<0 && Right_Continuous_Flag)
	  {Right_Continuous_Flag=0;}
  }
  
  Right_Lost_End=lSum;Left_Lost_End=lSum;
  
  if(lLine[BOTTOM]==LEFTLINE){Left_Lost_Flag=1;Left_Lost_Start=BOTTOM;}
  if(rLine[BOTTOM]==RIGHTLINE){Right_Lost_Flag=1;Right_Lost_Start=BOTTOM;}
  for(i=BOTTOM;i>lSum+3;i--)
  {
	  //...........记录左丢线
	  if( lLine[i-1]==LEFTLINE && !Left_Lost_Flag){Left_Lost_Flag=1;Left_Lost_Start=i;}
	  if(lLine[i]==LEFTLINE && lLine[i-1]!=LEFTLINE && Left_Lost_Flag){Left_Lost_End=i;}
	  //...........记录右丢线
	  if( rLine[i-1]==RIGHTLINE && !Right_Lost_Flag){Right_Lost_Flag=1;Right_Lost_Start=i;}
	  if(rLine[i]==RIGHTLINE && rLine[i-1]!=RIGHTLINE&& Right_Lost_Flag){Right_Lost_End=i;}
	  
	  //十字跳跃式掉线 判断
	  //...........记录左丢线
	  if( lLine[i-1]==LEFTLINE && !Left_Lost_Flag2 && lLine[i]>5+LEFTLINE ){Left_Lost_Flag2=1;Left_Lost_Start2=i;}
	  if(lLine[i]==LEFTLINE && lLine[i-3]>LEFTLINE+5 && Left_Lost_Flag2){Left_Lost_End2=i;}
	  //...........记录右丢线
	  if(rLine[i-1]==RIGHTLINE && !Right_Lost_Flag2 && rLine[i]<RIGHTLINE-5){Right_Lost_Flag2=1;Right_Lost_Start2=i;}
	  if(rLine[i]==RIGHTLINE && rLine[i-3]<RIGHTLINE-5 && Right_Lost_Flag2){Right_Lost_End2=i;}
  }
  if(Left_Lost_End2==-1)Left_Lost_Flag2=0;   //跳跃掉线的补充判断  需要上下都跳才选择 
  if(Right_Lost_End2==-1)Right_Lost_Flag2=0;

  //寻找全白丢线 针对十字
  if(lLine[BOTTOM]==LEFTLINE)
  {
	  
	  for(i=BOTTOM;i>lSum+3;i--)
		   if(lLine[i]==LEFTLINE && lLine[i-3]>LEFTLINE+5)
		   {
			   Left_Lost_Flag3=1;
			   Left_Lost_End3=i;
			   break;
		   }
  }
  if(rLine[BOTTOM]==RIGHTLINE)
  {
	  
	  for(i=BOTTOM;i>lSum+3;i--)
		   if(rLine[i]==RIGHTLINE && rLine[i-3]<RIGHTLINE-5)
		   {
			   Right_Lost_Flag3=1;
			   Left_Lost_End3=i;
			   break;
		   }
  }
  
  
  Left_Grad[BOTTOM]=0;Right_Grad[BOTTOM]=0;
  Left_Grad2[BOTTOM]=0;Right_Grad2[BOTTOM]=0;
  for(i=BOTTOM-1;i>BOTTOM-4;i--)
  {
     Left_Grad[i]=lLine[i]-lLine[i+1];
	 Left_Grad2[i]=Left_Grad[i]-Left_Grad[i+1];

	  Right_Grad[i]=rLine[i]-rLine[i+1];
	  Right_Grad2[i]=Right_Grad[i]-Right_Grad[i+1];
  }

  for(i=BOTTOM-4;i>=lSum;i--)   //求边界线 函数 一阶导数 和 二阶导数  注意：初始的 4行 没有 导数
  {
	  Left_Grad[i]=lLine[i]-lLine[i+4];
	  Left_Grad2[i]=Left_Grad[i]-Left_Grad[i+2];
	  
	  Right_Grad[i]=rLine[i]-rLine[i+4];
	  Right_Grad2[i]=Right_Grad[i]-Right_Grad[i+2];
  }

  //for(i=BOTTOM;i>lSum2+10;i--)  //故意 减少 10行 减少 误差,故意将最前沿 缩减20 行，来处理小 s 型弯道
  //{
	//  if(Left_Grad2[i]>10 || Left_Grad2[i] <-10)Left_Uncontinuous_Flag=1;
  //    if(Right_Grad2[i]>10 || Right_Grad2[i] <-10)Right_Uncontinuous_Flag=1;
	//  Left_Grad_Sum+=Left_Grad[i];
	//  Right_Grad_Sum+=Right_Grad[i];
 // }
 //使用新的判断是否出现较大坑洞的 flag判断算法:斜率
  //使用新的flag判断直线 和曲线
 for(i=BOTTOM;i>lSum2+3;i--)
 {
	 if(Left_Grad[i]*Left_Grad[i+3]<0&& !Left_Uncontinuous_Flag)
	 {
		 Left_Uncontinuous_Flag=1;
		 //str.Format("Left_Uncontinuous_X=%d\n\r",i+3);PrintDebug(str);
		for(j=BOTTOM;j>i+3;j--)
		{
			if((Left_Grad[j]>5||Left_Grad[j]<-5)&& Left_Straint_Flag)Left_Straint_Flag=0;
		}
	 }
	 
	 if(Right_Grad[i]*Right_Grad[i+3]<0 && Right_Uncontinuous_Flag)
	 {
		 Right_Uncontinuous_Flag=1;
		for(j=BOTTOM;j>i+3;i--)
		{
			if((Right_Grad[j]>5||Right_Grad[j]<-5)&& Right_Straint_Flag)Right_Straint_Flag=0;
		}
	 }
 }

 
 
 
  // 判断 是否是出现十字
 
  
  if(Left_Uncontinuous_Flag && Right_Uncontinuous_Flag &&(Left_Lost_Flag || Right_Lost_Flag) )Cross_Flag=1;
   if((!Left_Continuous_Flag) && (!Right_Continuous_Flag) &&((Left_Lost_Flag2 && (Left_Lost_Start2-Left_Lost_End2)>20 )||(Right_Lost_Flag2 &&(Right_Lost_Start2-Right_Lost_End2)>20)))Cross_Flag=1;
  //if((Left_Lost_Flag && (Left_Lost_Start-Left_Lost_End)>20 )&&(Right_Lost_Flag &&(Right_Lost_Start-Right_Lost_End)>20))Cross_Flag=1;
    
   if(Left_Lost_Flag &&Right_Lost_Flag)Cross_Flag=1;
   //线段定长卡口法 来判断十字 最下面 最长200（实际特殊图像 可以达到220）且加上 左右边界不能靠近 中间部分
   //的判断（区别掉线的弯道）
   if((rLine[BOTTOM]-lLine[BOTTOM])>220 && lLine[BOTTOM]<40 && rLine[BOTTOM]>280)Cross_Flag=1;
  // if(Left_Lost_Flag3||Right_Lost_Flag3)Cross_Flag=1;
    
	//线段定高+某边全掉线 判别十字的新方法 排除 马蹄形误判的干扰
   
   if(((Left_Lost_Flag3 && BOTTOM-Left_Lost_End3+20<Mid_Deep) || (Right_Lost_Flag3 && BOTTOM-Right_Lost_End3+20<Mid_Deep ))&& Mid_Deep>70)	   
   {
	   Cross_Flag=1;
	   //if((Left_Lost_Flag3 && BOTTOM-Left_Lost_End3<Mid_Deep)||(Right_Lost_Flag3 && BOTTOM-Right_Lost_End3>Mid_Deep))
   }
  //排除十字误判 ：马蹄形
  //if(Left_Uncontinuous_Flag&& !Left_Straint_Flag)Cross_Flag=0;
 // if(Right_Uncontinuous_Flag && !Right_Straint_Flag)Cross_Flag=0;
  
  
  
 // 当出现十字  左右两边肯定都不单调 连续 ，即不需要补弯道
  if(Cross_Flag)
  {
	  Left_Continuous_Flag=0;
	  Right_Continuous_Flag=0;
  }

  //判断是什么十字
  if(Cross_Flag)
  {
	  
	  if((rLine[BOTTOM]-lLine[BOTTOM])>240 && lLine[BOTTOM]<60 && rLine[BOTTOM]>250)All_White_Flag=1;
	  else
	  if(Right_Lost_Flag && !Left_Lost_Flag)Right_Cross_Flag=1;else
	  if(!Right_Lost_Flag && Left_Lost_Flag)Left_Cross_Flag=1;else
	  Strait_Cross_Flag=1;
	  
	  if((Right_Lost_Flag && Right_Lost_Start==BOTTOM)&&(Left_Lost_Flag && Left_Lost_Start==BOTTOM))
		 All_White_Flag=1; 
	 else
		 if((Right_Lost_Flag && Right_Lost_Start==BOTTOM)||(Left_Lost_Flag && Left_Lost_Start==BOTTOM))
		Half_Cross_Flag=1;
	 else
		 if((Left_Lost_Flag3 || Right_Lost_Flag3)&& Mid_Deep>90)Half_Cross_Flag=1;
	  
  }






//*****************************补十字用的代码***********************************//

ex[0]=0;ex[1]=0;ex[2]=0;ex[3]=0;  //记录4个点坐标用的 数组


 if(Cross_Flag)  // 十字识别 的启发条件
  {
    
	if(Strait_Cross_Flag && !Half_Cross_Flag && !All_White_Flag)
	{
		Strait_Cross_Left(imgbuff);   //寻找 左边的二点
		Strait_Cross_Right(imgbuff);//寻找右边二个点
	}
	
	
	if(Left_Cross_Flag && !Half_Cross_Flag && !All_White_Flag)
	{
		Left_Cross_Left(imgbuff);
		Strait_Cross_Right(imgbuff);
	}
	
	if(Right_Cross_Flag && !Half_Cross_Flag && !All_White_Flag)
	{
		Strait_Cross_Left(imgbuff);
		Right_Cross_Right(imgbuff);
	}
	
	/*if(Half_Cross_Flag && Strait_Cross_Flag && !All_White_Flag)
	{
		All_White(imgbuff);//线段定长卡口法 来处理 strait 且 half-flag
	}
	else*/
	if(Half_Cross_Flag && !All_White_Flag)
	{
		if(Left_Lost_Flag && Left_Lost_Start==BOTTOM)Left_All_White(imgbuff);else Strait_Cross_Left(imgbuff);
		
		if(Right_Lost_Flag && Right_Lost_Start==BOTTOM)Right_Cross_All_White(imgbuff);else Strait_Cross_Right(imgbuff);
	}
	
	if(All_White_Flag)All_White(imgbuff);//当左右两边全白


   
   } 



//.................. 中线重新搜索...................
        LineSearch(imgbuff);
	  
	  
		// 弯道平滑处理  补弯道    实质上很不完善， 对于掉线较为严重的弯道没有用
		//...........................补弯道用得代码.........................................//
		if(!Cross_Flag)
		if((Left_Continuous_Flag && Right_Lost_Flag)||(!Left_Lost_Flag && Right_Lost_Flag && !Right_Lost_Flag2))//如果左连续 并且 右丢线 说明进入了 右弯道
		{
			DrawLine(MidLine,Right_Lost_Start,MidLine[Right_Lost_Start],lSum,RIGHTLINE);
		}
		if(!Cross_Flag)
		if((Right_Continuous_Flag && Left_Lost_Flag)||(!Right_Lost_Flag && Left_Lost_Flag && !Left_Lost_Flag2))
		{
			DrawLine(MidLine,Left_Lost_Start,MidLine[Left_Lost_Start],lSum,LEFTLINE);
		}


  Servo_Control();





																		//在显示屏幕上显示 中线
  for(i=BOTTOM;i>=lSum;i--)
  {
    if(imgbuff[i*40+MidLine[i]/8]+pow[7-MidLine[i]%8]<256)
			{


				imgbuff[i*40+MidLine[i]/8]+=pow[7-MidLine[i]%8];   //显示中线    通过加法来显示中线  化0为 一
		    }
  }


}



void DrawLine(int buff[],int x1,int y1,int x2,int y2)  //在 中线上补线  补弯道用得代码
{
	float K,B;
	int i;
	K=(y1-y2)*1.0/(x1-x2);
	B=y1-K*x1;

	for(i=x2;i<=x1;i++)
		buff[i]=K*i+B;
}

void DrawLine2(unsigned char buff[],int x1,int y1,int x2,int y2)  //在原图像上补线
{
	float K,B;
	int i,y;
	K=(y1-y2)*1.0/(x1-x2);
	B=y1-K*x1;
	
	for(i=x2;i<=x1;i++)
	{
		y=K*i+B;
		if(buff[i*40+y/8]+pow[7-y%8]<256)buff[i*40+y/8]+=pow[7-y%8];
	}
	
}

void DrawLine3(unsigned char buff[],int x1,int y1,float K,float B) //在原图像上向下补射线
{
	int i,y;
	for(i=x1;i<=BOTTOM;i++)
	{
		y=K*i+B;
		if(y<LEFTLINE)y=LEFTLINE;
		if(y>RIGHTLINE)y=RIGHTLINE;
		if(buff[i*40+y/8]+pow[7-y%8]<256)buff[i*40+y/8]+=pow[7-y%8];
	}
	 
}

void ArrayClear(int buff[],int x1,int x2)
{
	int i;

	for(i=x1;i>=x2;i--)
		buff[i]=0;
}
void LineSearch(unsigned char buff[])
{
	  int Mid;
	  int i,j;
	  Mid=(LEFTLINE+RIGHTLINE)/2;
      for(i=BOTTOM;i>=TOP;i--)              //从下面向上面搜索
	  {
		  rLine[i]=RIGHTLINE;              //初始是  280 列 和  40 列  减小视野
		  lLine[i]=LEFTLINE;
		  for(j=Mid;j<=RIGHTLINE;j++)
			  if((buff[i*40+j/8]&pow[7-j%8])!=0)   //从中间向两边寻找 边线  1为黑边
			  {
				  rLine[i]=j-1;                        //记录边线位置
				  break;
			  }
		  for(j=Mid;j>=LEFTLINE;j--)
			  if((buff[i*40+j/8]&pow[7-j%8])!=0)
			  {
				  lLine[i]=j+1;
				  break;
			  }
		                 //更新中点值 方便搜索 上一行
		
		if(buff[i*40+Mid/8]+pow[7-Mid%8]<256)
			{
				lSum=i;                               //当 上面为 黑边的时候跳出 寻中线
				Mid=(lLine[i]+rLine[i])/2;
				MidLine[i]=Mid;

				
		    }else break;
	  

	  } 
}

void Strait_Cross_Left(unsigned char buff[])        //直十字 左边二点
{
	int pUp,nUp=0;    //寻找上面两个点用的 记录上层的变量
	pUp=BOTTOM;
	int t=0,delta=0,Mid;
	int i,j;
for(i=lLine[BOTTOM];i<=RIGHTLINE;i++) //寻找 左下点   竖着搜跳点
	{
		for(j=BOTTOM;j>=TOP;j--)
			if((buff[j*40+i/8]&pow[7-i%8])!=0)
			{
			 nUp=j; 
             break;			 
			}
	if(pUp-nUp>10  && pUp!=-1)				//当两次 上边界距离相差 超过10 即为找到 跳点 记录下来 退出寻找
	{
		ex[0]=pUp;
		ey[0]=i-1;
		t++;
		break;	
	}
	pUp=nUp;
	}
	
	

        //***************如果找到左下点就找左上点
	if(t==1){										//当找到了下面的点  就开始寻找上面的跳点
    nUp=0;
	pUp=-1;
 Mid=ex[0]-10;if(Mid<TOP)Mid=TOP;
	for(i=ey[0];i<=RIGHTLINE;i+=3)					//沿着 下面跳点 的上方 向 右部分寻找 上面的跳点   缺点：当十字非常偏 上面的跳点可能在下面 跳点的偏左边 一点
	{
		
		for(j=Mid;j>=0;j--)
			if((buff[j*40+i/8]&pow[7-i%8])!=0)
			{
			 nUp=j; 
             break;			 
			}
    
	if(pUp!=-1&&pUp-nUp-delta>1)
	{
		ex[3]=pUp;
		ey[3]=i-1;
		++t;
		break;
		
	}
	if(pUp!=-1)delta=pUp-nUp;
	pUp=nUp;
	Mid=nUp+10;if(Mid>BOTTOM)Mid=BOTTOM;
    	  if((buff[Mid*40+i/8]&pow[7-i%8])!=0)break;            // 保险装置 ，防止搜进边界里面去了  对于 解决 误判十字 有80%的作用 
	}

    if(t==2)DrawLine2(buff,ex[0],ey[0],ex[3],ey[3]);
	}
	
}
void Strait_Cross_Right(unsigned char buff[])                   //直十字 右边 的二个点
{
	int pUp,nUp=0;    //寻找上面两个点用的 记录上层的变量
	pUp=BOTTOM;
	int t=0,delta=0,Mid;
	int i,j;
	
    t=0;
	nUp=0;
	pUp=BOTTOM;
//	if(rLine[BOTTOM]!=RIGHTLINE)
	for(i=rLine[BOTTOM];i>=LEFTLINE;i--) //寻找 右下点
	{
		for(j=BOTTOM;j>=0;j--)
			if((buff[j*40+i/8]&pow[7-i%8])!=0)
			{
			 nUp=j; 
             break;			 
			}
	if(pUp-nUp>10)
	{
		ex[1]=pUp;
		ey[1]=i-1;
		t++;
		break;	
	}
	
	pUp=nUp;
	}
	


	            //..........如果找到右下的点 就找 右上的点

	if(t==1)
	{
 
	Mid=ex[1]-10;if(Mid<TOP)Mid=TOP;
	pUp=-1;
	nUp=0;
	delta=0;
 for(i=ey[1];i>=LEFTLINE;i-=3)
 {
	 for(j=Mid;j>0;j--)
		 if((buff[j*40+i/8]&pow[7-i%8])!=0)
		 {
			nUp=j;
            break;			
		 }
		 
	 
 		 if(pUp!=-1&&pUp-nUp>3)
		 {
			 ex[2]=pUp;
			 ey[2]=i;
			 ++t;
			 break;
		 }
		 //if(pUp!=-1)delta=pUp-nUp;
		 pUp=nUp;
		 Mid=nUp+10;if(Mid>BOTTOM)Mid=BOTTOM;
         if((buff[Mid*40+i/8]&pow[7-i%8])!=0)break;    
 }

		
		
	if(t==2)DrawLine2(buff,ex[1],ey[1],ex[2],ey[2]);  //在原图上补全十字
	}
}
void Left_Cross_Left(unsigned char buff[])
{
	int Mid=(Left_Lost_Start+Left_Lost_End)/2;
	int i,j,t=1;
	int pUp=-1,nUp=0;
	ex[0]=Left_Lost_Start;
	ey[0]=lLine[ex[0]];
	
	for(i=LEFTLINE;i<=RIGHTLINE;i++)
	{
		for(j=Mid;j>=TOP;j--)
			if(buff[j*40+i/8]&pow[7-i%8]!=0)
			{
				nUp=j;
				break;
			}
		if(pUp-nUp>5 && pUp!=-1)
		{
			ex[3]=pUp;
			ey[3]=i-1;
			++t;
			break;
		}
		pUp=nUp;
		if(nUp+10<=BOTTOM)Mid=nUp+10;else Mid=BOTTOM;
		if((buff[Mid*40+i/8]&pow[7-i%8])!=0)break;
	}
	if(t==2)DrawLine2(buff,ex[0],ey[0],ex[3],ey[3]);  //在原图上补全十字
}

void Right_Cross_Right(unsigned char buff[])  //启发条件：十字非常向右，且右边 掉线
{
	int Mid=(Right_Lost_Start+Right_Lost_End)/2;
	int i,j,t=1;
	int pUp=-1,nUp=0;
	ex[1]=Right_Lost_Start;
	ey[1]=rLine[ex[1]];
	
	for(i=RIGHTLINE;i>=LEFTLINE;i--)
	{
		for(j=Mid;j>=TOP;j--)
			if(buff[j*40+i/8]&pow[7-i%8]!=0)
			{
				nUp=j;
				break;
			}
			
			if(pUp-nUp>5 && pUp!=-1)
			{
				ex[2]=pUp;
				ey[2]=i;
				++t;
				break;
			}
			pUp=nUp;
			if(nUp+10<=BOTTOM)Mid=nUp+10;else Mid=BOTTOM;
			if((buff[Mid*40+i/8]&pow[7-i%8])!=0)break;
		
	}
	if(t==2)DrawLine2(buff,ex[1],ey[1],ex[2],ey[2]);  //在原图上补全十字

}

void Left_All_White(unsigned char buff[])   // 十字左下边全白的情况
{
	float K,B;
	int i,j;
	int pUp=-1,nUp=0;
	for(i=LEFTLINE;i<=RIGHTLINE;i+=4)
	{
		for(j=BOTTOM;j>=TOP;j--)
			if(buff[j*40+i/8]&pow[7-i%8]!=0)
			{
				nUp=j;
				break;
			}
		if(pUp!=-1&&pUp-nUp>7)
		{
			K=4.0/(nUp-pUp);
			B=i-K*nUp;
			DrawLine3(buff,nUp,i,K,B);
			break;
		}
		pUp=nUp;
	}
}
void Right_Cross_All_White(unsigned char buff[])  //十字右下边 全白的情况
{
	float K,B;
	int i,j;
	int pUp=-1,nUp=0;
	for(i=RIGHTLINE;i>=LEFTLINE;i-=4)
	{
		for(j=BOTTOM;j>=TOP;j--)
			if(buff[j*40+i/8]&pow[7-i%8]!=0)
			{
				nUp=j;
				break;
			}
		if(pUp!=-1&&pUp-nUp>7)
		{
			K=-4.0/(nUp-pUp);
			B=i-K*nUp;
			DrawLine3(buff,nUp,i,K,B);
			break;
		}
		pUp=nUp;
	}
}
void All_White(unsigned char buff[]) //当左右两边 均为空白全丢线：直接中线长度固定来卡十字
{
	int i,j;
	int l=0,r=319,l1,r1;
	float K,B;
	int Mid;
	for(i=BOTTOM;i>TOP+4;i--)
	{ 
        l=0;r=319;

		for(j=160;j<=RIGHTLINE;j++)           //寻找十字路线，
			if((buff[i*40+j/8]&pow[7-j%8])!=0)
			{
				r=j-1;
				
				break;
			}
		for(j=160;j>=LEFTLINE;j--)
			if((buff[i*40+j/8]&pow[7-j%8])!=0)
			{
				l=j+1;
				
				break;
			}
		
		if(r-l<=160 && r<260 && l>60)   //当找到正确了方向就准备进行补线
		{

		    
			Mid=(r+l)/2;
		    for(j=Mid;j<=RIGHTLINE;j++)
			if((buff[(i-8)*40+j/8]&pow[7-j%8])!=0)
			{
				r1=j-1;
				//str.Format("r1=%d\n\r",r1);PrintDebug(str);
				break;
			}
		
		   for(j=Mid;j>=LEFTLINE;j--)
			if((buff[(i-8)*40+j/8]&pow[7-j%8])!=0)
			{
				l1=j+1;
				//str.Format("l1=%d\n\r",l1);PrintDebug(str);
				break;
			}

					
			K=(l-l1)*1.0/8;
			B=l-K*i;
			DrawLine3(buff,i-8,l1,K,B);
			
			K=(r-r1)*1.0/8;
			B=r-K*i;
			DrawLine3(buff,i-8,r1,K,B);
			
			break;
		}	
	}
	
	
}

void Servo_Control(void)
{
	int i;
	int i_deadline=0;
	int num=0;
	
	//求 平均的偏差
	Servo_Control2=Servo_Control1;
	Servo_Control1=0;
	i_deadline=lSum>VIEWROWUP?lSum:VIEWROWUP;
	for(i=VIEWROWDOWN;i>=i_deadline;i--)
	{
		Servo_Control1+=(MidLine[i]-160)*(1+(i_deadline-VIEWROWUP)/(VIEWROWDOWN-VIEWROWUP));
		++num;
	}
	str.Format("i_deadline=%d\n\r",i_deadline);PrintDebug(str);
	str.Format("VIEWROWUP=%d\n\r",VIEWROWUP);PrintDebug(str);
	str.Format("VIEWROWDOWN=%d\n\r",VIEWROWDOWN);PrintDebug(str);
	Servo_Control1/=num;
	
	
	
    Servo=Servo_Middle+Servo_Control1*Servo_P+(Servo_Control1-Servo_Control2)*Servo_D;
 
         
    if(Servo>4950)Servo=4950;
    if(Servo<3960)Servo=3960;
        ftm_pwm_duty(FTM1,FTM_CH0,Servo); 
		
		
	// 分段函数控制 舵机 直到 P值小  D值小   弯道 P值大  D 值大
	//调车确定直到的 Servo_Control1范围 然后就可以卡死
	/*Servo_Strait_Flag=1;
	Left_Continuous_Flag=1;
	Right_Continuous_Flag=1;
	Left_Continuous_Delta=lLine[BOTTOM-5]-lLine[BOTTOM];
	Right_Continuous_Delta=rLine[BOTTOM-5]-rLine[BOTTOM];
  
  
  if(Left_Continuous_Delta==0)Left_Continuous_Flag=0;
  if(Right_Continuous_Delta==0)Right_Continuous_Flag=0;
  for(i=BOTTOM;i>(lSum+4>VIEWROWUP?lSum+4:VIEWROWUP);i--)
  {
	  //............判断 左右边界是否  单调连续
	  if(((lLine[i-3]-lLine[i])*Left_Continuous_Delta)<0 && Left_Continuous_Flag)
	  {Left_Continuous_Flag=0;Servo_Strait_Flag=0;break;}
      if(((rLine[i-3]-rLine[i])*Right_Continuous_Delta)<0 && Right_Continuous_Flag)
	  {Right_Continuous_Flag=0;Servo_Strait_Flag=0;break;}
  }*/

}

void Variable_Clear(void)
{
	 Left_Lost_Flag=0;Right_Lost_Flag=0;   //原版掉线 包含 左右边界的马蹄形弯道掉线
	 Left_Lost_Flag2=0;Right_Lost_Flag2=0;   //跳跃式掉线，专注于十字的掉线 即明显的 边界坑洞
     Left_Lost_Flag3=0;Right_Lost_Flag3=0; //针对十字的全白 flag判断 
	 Left_Lost_End3=-1;Right_Lost_End3=-1;   //针对十字的全白flag 判断
     Left_Lost_Start2=-1;Left_Lost_End2=-1;Right_Lost_Start2=-1;Right_Lost_End2=-1;
     
	 Left_Lost_Start=-1;Left_Lost_End=-1;Right_Lost_Start=-1;Right_Lost_End=-1;
     Left_Continuous_Flag=1;Right_Continuous_Flag=1;Left_Continuous_Delta=0;Right_Continuous_Delta=0;
     Left_Bend_Flag=0;Right_Bend_Flag=0;
     Left_Grad_Sum=0;Right_Grad_Sum=0;        //判断 十字是向左 还是向右
     Left_Straint_Flag=1;Right_Straint_Flag=1;  // 边沿是否是直线判断
     Cross_Flag=0;Left_Cross_Flag=0;Right_Cross_Flag=0;Strait_Cross_Flag=0;  //十字类型判断
     Half_Cross_Flag=0;   //进入十字后的半十字判断
     All_White_Flag=0;   //彻底进入十字后的全白十字判断
     Mid_Deep=0;          //中线以上竖直距离记录
     lSum2=20;
	 //舵机控制flag
	Servo_Bend_90_Flag=0;   //90度弯道发现flag
    Servo_Bend_Flag=0;      //进入弯道flag
    Servo_Strait_Flag=0;          //进入直到flag
    //Servo_Control1=160;             //控制舵机用的 中点值 当前
    //Servo_Control2=160;             //控制舵机用的 中点值 之前
}



