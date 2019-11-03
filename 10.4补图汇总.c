// AlgorithmDLL.cpp : Defines the initialization routines for the DLL.
//

#include "stdafx.h"
#include "AlgorithmDLL.h"
#include "string.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


//说明：用户只需要修改：全局变量区 和 自定义仿真算法区 就可以进行算法仿真插件的开发
//其它内容请勿修改。



//=============================全局变量区==================================//

#define IMG_ROWS 128      //图像的总行数，即高
#define IMG_COLS 320      //图像的总列数，即宽
#define WHITE 255        //宏定义白
#define RED 128        //宏定义
#define BLUE 254        //宏定义
#define GREEN 100        //宏定义
#define BLACK 0          //宏定义黑
#define THRESHOLD 100    //宏定义阈值

unsigned char ImageData[IMG_ROWS][IMG_COLS];//图像数据

//=========================================================================//


char* pFileName = "中心线提取信息.txt";
FILE *pfile = fopen(pFileName,"w");
CString str="";



//添加到上位机后显示的算法名称，只需修改名称内容即可
void _stdcall AlgorithmName(char *pName)
{
	char *p = "The best";
	strcpy(pName,p);
}

// 初始化图像数据
// 该段代码不用做任何修改,作用为从上位机取数据填充到ImageData
// 就如同下位机的采集程序一样的效果.
void InitImgData(void *pData,int w,int h)
{
	int i=0,j=0;
	// 配置的图像大小与上位机的图像大小不一致，将发生错误。请自行修改宏定义的IMG_ROWS和IMG_COLS，要和上位机图像大小一致
	if (w != IMG_COLS || h != IMG_ROWS)
	{
		AfxMessageBox("图像大小设置不合理");
	}
	for (i = 0;i < IMG_ROWS;i++)
	{
		for (j = 0;j < IMG_COLS;j++)
		{
			ImageData[i][j] = *((UCHAR*)pData+i* IMG_COLS +j);
		}
	}
}


// 该段代码不用做任何修改
// 作用为：把执行算法处理后把处理后的图像数据送到上位机显示
// 上位机最终要显示的是以pData指针为首地址的数组数据
void UpdateImgData(void *pData)
{
	int i=0,j=0;
	for (i = 0;i < IMG_ROWS;i++)
	{
		for (j = 0;j < IMG_COLS;j++)
		{
			*((UCHAR*)pData+i* IMG_COLS +j) = ImageData[i][j];
		}
	}
}

void PrintDebug(CString ss)
{
	fprintf(pfile,ss);
}



//================================================自定义仿真算法区开始===========================================//

//=================================================================
//功能：图像处理仿真算法函数
//参数：w图像宽，即图像的总列数------>上位机调用时提供图像的宽度
//      h图像高，即图像的总行数------>上位机调用时提供图像的高度
//      pData 上位机调用该函数时提供显示图像缓冲区的开始指针
//================================================================
void _stdcall HandleImg(void *pData,ULONG w,ULONG h)
{
	InitImgData(pData,w,h);

	//============================用户需要添加的算法仿真代码开始=============================//
	#define BOTTOM 127  //仿真算法128行 结束行的位置
	#define WIDTH 320   //图像宽度
    #define LEFTLINE 0   //图像左边界
    #define RIGHTLINE 319 //图像有边界
    #define TOP 0        //仿真算法128行 开始行的位置
	#define HEIGHT 128    //图像的行数


	void SortBend(unsigned char imgbuff[],int pow[],int x,int y,int dir);
	unsigned char imgbuff[240*320/8];  //添加数组二维转一维的工具，解码,但注意的是 测试图像只有128行
	int pow[8]={1,2,4,8,16,32,64,128};
	int i,j,k;


	for(j=0;j<=BOTTOM;j++)  //行扫描
   for(i=0;i<WIDTH/8;i++)  //8列每列 的扫描
   {
	 imgbuff[j*40+i]=0;     //初始全部赋值为0 目的 是方便运用 位运算 八位压缩
     for(k=0;k<8;k++)      //8位压缩
	 {
		 imgbuff[j*40+i]<<=1;  //进位
		 if(ImageData[j][i*8+k]==0)   //将 上位机 黑白表示 与 车上摄像头 统一
		 imgbuff[j*40+i]|=1;
	 }	 
   }
	  
   //================================= 工程代码直接 开始处 ======================================//
  
  
float K,B;                                 //连接二点 用的  斜率 和 截距
int Mid=160;
int rLine[HEIGHT],lLine[HEIGHT];           //存储左右边界
int MidLine[140];							//存储 中线 的数组
int lSum=BOTTOM;							// 记录 中线 数组 的 元素个数

      for(i=BOTTOM;i>=TOP;i--)              //从下面向上面搜索
	  {
		  rLine[i]=RIGHTLINE;              //初始是  280 列 和  40 列  减小视野
		  lLine[i]=LEFTLINE;
		  for(j=Mid;j<=RIGHTLINE;j++)
			  if((imgbuff[i*40+j/8]&pow[7-j%8])!=0)   //从中间向两边寻找 边线  1为黑边
			  {
				  rLine[i]=j-1;                        //记录边线位置
				  break;
			  }
		  for(j=Mid;j>=LEFTLINE;j--)
			  if((imgbuff[i*40+j/8]&pow[7-j%8])!=0)
			  {
				  lLine[i]=j+1;
				  break;
			  }
		Mid=(lLine[i]+rLine[i])/2;                      //更新中点值 方便搜索 上一行
		if(imgbuff[i*40+Mid/8]+pow[7-Mid%8]<256)
		
			{
				lSum=i;                               //此行功能暂时较为鸡肋
				    //imgbuff[i*40+lLine[i]/8]+=pow[7-lLine[i]%8];
					//imgbuff[i*40+rLine[i]/8]+=pow[7-rLine[i]%8];

			//	imgbuff[i*40+Mid/8]+=pow[7-Mid%8];   //显示中线    通过加法来显示中线  化0为 一
		    }
	  
	  }










//*****************************补十字用的代码***********************************//
int ex[4],ey[4];
ex[0]=0;ex[1]=0;ex[2]=0;ex[3]=0;  //记录4个点坐标用的 数组
int t=0;
int pUp,nUp=0;    //寻找上面两个点用的 记录上层的变量
int delta=0;          //寻找拐点用的 增量增减记录变量



//**********寻找左边二个点
    pUp=BOTTOM;

	//if(lLine[BOTTOM]!=0)
for(i=lLine[BOTTOM];i<=RIGHTLINE;i++) //寻找 左下点   竖着搜跳点
	{
		for(j=BOTTOM;j>=TOP;j--)
			if((imgbuff[j*40+i/8]&pow[7-i%8])!=0)
			{
			 nUp=j; 
             break;			 
			}
	if(pUp-nUp-delta>10  && pUp!=-1)				//当两次 上边界距离相差 超过10 即为找到 跳点 记录下来 退出寻找
	{
		ex[0]=pUp;
		ey[0]=i-1;
		t++;
		break;	
	}
	pUp=nUp;
	}
	
	
/*	for(i=BOTTOM;i>lSum;i--)                //横着搜跳点
{
	if(lLine[i]-lLine[i-1]>10)
	{
		ex[0]=i;
		ey[0]=lLine[i]; 
		t=2;
		break;	
	}
}*/
        //***************如果找到左下点就找左上点
	if(t==1){										//当找到了下面的点  就开始寻找上面的跳点
    nUp=0;
	pUp=-1;
	delta=0;
 Mid=ex[0]-10;
	for(i=ey[0];i<=RIGHTLINE;i+=3)					//沿着 下面跳点 的上方 向 右部分寻找 上面的跳点   缺点：当十字非常偏 上面的跳点可能在下面 跳点的偏左边 一点
	{
		
		for(j=Mid;j>=0;j--)
			if((imgbuff[j*40+i/8]&pow[7-i%8])!=0)
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
	Mid=nUp+10;
    	  if((imgbuff[Mid*40+i/8]&pow[7-i%8])!=0)break;            // 保险装置 ，防止搜进边界里面去了  对于 解决 误判十字 有80%的作用 
	}


	}
	





		//**********寻找右边二个点

	
	nUp=0;
	pUp=BOTTOM;
//	if(rLine[BOTTOM]!=RIGHTLINE)
	for(i=rLine[BOTTOM];i>=LEFTLINE;i--) //寻找 右下点
	{
		for(j=BOTTOM;j>=0;j--)
			if((imgbuff[j*40+i/8]&pow[7-i%8])!=0)
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
	
/*for(i=BOTTOM;i>lSum;i--)
{
	if(rLine[i-1]-rLine[i]>10)
	{
		ex[1]=i;
		ey[1]=rLine[i]; 
		t=2;
		break;	
	}
}*/

	            //..........如果找到右下的点 就找 右上的点

	if(t==3)
	{
 
	Mid=ex[1]-10;
	pUp=-1;
	nUp=0;
	delta=0;
 for(i=ey[1];i>=LEFTLINE;i-=3)
 {
	 for(j=Mid;j>0;j--)
		 if((imgbuff[j*40+i/8]&pow[7-i%8])!=0)
		 {
			nUp=j;
            break;			
		 }
		 
	 
 		 if(pUp!=-1&&pUp-nUp-delta>1)
		 {
			 ex[2]=pUp;
			 ey[2]=i;
			 ++t;
			 break;
		 }
		 if(pUp!=-1)delta=pUp-nUp;
		 pUp=nUp;
		 Mid=nUp+10;
         if((imgbuff[Mid*40+i/8]&pow[7-i%8])!=0)break;    
 }
 		K=(ey[2]-ey[1])*1.0/(ex[2]-ex[1]);
		B=ey[1]-K*ex[1];
		for(i=ex[2];i<=ex[1];i++)
		{
			rLine[i]=K*i+B;
			if(imgbuff[i*40+rLine[i]/8]+pow[7-rLine[i]%8]<256)imgbuff[i*40+rLine[i]/8]+=pow[7-rLine[i]%8];
		}
	
		K=(ey[3]-ey[0])*1.0/(ex[3]-ex[0]);
		B=ey[0]-K*ex[0];
		for(i=ex[3];i<=ex[0];i++)
		{
			lLine[i]=K*i+B;
			if(imgbuff[i*40+lLine[i]/8]+pow[7-lLine[i]%8]<256)imgbuff[i*40+lLine[i]/8]+=pow[7-lLine[i]%8];

		}
	}






//.................. 中线重新搜索...................
	Mid=(LEFTLINE+RIGHTLINE)/2;
	int pMid=-1;
      for(i=BOTTOM;i>=TOP;i--)              //从下面向上面搜索
	  {
		  rLine[i]=RIGHTLINE;              //初始是  280 列 和  40 列  减小视野
		  lLine[i]=LEFTLINE;
		  for(j=Mid;j<=RIGHTLINE;j++)
			  if((imgbuff[i*40+j/8]&pow[7-j%8])!=0)   //从中间向两边寻找 边线  1为黑边
			  {
				  rLine[i]=j-1;                        //记录边线位置
				  break;
			  }
		  for(j=Mid;j>=LEFTLINE;j--)
			  if((imgbuff[i*40+j/8]&pow[7-j%8])!=0)
			  {
				  lLine[i]=j+1;
				  break;
			  }
		                 //更新中点值 方便搜索 上一行
		
		if(imgbuff[i*40+Mid/8]+pow[7-Mid%8]<256)
			{
				lSum=i;                               //当 上面为 黑边的时候跳出 寻中线
				Mid=(lLine[i]+rLine[i])/2;
				MidLine[i]=Mid;
				  //  imgbuff[i*40+lLine[i]/8]+=pow[7-lLine[i]%8];
				//	imgbuff[i*40+rLine[i]/8]+=pow[7-rLine[i]%8];

//				imgbuff[i*40+Mid/8]+=pow[7-Mid%8];   //显示中线    通过加法来显示中线  化0为 一
		    }else break;
	  


		// 弯道平滑处理  补弯道    实质上很不完善， 对于掉线较为严重的弯道没有用
		int Mid2;
		int up=TOP,down=BOTTOM;
	  		

        if(pMid-Mid>20 && pMid!=-1)									//当两个中点相差较大 则说明出现跳点
		{
			for(j=i-1;j>=TOP;j--)
				if(imgbuff[j*40+lLine[i+1]/8]&pow[7-lLine[i+1]%8]!=0)		//看笔记图
				{
					up=j;
					break;
				}
			for(j=i-1;j<=BOTTOM;j++)
				if(imgbuff[j*40+lLine[i+1]/8]&pow[7-lLine[i+1]%8]!=0)
				{
					down=j;
					break;
				}
				Mid2=(up+down)/2;
				for(j=i;j>=Mid2;j--)
					MidLine[j]=MidLine[j+1]+(lLine[i]-pMid)/(i+1-Mid2);
				lSum=Mid2;

			break;
		}



		if(Mid-pMid>20 && pMid!=-1)
		{
			for(j=i-1;j>=TOP;j--)
				if(imgbuff[j*40+rLine[i+1]/8]&pow[7-rLine[i+1]%8]!=0)
				{
					up=j;
					break;
				}
			for(j=i-1;j<=BOTTOM;j++)
				if(imgbuff[j*40+rLine[i+1]/8]&pow[7-rLine[i+1]%8]!=0)
				{
					down=j;
					break;
				}
				Mid2=(up+down)/2;
				for(j=i;j>=Mid2;j--)
					MidLine[j]=MidLine[j+1]+(rLine[i]-pMid)/(i+1-Mid2);
				lSum=Mid2;

			break;
		}





	  pMid=Mid;
	  }            
  





																		//在显示屏幕上显示 中线
  for(i=BOTTOM;i>=lSum;i--)
  {
    if(imgbuff[i*40+MidLine[i]/8]+pow[7-MidLine[i]%8]<256)
			{


				imgbuff[i*40+MidLine[i]/8]+=pow[7-MidLine[i]%8];   //显示中线    通过加法来显示中线  化0为 一
		    }
  }

     //================================= 工程代码直接 结束处 ======================================//
	for(j=0;j<=BOTTOM;j++)
   for(i=0;i<WIDTH/8;i++)
   {
     for(k=7;k>=0;k--)
	 {
		 if((imgbuff[j*40+i]&pow[k])==0)
                    ImageData[j][i*8+7-k]=WHITE;
        else
			ImageData[j][i*8+7-k]=BLACK;
	 }		 
   }  


   //============================= 特殊标志显示处


     /* for(i=0;i<4;i++)
	   ImageData[ex[i]][ey[i]]=RED;*/
	//============================用户需要的仿真算法代码结束================================//
	UpdateImgData(pData);
	fclose(pfile);
}

//================================================自定义仿真算法区结束===========================================//

void SortBend(unsigned char imgbuff[],int pow[],int x,int y,int dir)
{
	int Mid=x-10;
	int i,j,up,down;
    for(i=y;i>=LEFTLINE;i--)
	{
		for(j=Mid;j>=0;j--)
			if((imgbuff[j*40+i/8]&pow[7-i%8])== 0)
			{
				up=j;
				break;
			}
		for(j=Mid;j<BOTTOM;j++)
		{
			if((imgbuff[j*40+i/8]&pow[7-i%8])==0)
			{
				down=j;
				break;
			}
		}
	Mid=(up+down)/2;
	imgbuff[Mid*40+i/8]-=pow[7-i%8];
	}
	
}