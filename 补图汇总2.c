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
	  		

        if(pMid-Mid>10 && pMid!=-1)									//当两个中点相差较大 则说明出现跳点
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



		if(Mid-pMid>10 && pMid!=-1)
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
