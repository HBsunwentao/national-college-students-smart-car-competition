	//============================�û���Ҫ��ӵ��㷨������뿪ʼ=============================//
	#define BOTTOM 127  //�����㷨128�� �����е�λ��
	#define WIDTH 320   //ͼ����
    #define LEFTLINE 0   //ͼ����߽�
    #define RIGHTLINE 319 //ͼ���б߽�
    #define TOP 0        //�����㷨128�� ��ʼ�е�λ��
	#define HEIGHT 128    //ͼ�������


	void SortBend(unsigned char imgbuff[],int pow[],int x,int y,int dir);
	unsigned char imgbuff[240*320/8];  //��������άתһά�Ĺ��ߣ�����,��ע����� ����ͼ��ֻ��128��
	int pow[8]={1,2,4,8,16,32,64,128};
	int i,j,k;


	for(j=0;j<=BOTTOM;j++)  //��ɨ��
   for(i=0;i<WIDTH/8;i++)  //8��ÿ�� ��ɨ��
   {
	 imgbuff[j*40+i]=0;     //��ʼȫ����ֵΪ0 Ŀ�� �Ƿ������� λ���� ��λѹ��
     for(k=0;k<8;k++)      //8λѹ��
	 {
		 imgbuff[j*40+i]<<=1;  //��λ
		 if(ImageData[j][i*8+k]==0)   //�� ��λ�� �ڰױ�ʾ �� ��������ͷ ͳһ
		 imgbuff[j*40+i]|=1;
	 }	 
   }
	  
   //================================= ���̴���ֱ�� ��ʼ�� ======================================//
  
  
float K,B;                                 //���Ӷ��� �õ�  б�� �� �ؾ�
int Mid=160;
int rLine[HEIGHT],lLine[HEIGHT];           //�洢���ұ߽�
int MidLine[140];							//�洢 ���� ������
int lSum=BOTTOM;							// ��¼ ���� ���� �� Ԫ�ظ���

      for(i=BOTTOM;i>=TOP;i--)              //����������������
	  {
		  rLine[i]=RIGHTLINE;              //��ʼ��  280 �� ��  40 ��  ��С��Ұ
		  lLine[i]=LEFTLINE;
		  for(j=Mid;j<=RIGHTLINE;j++)
			  if((imgbuff[i*40+j/8]&pow[7-j%8])!=0)   //���м�������Ѱ�� ����  1Ϊ�ڱ�
			  {
				  rLine[i]=j-1;                        //��¼����λ��
				  break;
			  }
		  for(j=Mid;j>=LEFTLINE;j--)
			  if((imgbuff[i*40+j/8]&pow[7-j%8])!=0)
			  {
				  lLine[i]=j+1;
				  break;
			  }
		Mid=(lLine[i]+rLine[i])/2;                      //�����е�ֵ �������� ��һ��
		if(imgbuff[i*40+Mid/8]+pow[7-Mid%8]<256)
		
			{
				lSum=i;                               //���й�����ʱ��Ϊ����
				    //imgbuff[i*40+lLine[i]/8]+=pow[7-lLine[i]%8];
					//imgbuff[i*40+rLine[i]/8]+=pow[7-rLine[i]%8];

			//	imgbuff[i*40+Mid/8]+=pow[7-Mid%8];   //��ʾ����    ͨ���ӷ�����ʾ����  ��0Ϊ һ
		    }
	  
	  }










//*****************************��ʮ���õĴ���***********************************//
int ex[4],ey[4];
ex[0]=0;ex[1]=0;ex[2]=0;ex[3]=0;  //��¼4���������õ� ����
int t=0;
int pUp,nUp=0;    //Ѱ�������������õ� ��¼�ϲ�ı���
int delta=0;          //Ѱ�ҹյ��õ� ����������¼����



//**********Ѱ����߶�����
    pUp=BOTTOM;

	//if(lLine[BOTTOM]!=0)
for(i=lLine[BOTTOM];i<=RIGHTLINE;i++) //Ѱ�� ���µ�   ����������
	{
		for(j=BOTTOM;j>=TOP;j--)
			if((imgbuff[j*40+i/8]&pow[7-i%8])!=0)
			{
			 nUp=j; 
             break;			 
			}
	if(pUp-nUp-delta>10  && pUp!=-1)				//������ �ϱ߽������� ����10 ��Ϊ�ҵ� ���� ��¼���� �˳�Ѱ��
	{
		ex[0]=pUp;
		ey[0]=i-1;
		t++;
		break;	
	}
	pUp=nUp;
	}
	
	
/*	for(i=BOTTOM;i>lSum;i--)                //����������
{
	if(lLine[i]-lLine[i-1]>10)
	{
		ex[0]=i;
		ey[0]=lLine[i]; 
		t=2;
		break;	
	}
}*/
        //***************����ҵ����µ�������ϵ�
	if(t==1){										//���ҵ�������ĵ�  �Ϳ�ʼѰ�����������
    nUp=0;
	pUp=-1;
	delta=0;
 Mid=ex[0]-10;
	for(i=ey[0];i<=RIGHTLINE;i+=3)					//���� �������� ���Ϸ� �� �Ҳ���Ѱ�� ���������   ȱ�㣺��ʮ�ַǳ�ƫ ������������������ �����ƫ��� һ��
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
    	  if((imgbuff[Mid*40+i/8]&pow[7-i%8])!=0)break;            // ����װ�� ����ֹ�ѽ��߽�����ȥ��  ���� ��� ����ʮ�� ��80%������ 
	}


	}
	





		//**********Ѱ���ұ߶�����

	
	nUp=0;
	pUp=BOTTOM;
//	if(rLine[BOTTOM]!=RIGHTLINE)
	for(i=rLine[BOTTOM];i>=LEFTLINE;i--) //Ѱ�� ���µ�
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

	            //..........����ҵ����µĵ� ���� ���ϵĵ�

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






//.................. ������������...................
	Mid=(LEFTLINE+RIGHTLINE)/2;
	int pMid=-1;
      for(i=BOTTOM;i>=TOP;i--)              //����������������
	  {
		  rLine[i]=RIGHTLINE;              //��ʼ��  280 �� ��  40 ��  ��С��Ұ
		  lLine[i]=LEFTLINE;
		  for(j=Mid;j<=RIGHTLINE;j++)
			  if((imgbuff[i*40+j/8]&pow[7-j%8])!=0)   //���м�������Ѱ�� ����  1Ϊ�ڱ�
			  {
				  rLine[i]=j-1;                        //��¼����λ��
				  break;
			  }
		  for(j=Mid;j>=LEFTLINE;j--)
			  if((imgbuff[i*40+j/8]&pow[7-j%8])!=0)
			  {
				  lLine[i]=j+1;
				  break;
			  }
		                 //�����е�ֵ �������� ��һ��
		
		if(imgbuff[i*40+Mid/8]+pow[7-Mid%8]<256)
			{
				lSum=i;                               //�� ����Ϊ �ڱߵ�ʱ������ Ѱ����
				Mid=(lLine[i]+rLine[i])/2;
				MidLine[i]=Mid;
				  //  imgbuff[i*40+lLine[i]/8]+=pow[7-lLine[i]%8];
				//	imgbuff[i*40+rLine[i]/8]+=pow[7-rLine[i]%8];

//				imgbuff[i*40+Mid/8]+=pow[7-Mid%8];   //��ʾ����    ͨ���ӷ�����ʾ����  ��0Ϊ һ
		    }else break;
	  


		// ���ƽ������  �����    ʵ���Ϻܲ����ƣ� ���ڵ��߽�Ϊ���ص����û����
		int Mid2;
		int up=TOP,down=BOTTOM;
	  		

        if(pMid-Mid>10 && pMid!=-1)									//�������е����ϴ� ��˵����������
		{
			for(j=i-1;j>=TOP;j--)
				if(imgbuff[j*40+lLine[i+1]/8]&pow[7-lLine[i+1]%8]!=0)		//���ʼ�ͼ
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
  





																		//����ʾ��Ļ����ʾ ����
  for(i=BOTTOM;i>=lSum;i--)
  {
    if(imgbuff[i*40+MidLine[i]/8]+pow[7-MidLine[i]%8]<256)
			{


				imgbuff[i*40+MidLine[i]/8]+=pow[7-MidLine[i]%8];   //��ʾ����    ͨ���ӷ�����ʾ����  ��0Ϊ һ
		    }
  }

     //================================= ���̴���ֱ�� ������ ======================================//
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
