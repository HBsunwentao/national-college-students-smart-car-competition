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


//˵�����û�ֻ��Ҫ�޸ģ�ȫ�ֱ����� �� �Զ�������㷨�� �Ϳ��Խ����㷨�������Ŀ���
//�������������޸ġ�



//=============================ȫ�ֱ�����==================================//

#define IMG_ROWS 128      //ͼ���������������
#define IMG_COLS 320      //ͼ���������������
#define WHITE 255        //�궨���
#define RED 128        //�궨��
#define BLUE 254        //�궨��
#define GREEN 100        //�궨��
#define BLACK 0          //�궨���
#define THRESHOLD 100    //�궨����ֵ

unsigned char ImageData[IMG_ROWS][IMG_COLS];//ͼ������

//=========================================================================//


char* pFileName = "��������ȡ��Ϣ.txt";
FILE *pfile = fopen(pFileName,"w");
CString str="";



//��ӵ���λ������ʾ���㷨���ƣ�ֻ���޸��������ݼ���
void _stdcall AlgorithmName(char *pName)
{
	char *p = "The best";
	strcpy(pName,p);
}

// ��ʼ��ͼ������
// �öδ��벻�����κ��޸�,����Ϊ����λ��ȡ������䵽ImageData
// ����ͬ��λ���Ĳɼ�����һ����Ч��.
void InitImgData(void *pData,int w,int h)
{
	int i=0,j=0;
	// ���õ�ͼ���С����λ����ͼ���С��һ�£������������������޸ĺ궨���IMG_ROWS��IMG_COLS��Ҫ����λ��ͼ���Сһ��
	if (w != IMG_COLS || h != IMG_ROWS)
	{
		AfxMessageBox("ͼ���С���ò�����");
	}
	for (i = 0;i < IMG_ROWS;i++)
	{
		for (j = 0;j < IMG_COLS;j++)
		{
			ImageData[i][j] = *((UCHAR*)pData+i* IMG_COLS +j);
		}
	}
}


// �öδ��벻�����κ��޸�
// ����Ϊ����ִ���㷨�����Ѵ�����ͼ�������͵���λ����ʾ
// ��λ������Ҫ��ʾ������pDataָ��Ϊ�׵�ַ����������
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



//================================================�Զ�������㷨����ʼ===========================================//

//=================================================================
//���ܣ�ͼ��������㷨����
//������wͼ�����ͼ���������------>��λ������ʱ�ṩͼ��Ŀ��
//      hͼ��ߣ���ͼ���������------>��λ������ʱ�ṩͼ��ĸ߶�
//      pData ��λ�����øú���ʱ�ṩ��ʾͼ�񻺳����Ŀ�ʼָ��
//================================================================
void _stdcall HandleImg(void *pData,ULONG w,ULONG h)
{
	InitImgData(pData,w,h);

	//============================�û���Ҫ��ӵ��㷨������뿪ʼ=============================//
	#define BOTTOM 127  //�����㷨128�� �����е�λ��
	#define WIDTH 320   //ͼ����
    #define LEFTLINE 0   //ͼ����߽�
    #define RIGHTLINE 319 //ͼ���б߽�
    #define TOP 0        //�����㷨128�� ��ʼ�е�λ��
	#define HEIGHT 128    //ͼ�������
   
	unsigned char imgbuff[240*320/8];  //��������άתһά�Ĺ��ߣ�����,��ע����� ����ͼ��ֻ��128��
	int pow[8]={1,2,4,8,16,32,64,128};
	int i,j,k;

                             //ע�����  ��λ�� ��0��ʾ��  �� ����ͷ  1��ʾ �ڲ�ͬ  ����� Ҫ��ǰת��
	for(j=0;j<BOTTOM;j++)  //��ɨ��
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
  
    



float K,B;
int Mid=160;
int rLine[HEIGHT],lLine[HEIGHT];
int lSum=BOTTOM;
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
				    //imgbuff[i*40+lLine[i]/8]-=pow[7-lLine[i]%8];
					//imgbuff[i*40+rLine[i]/8]-=pow[7-rLine[i]%8];

				//imgbuff[i*40+Mid/8]+=pow[7-Mid%8];   //��ʾ����    ͨ���ӷ�����ʾ����  ��0Ϊ һ
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

	if(lLine[BOTTOM]!=0)
	for(i=lLine[BOTTOM];i<RIGHTLINE;i++) //Ѱ�� ���µ�   ����������
	{
		for(j=BOTTOM;j>=TOP;j--)
			if((imgbuff[j*40+i/8]&pow[7-i%8])!=0)
			{
			 nUp=j; 
             break;			 
			}
	if(pUp-nUp-delta>10)
	{
		ex[0]=pUp;
		ey[0]=i-1;
		t=1;
		break;	
	}
	pUp=nUp;
	}
	
	
	for(i=BOTTOM;i>lSum+1;i--)                //����������
{
	if(lLine[i]-lLine[i-1]>10)
	{
		ex[0]=i;
		ey[0]=lLine[i]; 
		t=2;
		break;	
	}
}
        //***************����ҵ����µ�������ϵ�
	if(t==1){  
    nUp=0;
	pUp=-1;
	delta=0;
 Mid=ex[0]-10;
	for(i=ey[0];i<RIGHTLINE;i+=3)
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
	}

		K=(ey[3]-ey[0])*1.0/(ex[3]-ex[0]);
		B=ey[0]-K*ex[0];
		for(i=ex[3];i<=ex[0];i++)
		{
			lLine[i]=K*i+B;
			if(imgbuff[i*40+lLine[i]/8]+pow[7-lLine[i]%8]<256)imgbuff[i*40+lLine[i]/8]+=pow[7-lLine[i]%8];

		}
	}
	
	else 
		if(t==2)                 //��������ѵ�������   �������߽� �ſ�
		{
    nUp=0;
	pUp=-1;
	delta=0;
 Mid=ex[0]-10;
	for(i=ey[0]-10;i<RIGHTLINE;i+=3)
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
	}

		K=(ey[3]-ey[0])*1.0/(ex[3]-ex[0]);
		B=ey[0]-K*ex[0];
		for(i=ex[3];i<=ex[0];i++)
		{
			lLine[i]=K*i+B;
			if(imgbuff[i*40+lLine[i]/8]+pow[7-lLine[i]%8]<256)imgbuff[i*40+lLine[i]/8]+=pow[7-lLine[i]%8];

		}
		}







//**********Ѱ���ұ߶�����

	t=0;
	nUp=0;
	pUp=BOTTOM;
	if(rLine[BOTTOM]!=RIGHTLINE)
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
		t=1;
		break;	
	}
	
	pUp=nUp;
	}
	
for(i=BOTTOM;i>lSum;i--)
{
	if(rLine[i-1]-rLine[i]>10)
	{
		ex[1]=i;
		ey[1]=rLine[i]; 
		t=2;
		break;	
	}
}

	            //..........����ҵ����µĵ� ���� ���ϵĵ�

	if(t==1)
	{
 
	Mid=ex[1]-10;
	pUp=-1;
	nUp=0;
	delta=0;
 for(i=ey[1];i>LEFTLINE;i-=3)
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
 
 }
 		K=(ey[2]-ey[1])*1.0/(ex[2]-ex[1]);
		B=ey[1]-K*ex[1];
		for(i=ex[2];i<=ex[1];i++)
		{
			rLine[i]=K*i+B;
			if(imgbuff[i*40+rLine[i]/8]+pow[7-rLine[i]%8]<256)imgbuff[i*40+rLine[i]/8]+=pow[7-rLine[i]%8];
		}
	}
	else 

	//****************
	if(t==2){
    Mid=ex[1]-10;
	pUp=-1;
	nUp=0;
	delta=0;
 for(i=ey[1]+10;i>LEFTLINE;i-=3)
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
 
 }
 		K=(ey[2]-ey[1])*1.0/(ex[2]-ex[1]);
		B=ey[1]-K*ex[1];
		for(i=ex[2];i<=ex[1];i++)
		{
			rLine[i]=K*i+B;
			if(imgbuff[i*40+rLine[i]/8]+pow[7-rLine[i]%8]<256)imgbuff[i*40+rLine[i]/8]+=pow[7-rLine[i]%8];
		}
	}

	//*******************��  ��  ��  ��  ʼ*************************//









for(i=BOTTOM;i>lSum;i--)
{
	Mid=(lLine[i]+rLine[i])/2;
	if(imgbuff[i*40+Mid/8]+pow[7-Mid%8]<256)
		
			{
			//	imgbuff[i*40+Mid/8]+=pow[7-Mid%8];   //��ʾ����
		    }
}







     //================================= ���̴���ֱ�� ������ ======================================//
	for(j=0;j<BOTTOM;j++)
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



   //============================= �����־��ʾ��
      for(i=0;i<4;i++)
	   ImageData[ex[i]][ey[i]]=RED;


	//============================�û���Ҫ�ķ����㷨�������================================//
	UpdateImgData(pData);
	fclose(pfile);
}

//================================================�Զ�������㷨������===========================================//

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