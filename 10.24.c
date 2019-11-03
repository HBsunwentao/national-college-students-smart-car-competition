// AlgorithmDLL.cpp : Defines the initialization routines for the DLL.
//

/*
10.24�Ľ��ĵط�
��uncontinuous���жϸı��б�ʵı仯
���� ��ֱ��ֱ ��ֱ���ж�  �����ڷ��� ���ڿ���ɾȥ��
�Ľ�ʮ�ֵ��ߵ����� ��������� ��  ʮ�ֵ���
*/

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



//���ӵ���λ������ʾ���㷨���ƣ�ֻ���޸��������ݼ���
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
// ����Ϊ����ִ���㷨������Ѵ������ͼ�������͵���λ����ʾ
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
//���ܣ�ͼ���������㷨����
//������wͼ�������ͼ���������------>��λ������ʱ�ṩͼ��Ŀ���
//      hͼ��ߣ���ͼ���������------>��λ������ʱ�ṩͼ��ĸ߶�
//      pData ��λ�����øú���ʱ�ṩ��ʾͼ�񻺳����Ŀ�ʼָ��
//================================================================



int pow[8]={1,2,4,8,16,32,64,128};                    //����ȫ�ֱ�����ʼ��
    #define BOTTOM 127  //�����㷨128�� �����е�λ��
	#define WIDTH 320   //ͼ�����
    #define LEFTLINE 0   //ͼ����߽�
    #define RIGHTLINE 319 //ͼ���б߽�
    #define TOP 0        //�����㷨128�� ��ʼ�е�λ��
	#define HEIGHT 128    //ͼ�������
	
	
int rLine[HEIGHT],lLine[HEIGHT];           //�洢���ұ߽�
int MidLine[140];							//�洢 ���� ������
int lSum=BOTTOM;							// ��¼ ���� ���� �� Ԫ�ظ���
int ex[4],ey[4];                            // ��¼ʮ��4���������õ�

//.............����ͼ���ж� ���õı���
  int Left_Lost_Flag=0,Right_Lost_Flag=0;   //ԭ����� ���� ���ұ߽���������������
  int Left_Lost_Flag2=0,Right_Lost_Flag2=0;   //��Ծʽ���ߣ�רע��ʮ�ֵĵ���
  int Left_Lost_Start2=-1,Left_Lost_End2=-1,Right_Lost_Start2=-1,Right_Lost_End2=-1;
  
  int Left_Lost_Start=-1,Left_Lost_End=-1,Right_Lost_Start=-1,Right_Lost_End=-1;
  int Left_Continuous_Flag=1,Right_Continuous_Flag=1,Left_Continuous_Delta=0,Right_Continuous_Delta=0;
  int Left_Bend_Flag=0,Right_Bend_Flag=0;
  int Left_Grad[250],Right_Grad[250];    // һ�׵���
  int Left_Grad_Sum=0,Right_Grad_Sum=0;        //�ж� ʮ�������� ��������
  int Left_Grad2[250],Right_Grad2[250];  //���׵���
  int Left_Straint_Flag=1,Right_Straint_Flag=1;  // �����Ƿ���ֱ���ж�
  int Cross_Flag=0,Left_Cross_Flag=0,Right_Cross_Flag=0,Strait_Cross_Flag=0;  //ʮ�������ж�
  int Half_Cross_Flag=0;
  int lSum2=20;
  
  
void _stdcall HandleImg(void *pData,ULONG w,ULONG h)
{
	InitImgData(pData,w,h);

	//============================�û���Ҫ���ӵ��㷨������뿪ʼ=============================//



	
	
	int i,j,k;
    unsigned char imgbuff[240*320/8];  //���������άתһά�Ĺ��ߣ�����,��ע����� ����ͼ��ֻ��128��

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
  
  
                               
int Mid=160;


void ArrayClear(int buff[],int x1,int x2);
void DrawLine2(unsigned char buff[],int x1,int y1,int x2,int y2);  //��ԭͼ���ϲ���
void LineSearch(unsigned char buff[]);                             //�߽� ���е� ������
void Strait_Cross_Left(unsigned char buff[]);                       //ֱʮ�� ��߶���
void Strait_Cross_Right(unsigned char buff[]);                   //ֱʮ�� �ұ� �Ķ�����
void Right_Cross_Right(unsigned char buff[]);  //����������ʮ�ַǳ����ң����ұ� ����
void Left_Cross_Left(unsigned char buff[]);    // ����������ʮ�ַǳ���������� ����
void Right_Cross_All_White(unsigned char buff[]);  //ʮ�����±� ȫ�׵����
void Left_All_White(unsigned char buff[]);   // ʮ�����±�ȫ�׵����
void DrawLine(int buff[],int x1,int y1,int x2,int y2);
void DrawLine3(unsigned char buff[],int x1,int y1,float K,float B); //��ԭͼ�������²�����
//...........�߽�ƽ������..............//
//δ���

                LineSearch(imgbuff);                             //�߽�����


//..........................����ͼ�����ǰ�ж�............................//

  if(lSum>TOP+20)lSum2=lSum;

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
	  //............�ж� ���ұ߽��Ƿ�  ��������
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
	  //...........��¼����
	  if( lLine[i-1]==LEFTLINE && !Left_Lost_Flag){Left_Lost_Flag=1;Left_Lost_Start=i;}
	  if(lLine[i]==LEFTLINE && lLine[i-1]!=LEFTLINE && Left_Lost_Flag){Left_Lost_End=i;}
	  //...........��¼�Ҷ���
	  if( rLine[i-1]==RIGHTLINE && !Right_Lost_Flag){Right_Lost_Flag=1;Right_Lost_Start=i;}
	  if(rLine[i]==RIGHTLINE && rLine[i-1]!=RIGHTLINE&& Right_Lost_Flag){Right_Lost_End=i;}
	  
	  //ʮ����Ծʽ���� �ж�
	  //...........��¼����
	  if( lLine[i-1]==LEFTLINE && !Left_Lost_Flag2 && lLine[i]>5+LEFTLINE ){Left_Lost_Flag2=1;Left_Lost_Start2=i;}
	  if(lLine[i]==LEFTLINE && lLine[i-3]>LEFTLINE+5 && Left_Lost_Flag2){Left_Lost_End2=i;}
	  //...........��¼�Ҷ���
	  if(rLine[i-1]==RIGHTLINE && !Right_Lost_Flag2 && rLine[i]<RIGHTLINE-5){Right_Lost_Flag2=1;Right_Lost_Start2=i;}
	  if(rLine[i]==RIGHTLINE && rLine[i-3]<RIGHTLINE-5 && Right_Lost_Flag2){Right_Lost_End2=i;}
  }
  if(Left_Lost_End2==-1)Left_Lost_Flag2=0;   //��Ծ���ߵĲ����ж�  ��Ҫ���¶�����ѡ�� 
  if(Right_Lost_End2==-1)Right_Lost_Flag2=0;

  Left_Grad[BOTTOM]=0;Right_Grad[BOTTOM]=0;
  Left_Grad2[BOTTOM]=0;Right_Grad2[BOTTOM]=0;
  for(i=BOTTOM-1;i>BOTTOM-4;i--)
  {
     Left_Grad[i]=lLine[i]-lLine[i+1];
	 Left_Grad2[i]=Left_Grad[i]-Left_Grad[i+1];

	  Right_Grad[i]=rLine[i]-rLine[i+1];
	  Right_Grad2[i]=Right_Grad[i]-Right_Grad[i+1];
  }

  for(i=BOTTOM-4;i>=lSum;i--)   //��߽��� ���� һ�׵��� �� ���׵���  ע�⣺��ʼ�� 4�� û�� ����
  {
	  Left_Grad[i]=lLine[i]-lLine[i+4];
	  Left_Grad2[i]=Left_Grad[i]-Left_Grad[i+2];
	  
	  Right_Grad[i]=rLine[i]-rLine[i+4];
	  Right_Grad2[i]=Right_Grad[i]-Right_Grad[i+2];
  }

  //for(i=BOTTOM;i>lSum2+10;i--)  //���� ���� 10�� ���� ���,���⽫��ǰ�� ����20 �У�������С s �����
  //{
	//  if(Left_Grad2[i]>10 || Left_Grad2[i] <-10)Left_Uncontinuous_Flag=1;
  //    if(Right_Grad2[i]>10 || Right_Grad2[i] <-10)Right_Uncontinuous_Flag=1;
	//  Left_Grad_Sum+=Left_Grad[i];
	//  Right_Grad_Sum+=Right_Grad[i];
 // }
 //ʹ���µ��ж��Ƿ���ֽϴ�Ӷ��� flag�ж��㷨:б��
  //ʹ���µ�flag�ж�ֱ�� ������
 for(i=BOTTOM;i>lSum2+3;i--)
 {
	 if(Left_Grad[i]*Left_Grad[i+3]<0&& !Left_Uncontinuous_Flag)
	 {
		 Left_Uncontinuous_Flag=1;
		 str.Format("Left_Uncontinuous_X=%d\n\r",i+3);PrintDebug(str);
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

 
 
 
  // �ж� �Ƿ��ǳ���ʮ��
 
  
  if(Left_Uncontinuous_Flag && Right_Uncontinuous_Flag &&(Left_Lost_Flag || Right_Lost_Flag) )Cross_Flag=1;
   if((!Left_Continuous_Flag) && (!Right_Continuous_Flag) &&((Left_Lost_Flag2 && (Left_Lost_Start2-Left_Lost_End2)>20 )||(Right_Lost_Flag2 &&(Right_Lost_Start2-Right_Lost_End2)>20)))Cross_Flag=1;
  //if((Left_Lost_Flag && (Left_Lost_Start-Left_Lost_End)>20 )&&(Right_Lost_Flag &&(Right_Lost_Start-Right_Lost_End)>20))Cross_Flag=1;
    
   if(Left_Lost_Flag &&Right_Lost_Flag)Cross_Flag=1;
 

  //�ų�ʮ������ ��������
  //if(Left_Uncontinuous_Flag&& !Left_Straint_Flag)Cross_Flag=0;
 // if(Right_Uncontinuous_Flag && !Right_Straint_Flag)Cross_Flag=0;
  
  
  
 // ������ʮ��  �������߿϶��������� ���� ��������Ҫ�����
  if(Cross_Flag)
  {
	  Left_Continuous_Flag=0;
	  Right_Continuous_Flag=0;
  }

  //�ж���ʲôʮ��
  if(Cross_Flag)
  {
	  if(Right_Lost_Flag && !Left_Lost_Flag)Right_Cross_Flag=1;else
	  if(!Right_Lost_Flag && Left_Lost_Flag)Left_Cross_Flag=1;else
	  Strait_Cross_Flag=1;
	  
	  if((Right_Lost_Flag && Right_Lost_Start==BOTTOM)||(Left_Lost_Flag && Left_Lost_Start==BOTTOM))
		 Half_Cross_Flag=1; 
	  
  }






//*****************************��ʮ���õĴ���***********************************//

ex[0]=0;ex[1]=0;ex[2]=0;ex[3]=0;  //��¼4���������õ� ����


 if(Cross_Flag)  // ʮ��ʶ�� ����������
  {
    
	if(Strait_Cross_Flag && !Half_Cross_Flag)
	{
		Strait_Cross_Left(imgbuff);   //Ѱ�� ��ߵĶ���
		Strait_Cross_Right(imgbuff);//Ѱ���ұ߶�����
	}
	if(Left_Cross_Flag && !Half_Cross_Flag)
	{
		Left_Cross_Left(imgbuff);
		Strait_Cross_Right(imgbuff);
	}
	
	if(Right_Cross_Flag && !Half_Cross_Flag)
	{
		Strait_Cross_Left(imgbuff);
		Right_Cross_Right(imgbuff);
	}
	
	if(Half_Cross_Flag)
	{
		if(Left_Lost_Flag && Left_Lost_Start==BOTTOM)Left_All_White(imgbuff);else Strait_Cross_Left(imgbuff);
		
		if(Right_Lost_Flag && Right_Lost_Start==BOTTOM)Right_Cross_All_White(imgbuff);else Strait_Cross_Right(imgbuff);
	}


   
   } 



//.................. ������������...................
        LineSearch(imgbuff);
	  
	  
		// ���ƽ������  �����    ʵ���Ϻܲ����ƣ� ���ڵ��߽�Ϊ���ص����û����
		//...........................������õô���.........................................//
		
		if(Left_Continuous_Flag && Right_Lost_Flag)//��������� ���� �Ҷ��� ˵�������� �����
		{
			DrawLine(MidLine,Right_Lost_Start,MidLine[Right_Lost_Start],lSum,RIGHTLINE);
		}
		if(Right_Continuous_Flag && Left_Lost_Flag)
		{
			DrawLine(MidLine,Left_Lost_Start,MidLine[Left_Lost_Start],lSum,LEFTLINE);
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


   //============================= �����־��ʾ��


      for(i=0;i<4;i++)
	   ImageData[ex[i]][ey[i]]=RED;

	  for(i=LEFTLINE;i<=RIGHTLINE;i++)
	  {
		  ImageData[TOP+80][i]=98;
		  ImageData[lSum][i]=98;
	  }
	  for(i=BOTTOM;i>=lSum;i--)
	  {
		  ImageData[i][lLine[i]]=BLUE;
		  ImageData[i][rLine[i]]=GREEN;
		  
	  }
	  str.Format("Left_Continuous_Flag=%d\n\r",Left_Continuous_Flag);PrintDebug(str);
	  str.Format("Right_Continuous_Flag=%d\n\r",Right_Continuous_Flag);PrintDebug(str);

	  str.Format("Left_Uncontinuous_Flag=%d\n\r",Left_Uncontinuous_Flag);PrintDebug(str);
	  str.Format("Left_Straint_Flag=%d\n\r",Left_Straint_Flag);PrintDebug(str);

	  str.Format("Right_Uncontinuous_Flag=%d\n\r",Right_Uncontinuous_Flag);PrintDebug(str);

	  str.Format("Left_Lost_Flag=%d\n\r",Left_Lost_Flag);PrintDebug(str);
	  str.Format("Right_Lost_Flag=%d\n\r",Right_Lost_Flag);PrintDebug(str);
	  
	  str.Format("Left_Lost_Flag2=%d\n\r",Left_Lost_Flag2);PrintDebug(str);
	  str.Format("Right_Lost_Flag2=%d\n\r",Right_Lost_Flag2);PrintDebug(str);

	  str.Format("Cross_Flag=%d\n\r",Cross_Flag);PrintDebug(str);
	  str.Format("Left_Cross_Flag=%d\n\r",Left_Cross_Flag);PrintDebug(str);
	  str.Format("Right_Cross_Flag=%d\n\r",Right_Cross_Flag);PrintDebug(str);
	  str.Format("Strait_Cross_Flag=%d\n\r",Strait_Cross_Flag);PrintDebug(str);
	  str.Format("Half_Cross_Flag=%d\n\r",Half_Cross_Flag);PrintDebug(str);

	  str.Format("Left_Grad_Sum=%d\n\r",Left_Grad_Sum);PrintDebug(str);
	  str.Format("Right_Grad_Sum=%d\n\r",Right_Grad_Sum);PrintDebug(str);
	  
      
	  for(i=BOTTOM;i>=lSum2;i--)   // ��ʾ ���� ֵ 
	  {
		  str.Format("Left_Grad[%d]=%d\n\r",i,Left_Grad[i]);
		  PrintDebug(str);
	  }
	     str.Format("\n\n");
		 PrintDebug(str);

	  for(i=BOTTOM;i>=lSum2;i--)
	  {
		  str.Format("Left_Grad2[%d]=%d\n\r",i,Left_Grad2[i]);
		  PrintDebug(str);
	  }
		  

	  for(i=BOTTOM;i>=lSum2;i--)  //��ʾ�ҵ���ֵ
	  {
		  str.Format("Right_Grad[%d]=%d\n\r",i,Right_Grad[i]);
		  PrintDebug(str);
	  }
	     str.Format("\n\r");
		 PrintDebug(str);

	  for(i=BOTTOM;i>=lSum2;i--)
	  {
		  str.Format("Right_Grad2[%d]=%d\n\r",i,Right_Grad2[i]);
		  PrintDebug(str);
	  }

	//============================�û���Ҫ�ķ����㷨�������================================//
	UpdateImgData(pData);
	fclose(pfile);
}

//================================================�Զ�������㷨������===========================================//
void DrawLine(int buff[],int x1,int y1,int x2,int y2)  //�� �����ϲ���
{
	float K,B;
	int i;
	K=(y1-y2)*1.0/(x1-x2);
	B=y1-K*x1;

	for(i=x2;i<=x1;i++)
		buff[i]=K*i+B;
}

void DrawLine2(unsigned char buff[],int x1,int y1,int x2,int y2)  //��ԭͼ���ϲ���
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

void DrawLine3(unsigned char buff[],int x1,int y1,float K,float B) //��ԭͼ�������²�����
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
      for(i=BOTTOM;i>=TOP;i--)              //����������������
	  {
		  rLine[i]=RIGHTLINE;              //��ʼ��  280 �� ��  40 ��  ��С��Ұ
		  lLine[i]=LEFTLINE;
		  for(j=Mid;j<=RIGHTLINE;j++)
			  if((buff[i*40+j/8]&pow[7-j%8])!=0)   //���м�������Ѱ�� ����  1Ϊ�ڱ�
			  {
				  rLine[i]=j-1;                        //��¼����λ��
				  break;
			  }
		  for(j=Mid;j>=LEFTLINE;j--)
			  if((buff[i*40+j/8]&pow[7-j%8])!=0)
			  {
				  lLine[i]=j+1;
				  break;
			  }
		                 //�����е�ֵ �������� ��һ��
		
		if(buff[i*40+Mid/8]+pow[7-Mid%8]<256)
			{
				lSum=i;                               //�� ����Ϊ �ڱߵ�ʱ������ Ѱ����
				Mid=(lLine[i]+rLine[i])/2;
				MidLine[i]=Mid;

				
		    }else break;
	  

	  } 
}

void Strait_Cross_Left(unsigned char buff[])        //ֱʮ�� ��߶���
{
	int pUp,nUp=0;    //Ѱ�������������õ� ��¼�ϲ�ı���
	pUp=BOTTOM;
	int t=0,delta=0,Mid;
	int i,j;
for(i=lLine[BOTTOM];i<=RIGHTLINE;i++) //Ѱ�� ���µ�   ����������
	{
		for(j=BOTTOM;j>=TOP;j--)
			if((buff[j*40+i/8]&pow[7-i%8])!=0)
			{
			 nUp=j; 
             break;			 
			}
	if(pUp-nUp>10  && pUp!=-1)				//������ �ϱ߽������� ����10 ��Ϊ�ҵ� ���� ��¼���� �˳�Ѱ��
	{
		ex[0]=pUp;
		ey[0]=i-1;
		t++;
		break;	
	}
	pUp=nUp;
	}
	
	

        //***************����ҵ����µ�������ϵ�
	if(t==1){										//���ҵ�������ĵ�  �Ϳ�ʼѰ�����������
    nUp=0;
	pUp=-1;
 Mid=ex[0]-10;if(Mid<TOP)Mid=TOP;
	for(i=ey[0];i<=RIGHTLINE;i+=3)					//���� �������� ���Ϸ� �� �Ҳ���Ѱ�� ���������   ȱ�㣺��ʮ�ַǳ�ƫ ������������������ �����ƫ��� һ��
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
    	  if((buff[Mid*40+i/8]&pow[7-i%8])!=0)break;            // ����װ�� ����ֹ�ѽ��߽�����ȥ��  ���� ��� ����ʮ�� ��80%������ 
	}

    if(t==2)DrawLine2(buff,ex[0],ey[0],ex[3],ey[3]);
	}
	
}
void Strait_Cross_Right(unsigned char buff[])                   //ֱʮ�� �ұ� �Ķ�����
{
	int pUp,nUp=0;    //Ѱ�������������õ� ��¼�ϲ�ı���
	pUp=BOTTOM;
	int t=0,delta=0,Mid;
	int i,j;
	
    t=0;
	nUp=0;
	pUp=BOTTOM;
//	if(rLine[BOTTOM]!=RIGHTLINE)
	for(i=rLine[BOTTOM];i>=LEFTLINE;i--) //Ѱ�� ���µ�
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
	


	            //..........����ҵ����µĵ� ���� ���ϵĵ�

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

		
		
	if(t==2)DrawLine2(buff,ex[1],ey[1],ex[2],ey[2]);  //��ԭͼ�ϲ�ȫʮ��
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
			if(buff[j*40+i/8]&pow[7-j%8]!=0)
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
	if(t==2)DrawLine2(buff,ex[0],ey[0],ex[3],ey[3]);  //��ԭͼ�ϲ�ȫʮ��
}

void Right_Cross_Right(unsigned char buff[])  //����������ʮ�ַǳ����ң����ұ� ����
{
	int Mid=(Right_Lost_Start+Right_Lost_End)/2;
	int i,j,t=1;
	int pUp=-1,nUp=0;
	ex[1]=Right_Lost_Start;
	ey[1]=rLine[ex[1]];
	
	for(i=RIGHTLINE;i>=LEFTLINE;i--)
	{
		for(j=Mid;j>=TOP;j--)
			if(buff[j*40+i/8]&pow[7-j%8]!=0)
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
	if(t==2)DrawLine2(buff,ex[1],ey[1],ex[2],ey[2]);  //��ԭͼ�ϲ�ȫʮ��

}

void Left_All_White(unsigned char buff[])   // ʮ�����±�ȫ�׵����
{
	float K,B;
	int i,j;
	int pUp=-1,nUp=0;
	for(i=LEFTLINE;i<=RIGHTLINE;i+=4)
	{
		for(j=BOTTOM;j>=TOP;j--)
			if(buff[j*40+i/8]&pow[7-j%8]!=0)
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
void Right_Cross_All_White(unsigned char buff[])  //ʮ�����±� ȫ�׵����
{
	float K,B;
	int i,j;
	int pUp=-1,nUp=0;
	for(i=RIGHTLINE;i>=LEFTLINE;i-=4)
	{
		for(j=BOTTOM;j>=TOP;j--)
			if(buff[j*40+i/8]&pow[7-j%8]!=0)
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
