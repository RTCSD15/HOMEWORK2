#include "class_dec.h"
#include "auxiliary.h"

myRobot robot;

int main()
{
    while(1)
        {
            robot.Operation();
        }   
}

/*************************************************************************************************/
/*@note
    這個文件里定義了所有的類的成員函數 
*/

/**includes***************************************************************************************/ 
#include "auxiliary.h"
#include "class_dec.h"

/**CS::Fuctions***********************************************************************************/

/*@note
    CS的構造函數 
*/
CS::CS()
    {
        strcpy(name,"TCS");
        x=0;
        y=0;
        angle=0;
        //mov(3,3);
    }

/*@note
    用來設定新建坐標系相對於世界坐標系的平移、旋轉量
*/  
void CS::Set(char n[11],float a,float b,float c)
    {
        strcpy(name,n);
        x=a;
        y=b;
        angle=c;
        /*mov(0,0)=cos(angle);
        mov(0,1)=sin(angle);
        mov(0,2)=x;
        mov(1,0)=-sin(angle);
        mov(1,1)=cos(angle);
        mov(1,2)=y;
        mov(2,0)=0;
        mov(2,1)=0;
        mov(2,2)=1;*/
    }

/*@note
    獲取坐標系名稱
*/  
char* CS::GetName()
    {
        return name;
    }

/*@note
    插入該坐標系下用戶要求機器人到達的坐標
*/  
void CS::Insert(float a,float b)
    {
        para1=a;
        para2=b;
    }   

/*@note
    將該坐標系下的坐標變換至世界坐標系并更新關節坐標系中的坐標值
*/ 
void CS::Transform()
    {
        MatrixXd mov(3,3);
        mov(0,0)=cos(angle);
        mov(0,1)=sin(angle);
        mov(0,2)=x;
        mov(1,0)=-sin(angle);
        mov(1,1)=cos(angle);
        mov(1,2)=y;
        mov(2,0)=0;
        mov(2,1)=0;
        mov(2,2)=1;
        MatrixXd coor_TCS(3,1);
        coor_TCS(0,0)=para1;
        coor_TCS(1,0)=para2;
        coor_TCS(2,0)=1;
        MatrixXd coor_WCS(3,1);
        coor_WCS=mov*coor_TCS;
        if((coor_WCS(0,0)*coor_WCS(0,0)+coor_WCS(1,0)*coor_WCS(1,0))>=400)    //這裡假定機器人兩個手臂的長度都是10，所以半徑20的圓以外的地方是到不了的 
            {
                cout<<"Robot can't reach that point!"<<endl;
            }
        else
            {
                cout<<"The coordinates of the Robot in WCS are shown as below:"<<endl<<"("<<coor_WCS(0,0)<<","<<coor_WCS(1,0)<<")"<<endl;
                robot.Set(acos(sqrt(coor_WCS(0,0)*coor_WCS(0,0)+coor_WCS(1,0)*coor_WCS(1,0))/20)+atan(coor_WCS(1,0)/coor_WCS(0,0)),\
                2*(90-acos(sqrt(coor_WCS(0,0)*coor_WCS(0,0)+coor_WCS(1,0)*coor_WCS(1,0))/20)));
            }
    }

/**myRobot::Functions*****************************************************************************/

/*@note
    構造函數
*/ 
myRobot::myRobot()
    {
        angle1=90;
        angle2=180;
        CS* p;
        p=new CS;
        char str[3]={'W','C','S'};
        p->Set(str,0,0,0);
        cs_vector.push_back(*p);
    }
    
/*@note
    更新關節坐標系下的坐標值
*/  
void myRobot::Set(float a,float b)
    {
        angle1=a;
        angle2=b;
    }

/*@note
    判斷用戶是新建坐標系還是移動機器人
*/  
void myRobot::Operation()
    {
        input=Input();
        if(strcmp(input.OperationType,"S")==0)
            {
                CS* p;
                p=new CS;
                p->Set(input.CSName,input.para1,input.para2,input.para3);
                cs_vector.push_back(*p);
            }
        else
            {
                PTPMove();
            }       
    }

/*@note
    實現坐標變換并更新關節坐標系下坐標
*/ 
void myRobot::PTPMove()
    {
        for(it=cs_vector.begin();it!=cs_vector.end();it++)
            {
                if(strcmp(it->GetName(),input.CSName)==0)
                    {
                        it->Insert(input.para1,input.para2);
                        it->Transform();
                        Show();
                        break;
                    }
            }
    }

/*@note
    顯示關節坐標系下坐標
*/  
void myRobot::Show()
    {
        cout<<"The coordinates of the Robot in Joint_CS are shown as below:"<<endl<<"("<<angle1<<","<<angle2<<")"<<endl;
    }
    
#ifndef __CLASS_DEC_H
#define __CLASS_DEC_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;

class CS
    {
        public:
            CS();
            void Set(char n[11],float a,float b,float c);
            char* GetName();
            void Insert(float a,float b);
            void Transform();
        protected:
            char name[11];
            float x;
            float y;
            float angle;
            //MatrixXd mov;
            float para1;
            float para2;
    };
    
class myRobot
    {
        public:
            myRobot();
            void Set(float a,float b);
            void Operation();
            void PTPMove(); 
            void Show();        
        protected:
            float angle1;
            float angle2;
            vector<CS> cs_vector;
            vector<CS>::iterator it;
    };
    
extern myRobot robot;
    
#endif  

#include "auxiliary.h"

InputCom input;

InputCom Input()
    {
        char str[11];
        InputCom input;
        cout<<"Input OperationType 'S'or'M':"<<endl;
        cin>>str;
        strcpy(input.OperationType,str);
        cout<<"Input CSName:"<<endl;
        cin>>str;
        strcpy(input.CSName,str);
        cout<<"Input para1:"<<endl;
        cin>>str;
        input.para1=Extraction(str);
        cout<<"Input para2:"<<endl;
        cin>>str;
        input.para2=Extraction(str);
        if(strcmp(input.OperationType,"S")==0)
            {
                cout<<"Input para3:"<<endl;
                cin>>str;
                input.para3=Extraction(str);
                return input;
            }
        else
            {
                input.para3=0;
                return input;
            }
    }

float Extraction(char str[11])
    {
        int i,k;
        float a=0;
        for(i=0;str[i]!='\0';i++)
            {
                if(str[i]=='.')
                    {
                        k=i;
                    }
            }
        if(str[0]=='-')
            {
                for(i=0;i<k-1;i++)
                    {
                        a+=((float)str[k-i-1]-48)*Index1(i);
                    }
                for(i=1;str[k+i]!='\0';i++)
                    {
                        a+=((float)str[k+i]-48)*Index2(i);
                    }
            }
        else
            {
                for(i=0;i<k;i++)
                    {
                        a+=((float)str[k-i-1]-48)*Index1(i);
                    }
                for(i=1;str[k+i]!='\0';i++)
                    {
                        a+=((float)str[k+i]-48)*Index2(i);
                    }
            }
        return a;
    }

float Index1(int n)
    {
        float a=1,i;
        for(i=n;i!=0;i--)
            {
                a*=10;
            }
        return a;
    }

float Index2(int n)
    {
        float a=1,i;
        for(i=n;i!=0;i--)
            {
                a*=0.1;
            }
        return a;
    }
    
#ifndef __AUXILIARY_H
#define __AUXILIARY_H

#include <iostream>
#include <string.h>

using namespace std;

/*@note
    用於存儲用戶輸入的內容
*/ 
typedef struct
    {
        char OperationType[11];
        char CSName[11];
        float para1;
        float para2;
        float para3;
    }InputCom;

extern InputCom input;

InputCom Input();
float Extraction(char str[40]);
float Index1(int n);
float Index2(int n);

#endif
