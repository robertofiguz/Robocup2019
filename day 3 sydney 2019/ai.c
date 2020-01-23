////////////////////////////////////////
//
//	File : ai.c
//	CoSpace Robot
//	Version 1.0.0
//	Jan 1 2016
//	Copyright (C) 2016 CoSpace Robot. All Rights Reserved
//
//////////////////////////////////////
//
// ONLY C Code can be compiled.
//
/////////////////////////////////////

#define CsBot_AI_H//DO NOT delete this line
#ifndef CSBOT_REAL
#include <windows.h>
#include <stdio.h>
#include <math.h>
#define DLL_EXPORT extern __declspec(dllexport)
#define false 0
#define true 1
#endif//The robot ID : It must be two char, such as '00','kl' or 'Cr'.
char AI_MyID[2] = {'0','2'};

int Duration = 0;
int SuperDuration = 0;
int bGameEnd = false;
int CurAction = -1;
int CurGame = 0;
int SuperObj_Num = 0;
int SuperObj_X = 0;
int SuperObj_Y = 0;
int Teleport = 0;
int LoadedObjects = 0;
int side = -1;
int moduledistX = 0;
int moduledistY = 0;
int i = 0;
int depoX = 346;
int depoY = 196;
int sideX = 0;
int sideY = 0;
int depo2X = 15;
int depo2Y = 71;
int moduledist2X = 0;
int moduledist2Y = 0;
int side2X = 0;
int side2Y = 0;
int US_Front = 0;
int US_Left = 0;
int US_Right = 0;
int CSLeft_R = 0;
int CSLeft_G = 0;
int CSLeft_B = 0;
int CSRight_R = 0;
int CSRight_G = 0;
int CSRight_B = 0;
int PositionX = 0;
int PositionY = 0;
int TM_State = 0;
int Compass = 0;
int Time = 0;
int WheelLeft = 0;
int WheelRight = 0;
int LED_1 = 0;
int MyState = 0;
int AI_SensorNum = 13;

#define CsBot_AI_C//DO NOT delete this line

DLL_EXPORT void SetGameID(int GameID)
{
    CurGame = GameID;
    bGameEnd = 0;
}

DLL_EXPORT int GetGameID()
{
    return CurGame;
}

//Only Used by CsBot Dance Platform
DLL_EXPORT int IsGameEnd()
{
    return bGameEnd;
}

#ifndef CSBOT_REAL

DLL_EXPORT char* GetDebugInfo()
{
    char info[1024];
    sprintf(info, "Duration=%d;SuperDuration=%d;bGameEnd=%d;CurAction=%d;CurGame=%d;SuperObj_Num=%d;SuperObj_X=%d;SuperObj_Y=%d;Teleport=%d;LoadedObjects=%d;side=%d;moduledistX=%d;moduledistY=%d;i=%d;depoX=%d;depoY=%d;sideX=%d;sideY=%d;depo2X=%d;depo2Y=%d;moduledist2X=%d;moduledist2Y=%d;side2X=%d;side2Y=%d;US_Front=%d;US_Left=%d;US_Right=%d;CSLeft_R=%d;CSLeft_G=%d;CSLeft_B=%d;CSRight_R=%d;CSRight_G=%d;CSRight_B=%d;PositionX=%d;PositionY=%d;TM_State=%d;Compass=%d;Time=%d;WheelLeft=%d;WheelRight=%d;LED_1=%d;MyState=%d;",Duration,SuperDuration,bGameEnd,CurAction,CurGame,SuperObj_Num,SuperObj_X,SuperObj_Y,Teleport,LoadedObjects,side,moduledistX,moduledistY,i,depoX,depoY,sideX,sideY,depo2X,depo2Y,moduledist2X,moduledist2Y,side2X,side2Y,US_Front,US_Left,US_Right,CSLeft_R,CSLeft_G,CSLeft_B,CSRight_R,CSRight_G,CSRight_B,PositionX,PositionY,TM_State,Compass,Time,WheelLeft,WheelRight,LED_1,MyState);
    return info;
}
 
DLL_EXPORT char* GetTeamName()
{
     return "BRC ";
}

DLL_EXPORT int GetCurAction()
{
    return CurAction;
}

//Only Used by CsBot Rescue Platform
DLL_EXPORT int GetTeleport()
{
    return Teleport;
}

//Only Used by CsBot Rescue Platform
DLL_EXPORT void SetSuperObj(int X, int Y, int num)
{
    SuperObj_X = X;
    SuperObj_Y = Y;
    SuperObj_Num = num;
}
//Only Used by CsBot Rescue Platform
DLL_EXPORT void GetSuperObj(int *X, int *Y, int *num)
{
    *X = SuperObj_X;
    *Y = SuperObj_Y;
    *num = SuperObj_Num;
}

#endif ////CSBOT_REAL

DLL_EXPORT void SetDataAI(volatile int* packet, volatile int *AI_IN)
{

    int sum = 0;

    US_Front = AI_IN[0]; packet[0] = US_Front; sum += US_Front;
    US_Left = AI_IN[1]; packet[1] = US_Left; sum += US_Left;
    US_Right = AI_IN[2]; packet[2] = US_Right; sum += US_Right;
    CSLeft_R = AI_IN[3]; packet[3] = CSLeft_R; sum += CSLeft_R;
    CSLeft_G = AI_IN[4]; packet[4] = CSLeft_G; sum += CSLeft_G;
    CSLeft_B = AI_IN[5]; packet[5] = CSLeft_B; sum += CSLeft_B;
    CSRight_R = AI_IN[6]; packet[6] = CSRight_R; sum += CSRight_R;
    CSRight_G = AI_IN[7]; packet[7] = CSRight_G; sum += CSRight_G;
    CSRight_B = AI_IN[8]; packet[8] = CSRight_B; sum += CSRight_B;
    PositionX = AI_IN[9]; packet[9] = PositionX; sum += PositionX;
    PositionY = AI_IN[10]; packet[10] = PositionY; sum += PositionY;
    TM_State = AI_IN[11]; packet[11] = TM_State; sum += TM_State;
    Compass = AI_IN[12]; packet[12] = Compass; sum += Compass;
    Time = AI_IN[13]; packet[13] = Time; sum += Time;
    packet[14] = sum;

}
DLL_EXPORT void GetCommand(int *AI_OUT)
{
    AI_OUT[0] = WheelLeft;
    AI_OUT[1] = WheelRight;
    AI_OUT[2] = LED_1;
    AI_OUT[3] = MyState;
}
void Game0()
{

    if(SuperDuration>0)
    {
        SuperDuration--;
    }
    else if(Duration>0)
    {
        Duration--;
    }
    else if(Time>=210 && Time<=230)
    {
        Duration = 0;
        CurAction =1;
    }
    else if(CSLeft_R>=190 && CSLeft_R<=255 && CSLeft_G>=100 && CSLeft_G<=180 && CSLeft_B>=0 && CSLeft_B<=15 && CSRight_R>=190 && CSRight_R<=255 && CSRight_G>=100 && CSRight_G<=180 && CSRight_B>=0 && CSRight_B<=15&&(LoadedObjects>0))
    {
        Duration = 49;
        CurAction =2;
    }
    else if(CSLeft_R>=190 && CSLeft_R<=255 && CSLeft_G>=100 && CSLeft_G<=180 && CSLeft_B>=0 && CSLeft_B<=15&&(LoadedObjects>0))
    {
        Duration = 0;
        CurAction =3;
    }
    else if(CSRight_R>=190 && CSRight_R<=255 && CSRight_G>=100 && CSRight_G<=180 && CSRight_B>=0 && CSRight_B<=15&&(LoadedObjects>0))
    {
        Duration = 0;
        CurAction =4;
    }
    else if(CSLeft_R>=195 && CSLeft_R<=255 && CSLeft_G>=195 && CSLeft_G<=255 && CSLeft_B>=0 && CSLeft_B<=10 && CSRight_R>=195 && CSRight_R<=255 && CSRight_G>=195 && CSRight_G<=255 && CSRight_B>=0 && CSRight_B<=10)
    {
        Duration = 0;
        CurAction =5;
    }
    else if(CSLeft_R>=190 && CSLeft_R<=255 && CSLeft_G>=190 && CSLeft_G<=255 && CSLeft_B>=0 && CSLeft_B<=15)
    {
        Duration = 1;
        CurAction =6;
    }
    else if(CSRight_R>=190 && CSRight_R<=255 && CSRight_G>=190 && CSRight_G<=255 && CSRight_B>=0 && CSRight_B<=15)
    {
        Duration = 1;
        CurAction =7;
    }
    else if(US_Front>=0 && US_Front<=10 && US_Left>=19 && US_Left<=255 && US_Right>=19 && US_Right<=255)
    {
        Duration = 0;
        CurAction =8;
    }
    else if(US_Front>=0 && US_Front<=11 && US_Left>=0 && US_Left<=12 && US_Right>=0 && US_Right<=12)
    {
        Duration = 3;
        CurAction =9;
    }
    else if(US_Right>=0 && US_Right<=8)
    {
        Duration = 0;
        CurAction =10;
    }
    else if(US_Left>=0 && US_Left<=8)
    {
        Duration = 0;
        CurAction =11;
    }
    else if(US_Front>=0 && US_Front<=10 && US_Left>=0 && US_Left<=12)
    {
        Duration = 1;
        CurAction =12;
    }
    else if(US_Front>=0 && US_Front<=10 && US_Right>=0 && US_Right<=11)
    {
        Duration = 3;
        CurAction =13;
    }
    else if(CSLeft_R>=200 && CSLeft_R<=255 && CSLeft_G>=20 && CSLeft_G<=60 && CSLeft_B>=20 && CSLeft_B<=60&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =14;
    }
    else if(CSLeft_R>=180 && CSLeft_R<=222 && CSLeft_G>=52 && CSLeft_G<=72 && CSLeft_B>=225 && CSLeft_B<=255 && CSRight_R>=180 && CSRight_R<=222 && CSRight_G>=52 && CSRight_G<=72 && CSRight_B>=225 && CSRight_B<=255 && Compass>=113 && Compass<=239&&(LoadedObjects>0))
    {
        Duration = 0;
        CurAction =15;
        if(Compass>=95 && Compass<=359)
        {
            Duration = 1;
            CurAction =16;
        }
        else if(Compass>=0 && Compass<=85)
        {
            Duration = 1;
            CurAction =17;
        }
    }
    else if(CSRight_R>=180 && CSRight_R<=222 && CSRight_G>=52 && CSRight_G<=72 && CSRight_B>=225 && CSRight_B<=255 && Compass>=70 && Compass<=120&&(LoadedObjects>0))
    {
        Duration = 0;
        CurAction =18;
        if(Compass>=0 && Compass<=175)
        {
            Duration = 1;
            CurAction =19;
        }
        else if(Compass>=185 && Compass<=359)
        {
            Duration = 1;
            CurAction =20;
        }
    }
    else if(CSLeft_R>=180 && CSLeft_R<=222 && CSLeft_G>=52 && CSLeft_G<=72 && CSLeft_B>=225 && CSLeft_B<=255 && Compass>=230 && Compass<=310&&(LoadedObjects>0))
    {
        Duration = 0;
        CurAction =21;
        if(Compass>=0 && Compass<=175)
        {
            Duration = 1;
            CurAction =22;
        }
        else if(Compass>=185 && Compass<=359)
        {
            Duration = 1;
            CurAction =23;
        }
    }
    else if(CSRight_R>=200 && CSRight_R<=255 && CSRight_G>=20 && CSRight_G<=70 && CSRight_B>=20 && CSRight_B<=70&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =24;
    }
    else if(CSLeft_R>=23 && CSLeft_R<=53 && CSLeft_G>=23 && CSLeft_G<=53 && CSLeft_B>=23 && CSLeft_B<=53&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =25;
    }
    else if(CSRight_R>=23 && CSRight_R<=53 && CSRight_G>=23 && CSRight_G<=53 && CSRight_B>=23 && CSRight_B<=53&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =26;
    }
    else if(CSLeft_R>=15 && CSLeft_R<=65 && CSLeft_G>=220 && CSLeft_G<=255 && CSLeft_B>=220 && CSLeft_B<=255&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =27;
    }
    else if(CSRight_R>=15 && CSRight_R<=65 && CSRight_G>=220 && CSRight_G<=255 && CSRight_B>=220 && CSRight_B<=255&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =28;
    }
    else if(true)
    {
        Duration = 0;
        CurAction =29;
    }
    switch(CurAction)
    {
        case 1:
            WheelLeft=0;
            WheelRight=0;
            LED_1=0;
            MyState=0;
             Teleport =2;  
            LoadedObjects = 0;
            WheelLeft = 0;  WheelRight = 0;  LED_1=0;
            Duration = 0;   SuperDuration = 0;
            break;
        case 2:
            WheelLeft=0;
            WheelRight=0;
            LED_1=2;
            MyState=0;
            if(Duration == 1) {LoadedObjects = 0; } 
            break;
        case 3:
            WheelLeft=0;
            WheelRight=2;
            LED_1=0;
            MyState=0;
            break;
        case 4:
            WheelLeft=2;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            break;
        case 5:
            WheelLeft=-4;
            WheelRight=3;
            LED_1=0;
            MyState=0;
            if (LoadedObjects==0){
WheelLeft=5;
                    
WheelRight=5;
                    

}
else if (LoadedObjects>0){
WheelLeft=-4;
                    
WheelRight=0;
                    

}
            break;
        case 6:
            WheelLeft=2;
            WheelRight=-5;
            LED_1=0;
            MyState=0;
            if (LoadedObjects==0){
WheelLeft=5;
                    
WheelRight=5;
                    

}
if (LoadedObjects>0){
WheelLeft=3;
                    
WheelRight=-5;
                    

}
            break;
        case 7:
            WheelLeft=-5;
            WheelRight=2;
            LED_1=0;
            MyState=0;
            if (LoadedObjects==0){
WheelLeft=5;
                    
WheelRight=5;
                    

}
if (LoadedObjects>0){
WheelLeft=-5;
                    
WheelRight=2;
                    

}


            break;
        case 8:
            WheelLeft=2;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            break;
        case 9:
            WheelLeft=2;
            WheelRight=-3;
            LED_1=0;
            MyState=0;
            break;
        case 10:
            WheelLeft=-1;
            WheelRight=3;
            LED_1=0;
            MyState=0;
            break;
        case 11:
            WheelLeft=3;
            WheelRight=-1;
            LED_1=0;
            MyState=0;
            break;
        case 12:
            WheelLeft=2;
            WheelRight=-1;
            LED_1=0;
            MyState=0;
            break;
        case 13:
            WheelLeft=-1;
            WheelRight=2;
            LED_1=0;
            MyState=0;
            break;
        case 14:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 15:
            WheelLeft=-5;
            WheelRight=-5;
            LED_1=0;
            MyState=0;
            break;
        case 16:
            WheelLeft=2;
            WheelRight=-2;
            LED_1=0;
            MyState=0;
            break;
        case 17:
            WheelLeft=-2;
            WheelRight=2;
            LED_1=0;
            MyState=0;
            break;
        case 18:
            WheelLeft=-5;
            WheelRight=-5;
            LED_1=0;
            MyState=0;
            break;
        case 19:
            WheelLeft=-2;
            WheelRight=2;
            LED_1=0;
            MyState=0;
            break;
        case 20:
            WheelLeft=2;
            WheelRight=-2;
            LED_1=0;
            MyState=0;
            break;
        case 21:
            WheelLeft=-5;
            WheelRight=-5;
            LED_1=0;
            MyState=0;
            break;
        case 22:
            WheelLeft=-2;
            WheelRight=2;
            LED_1=0;
            MyState=0;
            break;
        case 23:
            WheelLeft=2;
            WheelRight=-2;
            LED_1=0;
            MyState=0;
            break;
        case 24:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 25:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 26:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 27:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 28:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 29:
            WheelLeft=3;
            WheelRight=3;
            LED_1=0;
            MyState=0;
            break;
        default:
            break;
    }

}

void Game1()
{

    if(SuperDuration>0)
    {
        SuperDuration--;
    }
    else if(Duration>0)
    {
        Duration--;
    }
    else if(CSLeft_R>=190 && CSLeft_R<=255 && CSLeft_G>=190 && CSLeft_G<=255 && CSLeft_B>=0 && CSLeft_B<=15 && CSRight_R>=190 && CSRight_R<=210 && CSRight_G>=212 && CSRight_G<=223 && CSRight_B>=0 && CSRight_B<=5&&(LoadedObjects>0))
    {
        Duration = 0;
        CurAction =1;
    }
    else if(CSRight_R>=190 && CSRight_R<=255 && CSRight_G>=190 && CSRight_G<=255 && CSRight_B>=0 && CSRight_B<=15&&(LoadedObjects>0))
    {
        Duration = 0;
        CurAction =2;
    }
    else if(CSLeft_R>=190 && CSLeft_R<=255 && CSLeft_G>=190 && CSLeft_G<=255 && CSLeft_B>=0 && CSLeft_B<=15&&(LoadedObjects>0))
    {
        Duration = 0;
        CurAction =3;
    }
    else if(CSLeft_R>=199 && CSLeft_R<=209 && CSLeft_G>=125 && CSLeft_G<=135 && CSLeft_B>=0 && CSLeft_B<=5 && CSRight_R>=199 && CSRight_R<=209 && CSRight_G>=125 && CSRight_G<=135 && CSRight_B>=0 && CSRight_B<=5&&(LoadedObjects>4))
    {
        Duration = 59;
        CurAction =4;
    }
    else if(CSRight_R>=180 && CSRight_R<=230 && CSRight_G>=110 && CSRight_G<=160 && CSRight_B>=0 && CSRight_B<=5&&(LoadedObjects>3))
    {
        Duration = 0;
        CurAction =5;
    }
    else if(CSLeft_R>=180 && CSLeft_R<=230 && CSLeft_G>=110 && CSLeft_G<=160 && CSLeft_B>=0 && CSLeft_B<=5&&(LoadedObjects>3))
    {
        Duration = 0;
        CurAction =6;
    }
    else if(US_Front>=0 && US_Front<=20)
    {
        Duration = 0;
        CurAction =7;
    }
    else if(US_Right>=0 && US_Right<=20)
    {
        Duration = 0;
        CurAction =8;
    }
    else if(US_Front>=40 && US_Front<=255 && US_Left>=0 && US_Left<=10 && US_Right>=0 && US_Right<=13)
    {
        Duration = 0;
        CurAction =9;
    }
    else if(US_Left>=0 && US_Left<=20)
    {
        Duration = 0;
        CurAction =10;
    }
    else if(PositionX>=0 && PositionX<=0 && PositionY>=0 && PositionY<=0)
    {
        Duration = 0;
        CurAction =11;
    }
    else if(PositionX>=314 && PositionX<=1000 && PositionY>=0 && PositionY<=102&&(LoadedObjects>9))
    {
        Duration = 0;
        CurAction =12;
        if(Compass>=185 && Compass<=360)
        {
            Duration = 0;
            CurAction =13;
        }
        else if(Compass>=0 && Compass<=175)
        {
            Duration = 0;
            CurAction =14;
        }
        else if(Compass>=175 && Compass<=185)
        {
            Duration = 0;
            CurAction =15;
        }
    }
    else if(CSLeft_R>=220 && CSLeft_R<=255 && CSLeft_G>=15 && CSLeft_G<=50 && CSLeft_B>=15 && CSLeft_B<=50&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =16;
    }
    else if(CSRight_R>=220 && CSRight_R<=255 && CSRight_G>=15 && CSRight_G<=50 && CSRight_B>=15 && CSRight_B<=50&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =17;
    }
    else if(CSLeft_R>=20 && CSLeft_R<=60 && CSLeft_G>=20 && CSLeft_G<=60 && CSLeft_B>=20 && CSLeft_B<=60&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =18;
    }
    else if(CSRight_R>=20 && CSRight_R<=60 && CSRight_G>=20 && CSRight_G<=60 && CSRight_B>=20 && CSRight_B<=60&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =19;
    }
    else if(CSLeft_R>=24 && CSLeft_R<=34 && CSLeft_G>=244 && CSLeft_G<=254 && CSLeft_B>=244 && CSLeft_B<=255&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =20;
    }
    else if(CSRight_R>=24 && CSRight_R<=34 && CSRight_G>=244 && CSRight_G<=254 && CSRight_B>=242 && CSRight_B<=255&&(LoadedObjects<6))
    {
        Duration = 49;
        CurAction =21;
    }
    else if(PositionY>=253 && PositionY<=1000)
    {
        Duration = 0;
        CurAction =22;
    }
    else if(PositionX>=352 && PositionX<=1000)
    {
        Duration = 0;
        CurAction =23;
    }
    else if(PositionY>=0 && PositionY<=10)
    {
        Duration = 0;
        CurAction =24;
    }
    else if(PositionX>=0 && PositionX<=14)
    {
        Duration = 0;
        CurAction =25;
    }
    else if(PositionX>=0 && PositionX<=160 && PositionY>=0 && PositionY<=80&&(LoadedObjects>4))
    {
        Duration = 0;
        CurAction =26;
    }
    else if(PositionX>=160 && PositionX<=1000 && PositionY>=0 && PositionY<=211&&(LoadedObjects>4))
    {
        Duration = 0;
        CurAction =27;
    }
    else if(true)
    {
        Duration = 0;
        CurAction =28;
    }
    switch(CurAction)
    {
        case 1:
            WheelLeft=-2;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            break;
        case 2:
            WheelLeft=-1;
            WheelRight=3;
            LED_1=0;
            MyState=0;
            break;
        case 3:
            WheelLeft=3;
            WheelRight=-1;
            LED_1=0;
            MyState=0;
            break;
        case 4:
            WheelLeft=0;
            WheelRight=0;
            LED_1=2;
            MyState=0;
            if(Duration<10 && Duration>5 ){
WheelLeft=-3;
                    
WheelRight=-4;
                    
}

if(Duration<5 ){
WheelLeft=4;
                    
WheelRight=2;
                    
}
            if(Duration == 1) {LoadedObjects = 0; } 
            break;
        case 5:
            WheelLeft=2;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            break;
        case 6:
            WheelLeft=0;
            WheelRight=2;
            LED_1=0;
            MyState=0;
            break;
        case 7:
            WheelLeft=3;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            break;
        case 8:
            WheelLeft=0;
            WheelRight=3;
            LED_1=0;
            MyState=0;
            break;
        case 9:
            WheelLeft=-2;
            WheelRight=1;
            LED_1=0;
            MyState=0;
            break;
        case 10:
            WheelLeft=3;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            break;
        case 11:
            WheelLeft=2;
            WheelRight=2;
            LED_1=0;
            MyState=0;
            break;
        case 12:
            WheelLeft=0;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            break;
        case 13:
            WheelLeft=1;
            WheelRight=-1;
            LED_1=0;
            MyState=0;
            break;
        case 14:
            WheelLeft=-1;
            WheelRight=1;
            LED_1=0;
            MyState=0;
            break;
        case 15:
            WheelLeft=2;
            WheelRight=2;
            LED_1=0;
            MyState=0;
            break;
        case 16:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 17:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 18:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 19:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 20:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 21:
            WheelLeft=0;
            WheelRight=0;
            LED_1=1;
            MyState=0;
            if(Duration == 1) LoadedObjects++;
            if(Duration < 6)
            {
                WheelLeft = 2;
                WheelRight = 2;
            }
            break;
        case 22:
            WheelLeft=3;
            WheelRight=1;
            LED_1=0;
            MyState=0;
            if ((Compass<340 && Compass>260) && PositionY>253){
WheelLeft=4;
                    
WheelRight=1;
                    

}
else if ((Compass>20 && Compass<100)&& PositionY>253){
WheelLeft=1;
                    
WheelRight=4;
                    

}

else if (((Compass>340 && Compass<360) || (Compass<20 &&Compass>0)) && PositionY>253){
WheelLeft=-5;
                    
WheelRight=-2;
                    


}
            break;
        case 23:
            WheelLeft=3;
            WheelRight=1;
            LED_1=0;
            MyState=0;
            if ((Compass<250 && Compass>170)&& PositionX>352){
WheelLeft=4;
                    
WheelRight=1;
                    

}
if ((Compass>290 && (Compass<360|| Compass<10))&&PositionX>352){
WheelLeft=1;
                    
WheelRight=4;
                    

}

if (Compass>250 && Compass<290 && PositionX>352){
WheelLeft=-5;
                    
WheelRight=-2;
                    


}
            break;
        case 24:
            WheelLeft=3;
            WheelRight=1;
            LED_1=0;
            MyState=0;
            if (Compass<160 && Compass>80){
WheelLeft=4;
                    
WheelRight=1;
                    

}
if (Compass>200 && Compass<280){
WheelLeft=1;
                    
WheelRight=4;
                    

}

if (Compass>160 && Compass<200){
WheelLeft=-5;
                    
WheelRight=-2;
                    


}
            break;
        case 25:
            WheelLeft=3;
            WheelRight=1;
            LED_1=0;
            MyState=0;
            if (Compass<80 && (Compass>0|| Compass<350)){
WheelLeft=4;
                    
WheelRight=1;
                    

}
if (Compass>100 && Compass<170){
WheelLeft=1;
                    
WheelRight=4;
                    

}

if (Compass>80 && Compass<100){
WheelLeft=-5;
                    
WheelRight=-2;
                    


}
            break;
        case 26:
            WheelLeft=0;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            if((depo2Y-PositionY)<0)
{
side2Y=-1;
                    
(moduledist2Y)=((depo2Y-PositionY)*(-1));
                    
}
if((depo2X-PositionX)<0)
{
side2X=-1;
                    
(moduledist2X)=((depo2X-PositionX)*(-1));
                    
}
if((depo2X-PositionX)>0)
{
side2X = 1;
                    
(moduledist2X)=((depo2X)-(PositionX));
                    
}


if((depo2Y-PositionY)>0)
{
side2Y = 1;
                    
(moduledist2Y)=((depo2Y)-(PositionY));
                    
}
/////////////////////////////
if((side2Y==-1) && (Compass>185 || Compass< 175)){
WheelLeft=-1;
                    
WheelRight=1;
                    
}

if (side2X==-1 && (Compass<95&&Compass>85))
{
WheelLeft=2;
                    
WheelRight=2;
                    
}
//////////////////////////////

if (side2Y=1 && (Compass>10&&Compass<350))
{
WheelLeft=-1;
                    
WheelRight=1;
                    
}
if (side2Y=1 &&(Compass<10||Compass>350) && (moduledist2Y>0))
{
WheelLeft=2;
                    
WheelRight=2;
                    
}
if (moduledist2Y<5)
{
WheelLeft=0;
                    
WheelRight=0;
                    
}

//////////////////////////////
if((side2X==1) && (Compass>285 || Compass< 265)&& moduledist2Y<5){
WheelLeft=1;
                    
WheelRight=-1;
                    
}

if (side2X==1 && (Compass<290&&Compass>260) && (moduledist2Y<10)&& moduledist2X>10)
{
WheelLeft=4;
                    
WheelRight=4;
                    
}
////////////////////////////
if((side2X==-1) && (Compass>95 || Compass< 85)&& moduledist2Y<5){
WheelLeft=-1;
                    
WheelRight=1;
                    
}

if (side2X==-1 && (Compass<100&&Compass>80) && (moduledist2Y<10)&& moduledist2X>10)
{
WheelLeft=4;
                    
WheelRight=4;
                    
}
/////////////////////////////

            break;
        case 27:
            WheelLeft=0;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            if((depoY-PositionY)<0)
{
sideY=-1;
                    
(moduledistY)=((depoY-PositionY)*(-1));
                    
}
if((depoX-PositionX)<0)
{
sideX=-1;
                    
(moduledistX)=((depoX-PositionX)*(-1));
                    
}
if((depoX-PositionX)>0)
{
sideX = 1;
                    
(moduledistX)=((depoX)-(PositionX));
                    
}


if((depoY-PositionY)>0)
{
sideY = 1;
                    
(moduledistY)=((depoY)-(PositionY));
                    
}
/////////////////////////////
if((sideY==-1) && (Compass>185 || Compass< 175)){
WheelLeft=-1;
                    
WheelRight=1;
                    
}

if (sideX==-1 && (Compass<95&&Compass>85))
{
WheelLeft=2;
                    
WheelRight=2;
                    
}
//////////////////////////////

if (sideY=1 && (Compass>10&&Compass<350))
{
WheelLeft=-1;
                    
WheelRight=1;
                    
}
if (sideY=1 &&(Compass<10||Compass>350) && (moduledistY>0))
{
WheelLeft=2;
                    
WheelRight=2;
                    
}
if (moduledistY<5)
{
WheelLeft=0;
                    
WheelRight=0;
                    
}

//////////////////////////////
if((sideX==1) && (Compass>285 || Compass< 265)&& moduledistY<5){
WheelLeft=1;
                    
WheelRight=-1;
                    
}

if (sideX==1 && (Compass<290&&Compass>260) && (moduledistY<10)&& moduledistX>10)
{
WheelLeft=4;
                    
WheelRight=4;
                    
}
////////////////////////////
if((sideX==-1) && (Compass>95 || Compass< 85)&& moduledistY<5){
WheelLeft=-1;
                    
WheelRight=1;
                    
}

if (sideX==-1 && (Compass<100&&Compass>80) && (moduledistY<10)&& moduledistX>10)
{
WheelLeft=4;
                    
WheelRight=4;
                    
}
/////////////////////////////

            break;
        case 28:
            WheelLeft=3;
            WheelRight=3;
            LED_1=0;
            MyState=0;
            break;
        default:
            break;
    }

}


DLL_EXPORT void OnTimer()
{
    switch (CurGame)
    {
        case 9:
            break;
        case 10:
            WheelLeft=0;
            WheelRight=0;
            LED_1=0;
            MyState=0;
            break;
        case 0:
            Game0();
            break;
        case 1:
            Game1();
            break;
        default:
            break;
    }
}

