/*摩擦轮电机PWM控制*/  
#define friction_pin 12

/*储弹管道电机控速*/
#define pwmpin 13
#define pinl1 14
#define pinl2 15



/*挡板舵机和滑轨舵机*/
#include<Servo.h>            //舵机
#define shooting_baffle_pin 10 
#define track_pin 11 
Servo baffle;
Servo track;


#include <PS2X_lib.h>        //PS2手柄
PS2X ps2x;                  // create a PS2 Controller Class Object
/*************************************************************
*               PS2手柄接线方式：
*         arduino          手柄接收器
*           5                  MOSI
*           6                  MISO
*           7                  CS
*           8                  SCLK
*************************************************************/
#define PS2_DAT        A0    //MOSI
#define PS2_CMD        A1   //MISO
#define PS2_SEL        A2   //CS
#define PS2_CLK        A3   //SCLK
/* GND to GND VCC to VCC */
/*************************************************************/
unsigned char servo,PS2_LY,PS2_LX,PS2_RY,PS2_RX,PS2_KEY;  //定义L侧Y轴，X轴以及R侧Y轴，X轴的变量
void (* resetFunc) (void) = 0;                            // Reset func
//int pos = 0;                                              //用于记录挡板舵机状态的全局变量
//int spin = 0;                                             //用于记录储弹管道电机状态的全局变量
//int friction_speed = 0;                                    //用于记录摩擦轮速率的全局变量

/*麦克纳姆轮控制*/
# define L1_IN1 3
# define L1_IN2 2//左前轮 pwm

# define R1_IN1 5
# define R1_IN2 4//右前轮 pwm

# define L2_IN1 7
# define L2_IN2 6//左后轮 pwm

# define R2_IN1 9
# define R2_IN2 8//右后轮 pwm
//int incomingByte = 0;
//void FORWARD(int sp);
//void BACKWARD(int sp);
//void CLOCKWISE(int sp);
//void COUNTERCLOCKWISE(int sp);
//void LEFTSHIFT(int sp);
//void RIGHTSHIFT(int sp);
//void LEFTUP(int sp);
//void RIGHTUP(int sp);
//void LEFTDOWN(int sp);
//void RIGHTDOWN(int sp);


void setup()   {
  Serial.begin(9600);        //开启串口，波特率9600
  char error;
  motor_initialization();      //电机驱动引脚激活
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);//PS2控制
}

/******函数功能：主循环程序体*******/
void loop(){
    ps2x.read_gamepad(false, 0); //read controller and set large motor to spin at 'vibrate' speed
    PS2_LX=ps2x.Analog(PSS_LX);  //读取L侧X轴的模拟值
    PS2_LY=ps2x.Analog(PSS_LY);  //读取L侧Y轴的模拟值
    PS2_RX=ps2x.Analog(PSS_RX);  //读取R侧X轴的模拟值
    PS2_RY=ps2x.Analog(PSS_RY);  //读取R侧Y轴的模拟值  
    
    /*摇杆相关程序*/    
/*************************************************************
*       LX与RX                       LY与RY
*左右摇杆的水平模拟量值       左右摇杆的竖直模拟量值
*     区间为0~255                  区间为0~255
*     最左端为0                    最前端为0
*     最右端为255                  最后端为255
*     中间为128                    中间为127
* 当摇杆未连接的时候，PS2手柄读取的摇杆值为 X:128 Y:128
*************************************************************/
    Serial.print("PS2_LX:");
    Serial.print(PS2_LX);
    Serial.print("   PS2_LY:");
    Serial.print(PS2_LY);
    Serial.print("   PS2_RX:");
    Serial.print(PS2_RX);
    Serial.print("   PS2_RY:");
    Serial.print(PS2_RY);
    
    /*不同按键所对应的函数，共有14个按键*/
    if(ps2x.Button(PSB_TRIANGLE))//挡板舵机开闭
      {Serial.println("  PSB_TRIANGLE"); 
       BAFFLEOPEN(baffle);
       delay(1200);
       BAFFLECLOSE(baffle);
        }
//    else if(ps2x.Button(PSB_TRIANGLE) && pos ==1)
//        {
//        Serial.println("  PSB_TRIANGLE");
//        BAFFLECLOSE(baffle);
//        pos =0;
//          }
    else if(ps2x.Button(PSB_CROSS))   
      {Serial.println("  PSB_CROSS");
        }
    else if(ps2x.Button(PSB_CIRCLE))
      {Serial.println("  PSB_CIRC");}
    else if(ps2x.Button(PSB_SQUARE))//储弹管道电机
    { 
       Serial.println("  PSB_SQUARE");
       analogWrite(pwmpin,120);
       delay(400);
       analogWrite(pwmpin,0);
    }
    /*else if(ps2x.Button(PSB_SQUARE) && spin == 0)
        {
        Serial.println("  PSB_SQUARE");
        digitalWrite(relay_pin,HIGH);
        spin =1;
          }
    else if(ps2x.Button(PSB_SQUARE) && spin == 1)   
      {
      Serial.println("  PSB_SQUARE");
      digitalWrite(relay_pin,LOW);
      spin =0;
      }*/
    else if(ps2x.Button(PSB_PAD_UP))//摩擦轮控速，二档
      {Serial.println("  PSB_PAD_UP");
//        analogWrite(friction_pin,140);
//        delay(2000);
//        analogWrite(friction_pin,0);
          analogWrite(friction_pin,54);
      /*FORWARD(160)*/}
    else if(ps2x.Button(PSB_PAD_DOWN))//摩擦轮控速，停转
      {Serial.println("  PSB_PAD_DOWN");
        analogWrite(friction_pin,0);
      /*BACKWARD(160)*/}
    else if(ps2x.Button(PSB_PAD_RIGHT))//摩擦轮控速，三档  
      {Serial.println("  PSB_PAD_RIGHT");
//        analogWrite(friction_pin,180);
//        delay(2000);
//        analogWrite(friction_pin,0);
          analogWrite(friction_pin,58);
      /*RIGHTSHIFT(160)*/}
    else if(ps2x.Button(PSB_PAD_LEFT))//摩擦轮控速，一档  
      {Serial.println("  PSB_PAD_LEFT");
//        analogWrite(friction_pin,100);
//        delay(2000);
//        analogWrite(friction_pin,0);
          analogWrite(friction_pin,50);
      /*LEFTSHIFT(160)*/}
    else if(ps2x.Button(PSB_SELECT))     
      {Serial.println("  PSB_SELECT");}
    else if(ps2x.Button(PSB_START))      
      {Serial.println("  PSB_START");}
    else if(ps2x.Button(PSB_L1))        
      {Serial.println("  PSB_L1");
       COUNTERCLOCKWISE(120);
      }
    else if(ps2x.Button(PSB_L2))         
      {Serial.println("  PSB_L2");
      }
    else if(ps2x.Button(PSB_R1))          
      {Serial.println("  PSB_R1");
       CLOCKWISE(120);
      }
    else  if(ps2x.Button(PSB_R2))    
      {Serial.println("  PSB_R2");
      }



    else if(int(PS2_LY)<(int(PS2_LX)-1) && int(PS2_LY)<(255-int(PS2_LX)) && int(PS2_LY)!=128 && (int(PS2_LX)+ int(PS2_LY)!=510))
    {
      Serial.println("  FORWARD");
      FORWARD(140); 
    }
    else if(int(PS2_LY)>(int(PS2_LX)-1) && int(PS2_LY)>(255-int(PS2_LX)) && int(PS2_LY)!=128 && (int(PS2_LX)+ int(PS2_LY)!=510))
    {
      Serial.println("  BACKWARD");
      BACKWARD(140);
    }
    else if(int(PS2_LY)>(int(PS2_LX)-1) && int(PS2_LY)<(255-int(PS2_LX)) && int(PS2_LY)!=128 && (int(PS2_LX)+ int(PS2_LY)!=510))
    {
      Serial.println("  LEFTSHIFT");
      LEFTSHIFT(160);   
    }
    else if(int(PS2_LY)<(int(PS2_LX)-1) && int(PS2_LY)>(255-int(PS2_LX)) && int(PS2_LY)!=128 && (int(PS2_LX)+ int(PS2_LY)!=510))
    {
      Serial.println("  RIGHTSHIFT");
      RIGHTSHIFT(160);  
    }
    else if(int(PS2_RX)<128 && int(PS2_RY)<127 && int(PS2_RY)!=128 && (int(PS2_LX)+ int(PS2_LY)!=510))
    {
      LEFTUP(180);
    }
    else if(int(PS2_RX)>128 && int(PS2_RY)<127 && int(PS2_RY)!=128 && (int(PS2_LX)+ int(PS2_LY)!=510))
    {
      RIGHTUP(180);
    }
    else if(int(PS2_RX)<128 && int(PS2_RY)>127 && int(PS2_RY)!=128 && (int(PS2_LX)+ int(PS2_LY)!=510))
    {
      LEFTDOWN(180);
    }
    else if(int(PS2_RX)>128 && int(PS2_RY)>127 && int(PS2_RY)!=128 && (int(PS2_LX)+ int(PS2_LY)!=510))
    {
      RIGHTDOWN(180);
    }
    else  
      {Serial.println("  KEY_RELEASE");
      ALLSTOP();}

    //int(PS2_LX) int(PS2_LY)
}

/*以下为驱动相关函数代码*/
void motor_initialization()
{
  pinMode(L1_IN1, OUTPUT);pinMode(L1_IN2, OUTPUT);
  pinMode(R1_IN1, OUTPUT);pinMode(R1_IN2, OUTPUT);
  pinMode(L2_IN1, OUTPUT);pinMode(L2_IN2, OUTPUT);
  pinMode(R2_IN1, OUTPUT);pinMode(R2_IN2, OUTPUT);
  pinMode(pinl1,OUTPUT);pinMode(pinl2,OUTPUT);pinMode(pwmpin,OUTPUT);
  baffle.attach(shooting_baffle_pin);
  baffle.write(90);
  track.attach(track_pin);
  track.write(90);
  analogWrite(pwmpin,0);
  digitalWrite(pinl1,HIGH);
  digitalWrite(pinl2,LOW);
  analogWrite(friction_pin,0);
}

/*麦轮驱动*/
void L1_forward(int sp)//左前轮前进
{
  analogWrite(L1_IN1,10);
  analogWrite(L1_IN2,10+sp);
}
void R1_forward(int sp)//右前轮前进
{
  analogWrite(R1_IN1,10+sp);
  analogWrite(R1_IN2,10);
}
void L2_forward(int sp)//左后轮前进
{
  analogWrite(L2_IN1,10+sp);
  analogWrite(L2_IN2,10);
}
void R2_forward(int sp)//右后轮前进
{
  analogWrite(R2_IN1,10+sp);
  analogWrite(R2_IN2,10);
}
void ALLSTOP()
{
  analogWrite(L1_IN1,10);
  analogWrite(L1_IN2,10);  
  analogWrite(R1_IN1,10);
  analogWrite(R1_IN2,10);
  analogWrite(L2_IN1,10);
  analogWrite(L2_IN2,10);
  analogWrite(R2_IN1,10);
  analogWrite(R2_IN2,10);
}
void L1_backward(int sp)//左前轮后退
{
  analogWrite(L1_IN1,10+sp);
  analogWrite(L1_IN2,10);
}
void R1_backward(int sp)//右前轮后退
{
  analogWrite(R1_IN1,10);
  analogWrite(R1_IN2,10+sp);
}
void L2_backward(int sp)//左后轮后退
{
  analogWrite(L2_IN1,10);
  analogWrite(L2_IN2,10+sp);
}
void R2_backward(int sp)//右后轮后退
{
  analogWrite(R2_IN1,10);
  analogWrite(R2_IN2,10+sp);
}

void FORWARD(int sp)//100
{/*前进*/
  L1_forward(sp);
  R1_forward(sp);
  L2_backward(sp);
  R2_backward(sp);
//  delay(1500);
//  allstop();
//  delay(1500);
}

void BACKWARD(int sp)//100
{ /*后退*/
  L1_backward(sp);
  R1_backward(sp);
  L2_forward(sp);
  R2_forward(sp);
//  delay(1500);
//  allstop();
//  delay(1500);
}

void CLOCKWISE(int sp)//200
{  /*顺时针原地旋转*/
  L1_forward(sp);
  R1_backward(sp);
  L2_forward(sp);
  R2_backward(sp);
//  delay(1500);
//  allstop();
//  delay(1500);
}

void COUNTERCLOCKWISE(int sp)//200
{  /*逆时针原地旋转*/
  L1_backward(sp);
  R1_forward(sp);
  L2_backward(sp);
  R2_forward(sp);
//  delay(1500);
//  allstop();
//  delay(1500);
}

void LEFTSHIFT(int sp)//150
{    /*左边平移*/
  L1_backward(sp);
  R1_forward(sp);
  L2_forward(sp);
  R2_backward(sp);
//  delay(1500);
//  allstop();
//  delay(1500);
}

void RIGHTSHIFT(int sp)//150
{  /*右边平移*/
  L1_forward(sp);
  R1_backward(sp);
  L2_backward(sp);
  R2_forward(sp);
//  delay(1500);
//  allstop();
//  delay(1500);
}

void LEFTUP(int sp)//150
{  /*斜向左上方*/

  R1_forward(sp);
  R2_backward(sp);
//  delay(1500);
//  allstop();
//  delay(1500);
  }

void RIGHTUP(int sp)//150
{ /*斜向右上方*/
  L1_forward(sp);
  L2_backward(sp);
//  delay(1500);
//  allstop();
//  delay(1500);
  }

void LEFTDOWN(int sp)//150
{  /*斜向左下方*/
  L1_backward(sp);
  L2_forward(sp);
//  delay(1500);
//  allstop();
//  delay(1500);
  }

void RIGHTDOWN(int sp)//150
{  /*斜向右下方*/
  R1_backward(sp);
  R2_forward(sp);
//  delay(1500);
//  allstop();
//  delay(1500);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
  }


/*以上为麦轮驱动相关函数代码*/

/*挡板舵机的开闭函数*/
void BAFFLECLOSE(Servo current_ser)
{
//    current_ser.write();
      int i = current_ser.read();
      current_ser.write(i-85);
}

void BAFFLEOPEN(Servo current_ser)
{
//    current_ser.write();
      int i = current_ser.read();
      current_ser.write(i+85);
}
