# Kevin-and-his-robots
#include <PinChangeInt.h>    //外部中断
#include <MsTimer2.h>        //定时中断
#define PIN1 22    //TB6612驱动模块控制信号6个
#define PIN2 24
#define PIN3 26
#define PIN4 28
#define PWMA 2
#define PWMB 3
#define ENCODER_L 23  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 25
#define ENCODER_R 27
#define DIRECTION_R 29
volatile long Velocity_L, Velocity_R = 0;   //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0;     //左右轮速度
int Motor_L,Motor_R   //电机叠加后的PWM值
int Flag_Forward,Flag_Backward,Flag_Left,Flag_Right  //遥控相关变量
void Set_Pwm(int Motor_L, int Motor_R)   //控制电机转速和转向
{
  if (Motor_L > 0){
    digitalWrite(PIN1, HIGH);  //TB6612的电平控制
    digitalWrite(PIN2, LOW);  
  }
  else{            
    digitalWrite(PIN1, LOW);
    digitalWrite(PIN2, HIGH); 
  }
  analogWrite(PWMA, abs(Motor_L));  //赋值给PWM寄存器
  if (Motor_R < 0){
    digitalWrite(PIN3, HIGH);  //TB6612的电平控制
    digitalWrite(PIN4, LOW); 
  }
  else{
    digitalWrite(PIN3, LOW);
    digitalWrite(PIN4, HIGH); 
  }
  analogWrite(PWMB, abs(Motor_R));  //赋值给PWM寄存器
}
void Restriction_PWM(void)  //限速函数
{
  int Amplitude = 250;  //===PWM满幅是255 限制在250
  if (Motor_L < -Amplitude) Motor1 = -Amplitude;
  if (Motor_L > Amplitude)  Motor1 = Amplitude;
  if (Motor_R < -Amplitude) Motor2 = -Amplitude;
  if (Motor_R > Amplitude)  Motor2 = Amplitude;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);    //开启串口，设置波特率为9600
  pinMode(PIN1, OUTPUT);        //TB6612控制引脚，控制电机1的方向，01为正转，10为反转
  pinMode(PIN2, OUTPUT);          //TB6612控制引脚，
  pinMode(PIN3, OUTPUT);          //TB6612控制引脚，控制电机2的方向，01为正转，10为反转
  pinMode(PIN4, OUTPUT);          //TB6612控制引脚，
  pinMode(PWMA, OUTPUT);         //TB6612控制引脚，电机PWM
  pinMode(PWMB, OUTPUT);         //TB6612控制引脚，电机PWM
  digitalWrite(PIN1, 0);          //TB6612控制引脚拉低
  digitalWrite(PIN2, 0);          //TB6612控制引脚拉低
  digitalWrite(PIN3, 0);          //TB6612控制引脚拉低
  digitalWrite(PIN4, 0);          //TB6612控制引脚拉低
  analogWrite(PWMA, 0);          //TB6612控制引脚拉低
  analogWrite(PWMB, 0);          //TB6612控制引脚拉低
  pinMode(ENCODER_L,INPUT);      //编码器引脚
  pinMode(DIRECTION_L,INPUT);
  pinMode(ENCODER_R,INPUT);
  pinMode(DIRECTION_R,INPUT);
  delay(1000);
  MsTimer2::set(5,contro);     //使用Timer2设置5ms定时中断
  MsTimer2::start();           //使用中断功能
  attachInterrupt(23,READ_ENCODER_L,CHANGE);     //开启外部中断，编码器1
  attachInterrupt(27,READ_ENCODER_R,CHANGE);     //开启外部中断，编码器2
}
void READ_ENCODER_L(){          //外部中断读取编码器数据，跳变沿触发
  if(digitalRead(ENCODER_L)==LOW){     //如果是下降沿触发的中断
    if(digitalRead(DIRECTION_L)==LOW){
      Velocity_L--;
    }
    else{
      Velocity_L++;
    }
  }
  else{                               //如果是上升沿触发的中断
    if(digitalRead(DIRECTION_L)==LOW){
      Velocity_L++;
    }
    else{
      Velocity_L--;
    }
  }
}
void READ_ENCODER_R(){          //外部中断读取编码器数据，跳变沿触发
  if(digitalRead(ENCODER_R)==LOW){     //如果是下降沿触发的中断
    if(digitalRead(DIRECTION_R)==LOW){
      Velocity_R--;
    }
    else{
      Velocity_R++;
    }
  }
  else{                               //如果是上升沿触发的中断
    if(digitalRead(DIRECTION_R)==LOW){
      Velocity_R++;
    }
    else{
      Velocity_R--;
    }
  }
void loop() {
  // put your main code here, to run repeatedly:
  Motor_L=Serial.read();
  Restriction_PWM;
  Set_PWM(Motor_l,Motor_R);
}
