#include "stm32f1xx_hal.h"                  // Device header
#include "PID.h"

#define Speed1 30
#define Speed2 25
#define Speed3 20
#define Speed4 15
#define Speed5 10
#define Speed6 5
#define BASE_SPEED 30      // 基础直行速度
#define KP_TRACKING 1.0f   // 循迹比例系数
int line_error = 0;        // 黑线偏差值 
int last_line_error = 0;  
uint8_t last_valid_status = 3; // 记录上一个“在赛道上”的有效状态

uint8_t trace1,trace2,trace3,trace4,trace5;

static uint8_t turn_lock = 0;    // 转弯锁：1正在直角转弯，0不在直角转弯
static uint16_t turn_timer = 0;
uint16_t Left_Sum = 0;
uint16_t Right_Sum = 0;
uint8_t Left_Turn_Flag = 0;
uint8_t Right_Turn_Flag = 0;
static uint8_t trace_status = 0;
uint8_t ts;

typedef enum {
    NORMAL_TRACKING,  // 0: 循迹
    GO_STRAIGHT,       // 1: 先直行
    TURN_ACTION        // 2: 到达位置
} Turn_State_t;

static Turn_State_t turn_state = NORMAL_TRACKING; // 当前状态
static uint16_t straight_timer = 0; // 直行阶段的计时器
static uint8_t cached_turn_dir = 0;  // 转弯方向：1=左，2=右

void Normal_Tracing(void)
{
	line_error = 0;//计算偏差
	if(trace1 == 0) line_error -= 4; // 最左
    if(trace2 == 0) line_error -= 2; // 左中
    if(trace3 == 0) line_error += 0; // 中间
    if(trace4 == 0) line_error += 2; // 右中
    if(trace5 == 0) line_error += 4; // 最右
//	ts = trace_status;
	
	if(trace1 == 1 && trace2 == 1 && trace3 == 1 && trace4 == 1 && trace5 == 1)
    {
        //断路
        line_error = last_line_error; 
    }
	else
	{
		last_line_error = line_error;//更新上次状态
	}
	
	int speed_left = BASE_SPEED + (int)(KP_TRACKING * line_error);//左轮 = 基础加修正
    int speed_right = BASE_SPEED - (int)(KP_TRACKING * line_error);//右轮 = 基础减修正
	
	 if(speed_left > 40) speed_left = 40;
    if(speed_left < -40) speed_left = -40;
    
    if(speed_right > 40) speed_right = 40;
    if(speed_right < -40) speed_right = -40;
	
	Motor_Target_Set(speed_left, speed_right);
}

void TCRT_Init(void)
{	
	trace1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	trace2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	trace3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	trace4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
	trace5 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
	
	switch(turn_state)
	{
		case(NORMAL_TRACKING)://普通寻迹
		{
			uint8_t need_turn = 0;//是否需要转弯
			//左转
			if((trace1 == 0 && trace2 == 0) || (trace1 == 0 && trace2 == 0 && trace3 == 0))
            {
                cached_turn_dir = 1;
                need_turn = 1;
            }
            // 右转
            else if((trace4 == 0 && trace5 == 0) || (trace3 == 0 && trace4 == 0 && trace5 == 0))
            {
                cached_turn_dir = 2;
                need_turn = 1;
            }
			
			 if(need_turn)
            {
                // 先直行
                turn_state = GO_STRAIGHT;
                straight_timer = 20 ; // 直行时间
            }
            else//没有检测到转弯
            {
                // 循迹
                Normal_Tracing();
            }
            break;

		}
		
		 case GO_STRAIGHT://直行
        {	
            Motor_Target_Set(Speed1, Speed1);
            
            straight_timer--;
            if(straight_timer == 0)
            {
                // 开始转弯
                turn_state = TURN_ACTION;
                // 设置转弯时间
                turn_timer = (cached_turn_dir == 1) ? 40 : 40; 
            }
            break;
        }
		
		 case TURN_ACTION: // 转弯
        {
           
            if(cached_turn_dir == 1)
            {
                Motor_Target_Set(-Speed3, Speed1); // 左转
            }
            else
            {
                Motor_Target_Set(Speed1, -Speed3); // 右转
            }

            turn_timer--;
            if(turn_timer == 0)
            {
                // 转弯结束
                turn_state = NORMAL_TRACKING;
                last_line_error = 0; // 清空误差
            }
            break;
        }
	}
}



//if(turn_lock == 0)
//    {
//        // 直角左转
//        if((trace1 == 0 && trace2 == 0) || 
//           (trace1 == 0 && trace2 == 0 && trace3 == 0) || 
//           (trace1 == 0 && trace2 == 0 && trace3 == 0 && trace4 == 0))
//        { 
//            turn_lock = 1;       
//            turn_timer = 3 0;     // 持续时间
//            trace_status = 1;
//        }
//        // 检测直角右转 
//        else if((trace4 == 0 && trace5 == 0) || 
//                (trace3 == 0 && trace4 == 0 && trace5 == 0) || 
//                (trace3 == 0 && trace4 == 0 && trace5 == 0 && trace2 == 0))
//        {
//            turn_lock = 1;       
//            turn_timer = 40;     // 持续时间
//            trace_status = 2;
//        }
//    }
//	//进入直角转弯
//	 if(turn_lock == 1)
//    {
//        if(trace_status == 1)
//        {
//            Motor_Target_Set(-Speed3, Speed1); // 左转
//        }
//        else if(trace_status == 2)
//        {
//            Motor_Target_Set(Speed1, -Speed3); // 右转
//        }
//        
//        turn_timer--;
//        if(turn_timer == 0)
//        {
//            turn_lock = 0; // 时间到
//            last_line_error = 0; //转弯结束直行
//        }
//        return; 
//    }


//	if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0) || (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0) || (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8 ) == 0))
//	{ 
//		trace_status = 1;	//一二三号识别到黑线，直角左转
//	}
//	else if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0) || (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0) || (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0))
//	{
//		trace_status = 2;	//三四五号识别到黑线，直角右转
//	}
//	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)
//	{
//		trace_status = 8;	//左侧一三号识别到黑线，可能出现环岛
//	}
//	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0)
//	{
//		trace_status = 9;	//右侧侧三五号识别到黑线，可能出现环岛
//	}
//	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0)
//	{
//		trace_status = 3;	//三号识别到黑线，直行
//	}
//	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0)
//	{
//		trace_status = 4;	//二号识别到黑线，左转
//	}
//	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)
//	{
//		trace_status = 5;		//四号识别到黑线，右转
//	}
//	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)
//	{
//		trace_status = 6; //一号识别到黑线，大角度左转
//	}
//	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0)
//	{
//		trace_status = 7;		//五号识别到黑线，大角度右转
//	}
//	else
//	{
//		trace_status = trace_status;
//	}
//	switch(trace_status)
//	{
//		case 1: 
//			Motor_Target_Set(-Speed3, Speed1);
//		break;
//		
//		case 2:
//			Motor_Target_Set(Speed1, -Speed3); 
//		break;
//		
//		case 3:
//			
////			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 1)//左侧检测到黑线，右侧没有检测到黑线，左转
////			{
////				Motor_Target_Set(Speed3, Speed1);
////			}
////			else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0)//右侧检测到黑线，左侧没有检测到黑线，右转
////			{
////				Motor_Target_Set(Speed1, Speed3);
////			}
////			else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0)
////			{
////				Motor_Target_Set(Speed1, Speed1);
////			}
//			Motor_Target_Set(Speed1, Speed1);
//		break;
//		
//		case 4:
//			Motor_Target_Set(Speed3, Speed1);
//		break;
//		
//		case 5:
//			Motor_Target_Set(Speed1, Speed3);
//		break;
//		
//		case 6:
//			Motor_Target_Set(Speed6, Speed1);
//		break;
//		
//		case 7:
//			Motor_Target_Set(Speed1, Speed6);
//		break;
//		
////		case 8:
////			Left_Sum++;
////			if(Left_Sum == 2)
////			{
////				//左转进环岛
////				Left_Turn_Flag = 1;
////				Left_Sum = 0;
////			}
////		break;
////		
////		case 9:
////			Right_Sum++;
////		if(Right_Sum == 2)
////			{
////				//右转转进环岛
////				Right_Turn_Flag = 1;
////				Right_Sum = 0;
////			}
////		break;
//	}






