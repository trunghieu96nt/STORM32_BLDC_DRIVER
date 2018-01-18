/**
  ******************************************************************************
  * @file    main.c 
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    13-January-2018
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "include.h"

/** @addtogroup GIMBAL_BLDC_NCKH
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SVPWM_TABLE_SIZE    720
#define SPEED_CONVERT       23148.148148148150f//100000/SVPWM_TABLE_SIZE/6*1000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern volatile uint32_t tick_count;
extern volatile uint32_t u32_tick_cnt_0, u32_tick_cnt_1;

static const uint16_t u16_svpwm_table_A[SVPWM_TABLE_SIZE] = 
{121,117,113,109,105,102,98,95,91,88,84,81,78,75,72,68,65,63,60,57,54,52,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,8,7,6,5,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,5,6,7,8,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,39,42,44,46,49,52,54,57,60,63,65,68,72,75,78,81,84,88,91,95,98,102,105,109,113,117,121,132,144,156,168,180,192,204,217,229,241,253,266,278,291,303,316,328,341,354,367,379,392,405,418,431,444,457,470,483,496,509,523,536,549,562,576,589,602,616,629,642,656,669,683,696,710,723,737,750,764,777,791,804,818,832,845,859,872,886,899,913,927,940,954,967,981,995,1008,1022,1035,1049,1062,1076,1089,1103,1116,1130,1143,1157,1170,1183,1197,1210,1223,1237,1250,1263,1276,1290,1303,1316,1329,1342,1355,1368,1381,1394,1407,1420,1432,1445,1458,1471,1483,1496,1508,1521,1533,1546,1558,1570,1582,1595,1607,1619,1631,1643,1655,1667,1678,1682,1686,1690,1694,1697,1701,1704,1708,1711,1715,1718,1721,1724,1727,1731,1734,1736,1739,1742,1745,1747,1750,1753,1755,1757,1760,1762,1764,1766,1768,1770,1772,1774,1776,1778,1779,1781,1782,1784,1785,1787,1788,1789,1790,1791,1792,1793,1794,1795,1796,1796,1797,1797,1798,1798,1798,1799,1799,1799,1799,1799,1799,1799,1798,1798,1798,1797,1797,1796,1796,1795,1794,1793,1792,1791,1790,1789,1788,1787,1785,1784,1782,1781,1779,1778,1776,1774,1772,1770,1768,1766,1764,1762,1760,1757,1755,1753,1750,1747,1745,1742,1739,1736,1734,1731,1727,1724,1721,1718,1715,1711,1708,1704,1701,1697,1694,1690,1686,1682,1678,1682,1686,1690,1694,1697,1701,1704,1708,1711,1715,1718,1721,1724,1727,1731,1734,1736,1739,1742,1745,1747,1750,1753,1755,1757,1760,1762,1764,1766,1768,1770,1772,1774,1776,1778,1779,1781,1782,1784,1785,1787,1788,1789,1790,1791,1792,1793,1794,1795,1796,1796,1797,1797,1798,1798,1798,1799,1799,1799,1799,1799,1799,1799,1798,1798,1798,1797,1797,1796,1796,1795,1794,1793,1792,1791,1790,1789,1788,1787,1785,1784,1782,1781,1779,1778,1776,1774,1772,1770,1768,1766,1764,1762,1760,1757,1755,1753,1750,1747,1745,1742,1739,1736,1734,1731,1727,1724,1721,1718,1715,1711,1708,1704,1701,1697,1694,1690,1686,1682,1678,1667,1655,1643,1631,1619,1607,1595,1582,1570,1558,1546,1533,1521,1508,1496,1483,1471,1458,1445,1432,1420,1407,1394,1381,1368,1355,1342,1329,1316,1303,1290,1276,1263,1250,1237,1223,1210,1197,1183,1170,1157,1143,1130,1116,1103,1089,1076,1062,1049,1035,1022,1008,995,981,967,954,940,927,913,899,886,872,859,845,832,818,804,791,777,764,750,737,723,710,696,683,669,656,642,629,616,602,589,576,562,549,536,523,509,496,483,470,457,444,431,418,405,392,379,367,354,341,328,316,303,291,278,266,253,241,229,217,204,192,180,168,156,144,132,121,117,113,109,105,102,98,95,91,88,84,81,78,75,72,68,65,63,60,57,54,52,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,8,7,6,5,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,5,6,7,8,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,39,42,44,46,49,52,54,57,60,63,65,68,72,75,78,81,84,88,91,95,98,102,105,109,113,117};
static const uint16_t u16_svpwm_table_B[SVPWM_TABLE_SIZE] = 
{1667,1678,1682,1686,1690,1694,1697,1701,1704,1708,1711,1715,1718,1721,1724,1727,1731,1734,1736,1739,1742,1745,1747,1750,1753,1755,1757,1760,1762,1764,1766,1768,1770,1772,1774,1776,1778,1779,1781,1782,1784,1785,1787,1788,1789,1790,1791,1792,1793,1794,1795,1796,1796,1797,1797,1798,1798,1798,1799,1799,1799,1799,1799,1799,1799,1798,1798,1798,1797,1797,1796,1796,1795,1794,1793,1792,1791,1790,1789,1788,1787,1785,1784,1782,1781,1779,1778,1776,1774,1772,1770,1768,1766,1764,1762,1760,1757,1755,1753,1750,1747,1745,1742,1739,1736,1734,1731,1727,1724,1721,1718,1715,1711,1708,1704,1701,1697,1694,1690,1686,1682,1678,1682,1686,1690,1694,1697,1701,1704,1708,1711,1715,1718,1721,1724,1727,1731,1734,1736,1739,1742,1745,1747,1750,1753,1755,1757,1760,1762,1764,1766,1768,1770,1772,1774,1776,1778,1779,1781,1782,1784,1785,1787,1788,1789,1790,1791,1792,1793,1794,1795,1796,1796,1797,1797,1798,1798,1798,1799,1799,1799,1799,1799,1799,1799,1798,1798,1798,1797,1797,1796,1796,1795,1794,1793,1792,1791,1790,1789,1788,1787,1785,1784,1782,1781,1779,1778,1776,1774,1772,1770,1768,1766,1764,1762,1760,1757,1755,1753,1750,1747,1745,1742,1739,1736,1734,1731,1727,1724,1721,1718,1715,1711,1708,1704,1701,1697,1694,1690,1686,1682,1678,1667,1655,1643,1631,1619,1607,1595,1582,1570,1558,1546,1533,1521,1508,1496,1483,1471,1458,1445,1432,1420,1407,1394,1381,1368,1355,1342,1329,1316,1303,1290,1276,1263,1250,1237,1223,1210,1197,1183,1170,1157,1143,1130,1116,1103,1089,1076,1062,1049,1035,1022,1008,995,981,967,954,940,927,913,899,886,872,859,845,832,818,804,791,777,764,750,737,723,710,696,683,669,656,642,629,616,602,589,576,562,549,536,523,509,496,483,470,457,444,431,418,405,392,379,367,354,341,328,316,303,291,278,266,253,241,229,217,204,192,180,168,156,144,132,121,117,113,109,105,102,98,95,91,88,84,81,78,75,72,68,65,63,60,57,54,52,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,8,7,6,5,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,5,6,7,8,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,39,42,44,46,49,52,54,57,60,63,65,68,72,75,78,81,84,88,91,95,98,102,105,109,113,117,121,117,113,109,105,102,98,95,91,88,84,81,78,75,72,68,65,63,60,57,54,52,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,8,7,6,5,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,5,6,7,8,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,39,42,44,46,49,52,54,57,60,63,65,68,72,75,78,81,84,88,91,95,98,102,105,109,113,117,121,132,144,156,168,180,192,204,217,229,241,253,266,278,291,303,316,328,341,354,367,379,392,405,418,431,444,457,470,483,496,509,523,536,549,562,576,589,602,616,629,642,656,669,683,696,710,723,737,750,764,777,791,804,818,832,845,859,872,886,899,913,927,940,954,967,981,995,1008,1022,1035,1049,1062,1076,1089,1103,1116,1130,1143,1157,1170,1183,1197,1210,1223,1237,1250,1263,1276,1290,1303,1316,1329,1342,1355,1368,1381,1394,1407,1420,1432,1445,1458,1471,1483,1496,1508,1521,1533,1546,1558,1570,1582,1595,1607,1619,1631,1643,1655};
static const uint16_t u16_svpwm_table_C[SVPWM_TABLE_SIZE] = 
{1682,1678,1667,1655,1643,1631,1619,1607,1595,1582,1570,1558,1546,1533,1521,1508,1496,1483,1471,1458,1445,1432,1420,1407,1394,1381,1368,1355,1342,1329,1316,1303,1290,1276,1263,1250,1237,1223,1210,1197,1183,1170,1157,1143,1130,1116,1103,1089,1076,1062,1049,1035,1022,1008,995,981,967,954,940,927,913,899,886,872,859,845,832,818,804,791,777,764,750,737,723,710,696,683,669,656,642,629,616,602,589,576,562,549,536,523,509,496,483,470,457,444,431,418,405,392,379,367,354,341,328,316,303,291,278,266,253,241,229,217,204,192,180,168,156,144,132,121,117,113,109,105,102,98,95,91,88,84,81,78,75,72,68,65,63,60,57,54,52,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,8,7,6,5,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,5,6,7,8,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,39,42,44,46,49,52,54,57,60,63,65,68,72,75,78,81,84,88,91,95,98,102,105,109,113,117,121,117,113,109,105,102,98,95,91,88,84,81,78,75,72,68,65,63,60,57,54,52,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,8,7,6,5,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,5,6,7,8,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,39,42,44,46,49,52,54,57,60,63,65,68,72,75,78,81,84,88,91,95,98,102,105,109,113,117,121,132,144,156,168,180,192,204,217,229,241,253,266,278,291,303,316,328,341,354,367,379,392,405,418,431,444,457,470,483,496,509,523,536,549,562,576,589,602,616,629,642,656,669,683,696,710,723,737,750,764,777,791,804,818,832,845,859,872,886,899,913,927,940,954,967,981,995,1008,1022,1035,1049,1062,1076,1089,1103,1116,1130,1143,1157,1170,1183,1197,1210,1223,1237,1250,1263,1276,1290,1303,1316,1329,1342,1355,1368,1381,1394,1407,1420,1432,1445,1458,1471,1483,1496,1508,1521,1533,1546,1558,1570,1582,1595,1607,1619,1631,1643,1655,1667,1678,1682,1686,1690,1694,1697,1701,1704,1708,1711,1715,1718,1721,1724,1727,1731,1734,1736,1739,1742,1745,1747,1750,1753,1755,1757,1760,1762,1764,1766,1768,1770,1772,1774,1776,1778,1779,1781,1782,1784,1785,1787,1788,1789,1790,1791,1792,1793,1794,1795,1796,1796,1797,1797,1798,1798,1798,1799,1799,1799,1799,1799,1799,1799,1798,1798,1798,1797,1797,1796,1796,1795,1794,1793,1792,1791,1790,1789,1788,1787,1785,1784,1782,1781,1779,1778,1776,1774,1772,1770,1768,1766,1764,1762,1760,1757,1755,1753,1750,1747,1745,1742,1739,1736,1734,1731,1727,1724,1721,1718,1715,1711,1708,1704,1701,1697,1694,1690,1686,1682,1678,1682,1686,1690,1694,1697,1701,1704,1708,1711,1715,1718,1721,1724,1727,1731,1734,1736,1739,1742,1745,1747,1750,1753,1755,1757,1760,1762,1764,1766,1768,1770,1772,1774,1776,1778,1779,1781,1782,1784,1785,1787,1788,1789,1790,1791,1792,1793,1794,1795,1796,1796,1797,1797,1798,1798,1798,1799,1799,1799,1799,1799,1799,1799,1798,1798,1798,1797,1797,1796,1796,1795,1794,1793,1792,1791,1790,1789,1788,1787,1785,1784,1782,1781,1779,1778,1776,1774,1772,1770,1768,1766,1764,1762,1760,1757,1755,1753,1750,1747,1745,1742,1739,1736,1734,1731,1727,1724,1721,1718,1715,1711,1708,1704,1701,1697,1694,1690,1686};
static int32_t s32_svpwm_inc_0, s32_svpwm_idx_0;
static int32_t s32_svpwm_inc_1, s32_svpwm_idx_1;
static uint32_t u32_ts_motor_0, u32_ts_motor_0_buff;
static uint32_t u32_ts_motor_1, u32_ts_motor_1_buff;

/* Private function prototypes -----------------------------------------------*/
static void v_Board_Init(void);
static void v_BLDC_Ctl_Init(void);
static void v_Set_BLDC_Speed(ENUM_MOTOR_T enum_motor, int32_t s32_speed);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  v_Board_Init();
  
  /* Infinite loop */
  while (1)
  {
    v_BLDC_Receive();

    if (u32_tick_cnt_0 >= u32_ts_motor_0) // unit: 10us
    {
      u32_tick_cnt_0 = 0;
      if (u32_ts_motor_0 != u32_ts_motor_0_buff)
        u32_ts_motor_0 = u32_ts_motor_0_buff;
      if (s32_svpwm_inc_0 != 0) // if need shift voltage
      {
        /* Increase index 0 with s32_svpwm_inc_0 */
        s32_svpwm_idx_0 += s32_svpwm_inc_0;
        if (s32_svpwm_idx_0 >= SVPWM_TABLE_SIZE) s32_svpwm_idx_0 = 0;
        else if (s32_svpwm_idx_0 < 0) s32_svpwm_idx_0 = SVPWM_TABLE_SIZE - 1;
        
        /* Write desired pwm */
        v_PWM_Set(MOTOR_0, u16_svpwm_table_A[s32_svpwm_idx_0], 
                  u16_svpwm_table_B[s32_svpwm_idx_0], u16_svpwm_table_C[s32_svpwm_idx_0]);
      }
    }
    
    if (u32_tick_cnt_1 >= u32_ts_motor_1) // unit: 10us
    {
      u32_tick_cnt_1 = 0;
      if (u32_ts_motor_1 != u32_ts_motor_1_buff)
        u32_ts_motor_1 = u32_ts_motor_1_buff;
      if (s32_svpwm_inc_1 != 0) // if need shift voltage
      {
        /* Increase index 1 with s32_svpwm_inc_1 */
        s32_svpwm_idx_1 += s32_svpwm_inc_1;
        if (s32_svpwm_idx_1 >= SVPWM_TABLE_SIZE) s32_svpwm_idx_1 = 0;
        else if (s32_svpwm_idx_1 < 0) s32_svpwm_idx_1 = SVPWM_TABLE_SIZE - 1;
        
        /* Write desired pwm */
        v_PWM_Set(MOTOR_1, u16_svpwm_table_A[s32_svpwm_idx_1], 
                  u16_svpwm_table_B[s32_svpwm_idx_1], u16_svpwm_table_C[s32_svpwm_idx_1]);
      }
    }
    
  }
}

/**
  * @brief  Init board
  * @param  None
  * @retval None
  */
static void v_Board_Init(void)
{
  /* Check System Clock*/
  RCC_ClocksTypeDef RCC_ClocksStructure;
  uint8_t u8_clock_source;
  u8_clock_source = RCC_GetSYSCLKSource();
  if (u8_clock_source != 0x08) // 0x08: PLL used as system clock
  {
    //while (true);
  }
  RCC_GetClocksFreq(&RCC_ClocksStructure);
  if (RCC_ClocksStructure.SYSCLK_Frequency != 72000000)
  {
    //while (true);
  }
  
  /* Enable SysTick at 1ms interrupt */
  SysTick_Config(SystemCoreClock / F_CTRL);
  
  /* Driver Initialization */
  v_GPIO_Init();
  v_Green_On();
  
  v_Motor_Init();
  //v_PWM_Set(MOTOR_0, 0, 900, 1799);
  
  v_UART_Comm_Init();
  
  v_BLDC_Ctl_Init();
}

/**
  * @brief  BLDC Init
  * @param  None
  * @retval None
  */
static void v_BLDC_Ctl_Init(void)
{
  /* MOTOR 0 */
  s32_svpwm_inc_0 = 0;
  s32_svpwm_idx_0 = 0;
  v_PWM_Set(MOTOR_0, u16_svpwm_table_A[s32_svpwm_idx_0], 
            u16_svpwm_table_B[s32_svpwm_idx_0], u16_svpwm_table_C[s32_svpwm_idx_0]);
  v_Set_BLDC_Speed(MOTOR_0, 0);
  
  /* MOTOR 1 */
  s32_svpwm_inc_1 = 0;
  s32_svpwm_idx_1 = 0;
  v_PWM_Set(MOTOR_1, u16_svpwm_table_A[s32_svpwm_idx_1], 
            u16_svpwm_table_B[s32_svpwm_idx_1], u16_svpwm_table_C[s32_svpwm_idx_1]);
  v_Set_BLDC_Speed(MOTOR_1, 0);
}

/**
  * @brief  Set BLDC Speed (Hz) (x1000)
  * @param  ...
  * @param  ...
  * @retval None
  */
static void v_Set_BLDC_Speed(ENUM_MOTOR_T enum_motor, int32_t s32_speed)
{
  float flt_ts_motor;
  
  switch (enum_motor)
  {
    case MOTOR_0: 
      if (s32_speed == 0)
      {
        s32_svpwm_inc_0  = 0;
      }
      else
      {
        if (s32_speed > 0) s32_svpwm_inc_0 = 1;
        else
        {
          s32_speed = -s32_speed;
          s32_svpwm_inc_0 = -1;
        }
        
        /* Calculate ts in 10us */
        flt_ts_motor = SPEED_CONVERT / (float)s32_speed;
        u32_ts_motor_0_buff = (uint32_t)flt_ts_motor;
      }
      break;
    case MOTOR_1:
      if (s32_speed == 0)
      {
        s32_svpwm_inc_1  = 0;
      }
      else
      {
        if (s32_speed > 0) s32_svpwm_inc_1 = 1;
        else
        {
          s32_speed = -s32_speed;
          s32_svpwm_inc_1 = -1;
        }
        
        /* Calculate ts in 10us */
        flt_ts_motor = SPEED_CONVERT / (float)s32_speed;
        u32_ts_motor_1_buff = (uint32_t)flt_ts_motor;
      }
      break;
    default:
      break;
  }
}

bool bool_Set_BLDC_Speed_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  int16_t s16_desired_speed;
  if (*pu8_payload == 0x03)
  {
    s16_desired_speed = (*(pu8_payload + 1) << 8) & 0x0ff00;
    s16_desired_speed += *(pu8_payload + 2) & 0x0ff;
    v_Set_BLDC_Speed(MOTOR_0, (int32_t)s16_desired_speed);
    
    s16_desired_speed = (*(pu8_payload + 3) << 8) & 0x0ff00;
    s16_desired_speed += *(pu8_payload + 4) & 0x0ff;
    v_Set_BLDC_Speed(MOTOR_1, (int32_t)s16_desired_speed);
  }
//  au8_respond_payload[0] = *pu8_payload;
//  au8_respond_payload[1] = 0x00; //Ok
//  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  v_Red_Toggle();
  return true;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
    
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
