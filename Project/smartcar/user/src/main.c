#include "main.h"
#include "steer.h"
#include "motor.h"

/**
 * @brief  Initailize all the prepherials
 * @param  void
 * @retval void
 * @note   This function should be called at the beginning of main function
 */
void init_all()
{
    board_init();
    PIT_Init();
    lcd_init();
    ADC_Init();
    PWM_Init();

    gpio_init(B13, GPO, 1, GPIO_PIN_CONFIG); // ���ʹ�ܶ�?
    gpio_interrupt_init(D26, FALLING, GPIO_INT_CONFIG);
    NVIC_SetPriority(GPIO3_Combined_16_31_IRQn, 3); // ����
    gpio_init(C27, GPO, 0, GPIO_PIN_CONFIG);        // ������

    // ��ʼ�� QTIMER_1 A��ʹ��QTIMER1_TIMER0_C0 B��ʹ��QTIMER1_TIMER1_C1
    qtimer_quad_init(QTIMER_1, QTIMER1_TIMER0_C0, QTIMER1_TIMER1_C1);
    // ��ʼ�� QTIMER_1 A��ʹ��QTIMER1_TIMER2_C2 B��ʹ��QTIMER1_TIMER3_C24
    qtimer_quad_init(QTIMER_1, QTIMER1_TIMER2_C2, QTIMER1_TIMER3_C24);

    mt9v03x_csi_init();
    uart_init(USART_3, 9600, UART3_TX_C8, UART3_RX_C9); // ����
    gpio_init(C9, GPI, 1, GPIO_PIN_CONFIG);
};

// ******  读取 ADC 采样  ******
void read_adc()
{
    if ((start >= 200) && (start < 3500))
    {
        start += 1;
    }

    if (start >= 3500)
    {
        tingcheban = gpio_get(C9);
        if (tingcheban == 0)
            tingche = 1;
    }

    float adc[7] = {0};

    for (int i = 0; i < 3; i++)
    {
        adc[0] += 1.0 * adc_convert(ADC_1, adc0) / 3;
        adc[1] += 1.0 * adc_convert(ADC_1, adc1) / 3;
        adc[2] += 1.0 * adc_convert(ADC_1, adc2) / 3;
        adc[3] += 1.0 * adc_convert(ADC_1, adc3) / 3;
        adc[4] += 1.0 * adc_convert(ADC_1, adc4) / 3;
        // adc[5]+=1.0*adc_convert(ADC_1,adc5)/3;
        // adc[6]+=1.0*adc_convert(ADC_1,adc6)/3;
    }

    for (int i = 0; i < 7; i++)
    {
        ADC_Memory[i][2] = ADC_Memory[i][1];
        ADC_Memory[i][1] = ADC_Memory[i][0];
        ADC_Memory[i][0] = 0.6 * adc[i] + 0.3 * ADC_Memory[i][1] + 0.1 * ADC_Memory[i][2];
    }

    L = ADC_Memory[0][0];
    LV = ADC_Memory[1][0];
    M = ADC_Memory[2][0];
    RV = ADC_Memory[3][0];
    R = ADC_Memory[4][0];
    // L2=ADC_Memory[5][0];
    // R2=ADC_Memory[6][0];

    L2 = 0;
    R2 = 0;
    R = (R + R2) / 2;
    L = (L + L2) / 2;
}

/**
 * @brief  judge the road element
 * @param  void
 * @retval void
 */
void road_judge()
{
    if (rjudge == 0)
    {
        start += 1;
        if (start >= 200)
            rjudge = 1;
    }
    else
    {
        rjudge = 1; // ��ԭ
        if (tingche >= 1)
            rjudge = 9;

        if (((M > 150) || (huandao > 0)) && (huandao < 140))
        {
            huandao += 1;
            rjudge = 4;
            if (huandao > 43)
            {
                rjudge = 5;
            }
        }
        else
        {
            if (((M > 140) || (huandao > 140)) && (huandao < 230))
            {
                huandao += 1;
                rjudge = 6;
                if (huandao >= 195)
                {
                    rjudge = 7;
                }
            }
        }
    }
}

// ******  舵机控制  ******
void Steer_Control()
{
    switch (rjudge)
    {
    case 0:
        STEER_PWM = 19000;
        pwm_duty(steer, STEER_PWM);
        break;
    case 1:
        if (L / R < N || R / L < N)
        {
            STEER_P = 0.01;
            STEER_D = 1;
        }

        if (((L / R >= N) && (L / R < 4 * N)) || ((R / L >= N) && (R / L < 4 * N)))
        {
            STEER_P = 0.25;
            STEER_D = 5;
        }

        if (((L / R >= 4 * N) && (L / R < 6 * N)) || ((R / L >= 4 * N) && (R / L < 6 * N)))
        {
            STEER_P = 0.8;
            STEER_D = 7.2;
        }

        if (L / R > 6 * N || R / L > 6 * N)
        {
            STEER_P = 1.5;
            STEER_D = 12;
        }

        offset[1] = offset[0];
        offset[0] = 5000.0 * (float)(k * (L - R) / (L + R) + (1 - k) * (LV - RV) / (LV + RV));
        STEER_PWM = (Steer_Mid + STEER_P * offset[0] + STEER_D * (offset[0] - offset[1]));

        if (STEER_PWM >= 24300)
            STEER_PWM = 24300; //          �����?
        else if (STEER_PWM <= 19000)
            STEER_PWM = 19000; // ��ƨ��   �Ҵ���
        pwm_duty(steer, STEER_PWM);
        break;

    case 4:
        STEER_PWM = 21683;
        pwm_duty(steer, STEER_PWM);
        break;

    case 5:
        STEER_PWM = 24200;
        pwm_duty(steer, STEER_PWM);
        break;

    case 6:
        STEER_PWM = 20000;
        pwm_duty(steer, STEER_PWM);
        break;

    case 7:
        STEER_PWM = 21683;
        pwm_duty(steer, STEER_PWM);
        break;

    case 9:
        tingche += 1;
        if (tingche < 123)
        {
            STEER_PWM = 19000;
            pwm_duty(steer, STEER_PWM);
        }
        else
        {
            STEER_PWM = 21683;
            pwm_duty(steer, STEER_PWM);
        }
        break;

    default:
        break;
    }
};

// ******  编码�?  ******
void encoder_filter()
{
    float Encoder[2] = {0};
    Encoder[0] = myabs(qtimer_quad_get(QTIMER_1, QTIMER1_TIMER0_C0));
    Encoder[1] = myabs(qtimer_quad_get(QTIMER_1, QTIMER1_TIMER2_C2));
    qtimer_quad_clear(QTIMER_1, QTIMER1_TIMER0_C0);
    qtimer_quad_clear(QTIMER_1, QTIMER1_TIMER2_C2);
    for (int i = 0; i < 2; i++)
    {
        encoder[i][2] = encoder[i][1];
        encoder[i][1] = encoder[i][0];
        encoder[i][0] = 0.6 * Encoder[i] + 0.3 * encoder[i][1] + 0.1 * encoder[i][2];
    }
    g_fReal_Speed[0] = encoder[0][0];
    g_fReal_Speed[1] = encoder[1][0];
};

// ******  电机控制  ******
void Motor_Control()
{
    if (rjudge == 0)
    {
        pwm_duty(motor1, 6000);
        pwm_duty(motor2, 6000);
    } //**********��**************

    if (rjudge == 9 || brake < 150)
    {
        // gpio_init(D0, GPO, 0, GPIO_PIN_CONFIG);
        // gpio_init(D1, GPO, 1, GPIO_PIN_CONFIG);
        // gpio_init(D2, GPO, 1, GPIO_PIN_CONFIG);
        // gpio_init(D3, GPO, 0, GPIO_PIN_CONFIG);             //*****������?

        // pwm_init(PWM1_MODULE3_CHB_D1, 800, 0);
        // pwm_init(PWM1_MODULE3_CHA_D0, 800, 0);
        // pwm_init(PWM2_MODULE3_CHB_D3, 800, 0);
        // pwm_init(PWM2_MODULE3_CHA_D2, 800, 0);

        // pwm_duty(PWM1_MODULE3_CHB_D1,1000);
        // pwm_duty(PWM2_MODULE3_CHA_D2,1000);

        if (brake < 0)
        {
            pwm_duty(motor1, 0);
            pwm_duty(motor2, 0);
        }
        else
        {
            pwm_duty(motor1, 4000);
            pwm_duty(motor2, 4000);
            brake -= 1;
        }

    } //************ͣ��*********

    else
    {
        if ((L / R >= 5 * N) || (R / L >= 5 * N))
        {
            DIF_ERR[1] = DIF_ERR[0];
            DIF_ERR[0] = 40.0 * (float)(k * (L - R) / (L + R) + (1 - k) * (LV - RV) / (LV + RV));
            DIF_delta = STEER_P * DIF_ERR[0] + STEER_D * (DIF_ERR[0] - DIF_ERR[1]);
            if (DIF_delta > 40)
                DIF_delta = 40;
            if (DIF_delta < -40)
                DIF_delta = -40;
            g_fSet_Speed[0] = speed + DIF_delta;
            g_fSet_Speed[1] = speed - DIF_delta;
        }

        for (int i = 0; i < 2; i++)
        {

            Y[i][1] = Y[i][0];
            Y[i][0] = g_fReal_Speed[i];

            UD[i][0] = ((gama * MOTOR_D) / (gama * MOTOR_D + MOTOR_P)) * UD[i][1] + ((MOTOR_P + MOTOR_D) / (gama * MOTOR_D)) * Y[i][0] + (MOTOR_D / (gama * MOTOR_D)) * Y[i][1];

            SPEED_ERR[i][0] = g_fSet_Speed[i] - g_fReal_Speed[i];
            g_fMOTOR_PWM[i] = MOTOR_P * (SPEED_ERR[i][0] - SPEED_ERR[i][1]) + MOTOR_I * SPEED_ERR[i][0] + UD[i][0];
            SPEED_ERR[i][2] = SPEED_ERR[i][1];
            SPEED_ERR[i][1] = SPEED_ERR[i][0];

            UD[i][1] = UD[i][0];

            if (g_fMOTOR_PWM[i] >= 30000)
                g_fMOTOR_PWM[i] = 30000;
            else if (g_fMOTOR_PWM[i] < 500)
                g_fMOTOR_PWM[i] = 500;
        }
        if (1)
        {
            pwm_duty(motor1, g_fMOTOR_PWM[1]); // ��ƨ�� ����
            pwm_duty(motor2, g_fMOTOR_PWM[0]);
        }
        else
        {
            pwm_duty(motor1, 0);
            pwm_duty(motor2, 0);
        }
    }
};

// ******  屏幕  ******
void show()
{
    // lcd_clear(YELLOW);
    // lcd_showchar(0, 0, 'L');
    // lcd_showfloat(10, 0, L, 4, 0);
    // lcd_showchar(0, 1, 'R');
    // lcd_showfloat(10, 1, R, 4, 0);
    // lcd_showstr(0, 2, "LV");
    // lcd_showfloat(20, 2, LV, 4, 0);
    // lcd_showstr(0, 3, "RV");
    // lcd_showfloat(20, 3, RV, 4, 0);
    // lcd_showstr(25, 4, "M");
    // lcd_showfloat(35, 4, M, 4, 0);
    // lcd_showstr(0, 5, "stP");
    // lcd_showfloat(25, 5, STEER_P, 4, 2);
    // lcd_showstr(0, 6, "hdao");
    // lcd_showfloat(35, 6, huandao, 4, 2);
    // lcd_showstr(0, 7, "tche");
    // lcd_showfloat(40, 7, tingche, 4, 2);

    lcd_displayimage032(mt9v03x_csi_image, MT9V03X_H, MT9V03X_W);

    // lcd_showfloat(0,2,offset[0],6,0);
    // lcd_showfloat(50,2,STEER_PWM,6,0);
    // lcd_showint16(0,3,encoder[0][0]);
    // lcd_showint16(50,3,encoder[1][0]);
    // lcd_showfloat(0,4,g_fMOTOR_PWM[0],6,0);
    // lcd_showfloat(60,4,g_fMOTOR_PWM[1],6,0);
    // lcd_showfloat(0,5,SPEED_ERR[0][0],6,0);
    // lcd_showfloat(50,5,SPEED_ERR[1][1],6,0);
};

// ******  主程�?  ******
int main(void)
{
    DisableGlobalIRQ();
    init_all();
    EnableGlobalIRQ(0);

    while (1)
    {
        // if(mt9v03x_csi_finish_flag)
        // mt9v03x_csi_finish_flag = 0;
        // lcd_displayimage032(mt9v03x_csi_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H);

        // show();
        systick_delay_ms(100);
    }
}

// ******  PIT �?�?服务函数  ******
void PIT_IRQHandler(void)
{
    if (PIT_FLAG_GET(PIT_CH0))
    {
        PIT_FLAG_CLEAR(PIT_CH0);
        read_adc();

        road_judge();

        Steer_Control();
        encoder_filter();
        Motor_Control();
    }

    if (PIT_FLAG_GET(PIT_CH1))
    {
        PIT_FLAG_CLEAR(PIT_CH1);
    }

    if (PIT_FLAG_GET(PIT_CH2))
    {
        PIT_FLAG_CLEAR(PIT_CH2);
    }

    if (PIT_FLAG_GET(PIT_CH3))
    {
        PIT_FLAG_CLEAR(PIT_CH3);
    }

    __DSB();
}

// ******  GPIO �?�?  ******
void GPIO3_Combined_16_31_IRQHandler(void)
{
    if (GET_GPIO_FLAG(D26))
    {
        // gpio_toggle(C27);
        // systick_delay_ms(50);
        CLEAR_GPIO_FLAG(D26);
    }
}

void PIT_Init()
{
    pit_interrupt_ms(PIT_CH0, 6); // ��ʼ��pitͨ��1 7ms
    NVIC_SetPriority(PIT_IRQn, 0);
    pit_interrupt_ms(PIT_CH1, 15); // ��ʼ��pitͨ��2 8ms
    NVIC_SetPriority(PIT_IRQn, 1);
    pit_interrupt_ms(PIT_CH2, 21); // ��ʼ��pitͨ��3 21ms
    NVIC_SetPriority(PIT_IRQn, 2); // �����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ�� ��·PIT����һ��PIT�жϺ���
}

/**
 * @brief  Init ADC config.
 * @retval void
 * @param  void
 */
void ADC_Init(void)
{
    adc_init(ADC_1, adc0, ADC_8BIT);
    adc_init(ADC_1, adc1, ADC_8BIT);
    adc_init(ADC_1, adc2, ADC_8BIT);
    adc_init(ADC_1, adc3, ADC_8BIT);
    adc_init(ADC_1, adc4, ADC_8BIT);
    // adc_init(ADC_1,adc5,ADC_8BIT);
    //  adc_init(ADC_1,adc6,ADC_8BIT);
}

/**
 * @brief  Init PWM configration of steer and motor
 * @retval void
 * @param  void
 */
void PWM_Init(void)
{
    // steer motor init to mid
    pwm_init(steer, 300, 24000);

    // motor init to speed as 800
    pwm_init(PWM1_MODULE3_CHB_D1, 800, 0);
    pwm_init(PWM1_MODULE3_CHA_D0, 800, 0);
    pwm_init(PWM2_MODULE3_CHB_D3, 800, 0);
    pwm_init(PWM2_MODULE3_CHA_D2, 800, 0);
}
