#include "app_remote_rx.h"
#include "usart.h"
#include "string.h"
#include "Dog.h"
#include "math.h"

#define PI 3.14159265f
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);
static void handle_frame(const uint8_t f[data_len_1]);
static void handle_mode_frame(const uint8_t f[mode_len]);

uint8_t clim_up_flag = 0;
uint8_t step_down_flag = 0;

// 解析缓存
struct
{
    uint8_t idx;
    uint8_t expect_len;
    uint8_t type;
    uint8_t buf[data_len_1];
} rx_cache = {0};

UART_HandleTypeDef *s_huart = NULL; // 串口句柄指针
osThreadId s_rx_thread_id = NULL;   // 接收线程ID

// dma接收数据的buf
uint8_t dma_rx_buf[RX_BUF_SIZE];
// 处理数据的buf
uint8_t process_buf[RX_BUF_SIZE];
// 等待处理数据的长度
uint16_t process_len = 0;

volatile remote_rx_data rx_data = {0};

void app_remote_rx_init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    memset((void *)&rx_data, 0, sizeof(rx_data));
    memset(&rx_cache, 0, sizeof(rx_cache));
}

void app_remote_rx_start(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(s_huart);

    __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);

    HAL_UART_Receive_DMA(s_huart, dma_rx_buf, RX_BUF_SIZE);

    __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
}
void app_remote_thread(osThreadId id)
{
    s_rx_thread_id = id;
}

void UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL || huart == NULL)
        return;
    if (huart->Instance == s_huart->Instance)
    {
        // 检查是否是IDLE中断触发的
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
        {
            // 1.清理中断标志位
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            // 2.关闭DMA防止在拷贝数据的时候打断
            HAL_UART_DMAStop(huart);
            // 3.设定的总长度-dma还没有搬完的数据
            uint16_t rx_len = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

            if (rx_len > 0 && rx_len <= RX_BUF_SIZE)
            {
                // 4.将数据拷贝到处理区
                memcpy(process_buf, dma_rx_buf, rx_len);
                process_len = rx_len;
                // 5.任务通知处理数据
                if (s_rx_thread_id != NULL)
                    osSignalSet(s_rx_thread_id, rx_flag_data);
            }
        }
        // 6.清空旧缓冲区重新启动dma接收下一帧
        HAL_UART_Receive_DMA(s_huart, dma_rx_buf, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    }
}

void remote_rx_on(const uint8_t *data, uint16_t len) // 处理数据帧并解析数据
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t c = data[i];

        switch (rx_cache.idx)
        {
        case 0:
            if (c == SOF0)
            {
                rx_cache.buf[0] = SOF0;
                rx_cache.idx = 1;
            }
            break;
        case 1:
            if (c == SOF1)
            {
                rx_cache.buf[1] = SOF1;
                rx_cache.type = 0;
                rx_cache.expect_len = data_len_1;
                rx_cache.idx = 2;
            }
            else if (c == SOF1_MODE)
            {
                rx_cache.buf[1] = SOF1_MODE;
                rx_cache.type = 1;
                rx_cache.expect_len = mode_len;
                rx_cache.idx = 2;
            }
            else if (c == SOF0)
            {
                rx_cache.buf[0] = SOF0;
                rx_cache.idx = 1;
            }
            else
            {
                rx_cache.idx = 0;
            }
            break;

        default:
            if (rx_cache.idx < data_len_1)
            {
                rx_cache.buf[rx_cache.idx++] = c;
            }
            else
            {
                rx_cache.idx = 0;
                break;
            }

            if (rx_cache.idx >= rx_cache.expect_len)
            {
                if (rx_cache.type == 0)
                {
                    handle_frame(rx_cache.buf);
                }
                else
                {
                    handle_mode_frame(rx_cache.buf);
                }
                rx_cache.idx = 0;
            }
            break;
        }
    }
}
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

static void handle_frame(const uint8_t f[data_len_1])
{
    uint16_t crc_calc = crc16_ccitt(f, 10);
    uint16_t crc_rc = (uint16_t)f[10] | ((uint16_t)f[11] << 8);

    if (crc_calc != crc_rc)
    {
        rx_data.crc_err_cnt++;
        return;
    }

    uint8_t seq = f[2];

    int16_t joy_x = (int16_t)((uint16_t)f[3] | ((uint16_t)f[4] << 8));
    int16_t joy_y = (int16_t)((uint16_t)f[5] | ((uint16_t)f[6] << 8));
    int16_t joy_z = (int16_t)((uint16_t)f[7] | ((uint16_t)f[8] << 8));

    uint8_t flags = f[9];

    if (rx_data.ok_cnt > 0)
    {
        uint8_t expected = (uint8_t)(rx_data.seq + 1);

        if (seq != expected)
        {
            uint8_t diff = (uint8_t)(seq - expected);
            rx_data.lost_cnt += diff;
        }
    }
    rx_data.seq = seq;
    rx_data.x = (float)joy_x;
    rx_data.y = (float)joy_y;
    rx_data.z = (float)joy_z;
    rx_data.flag = flags;

    rx_data.last_rx_tick = HAL_GetTick();
    rx_data.ok_cnt++;
    rx_data.link_ok = 1;
    rx_data.new_data = 1;
}
static void handle_mode_frame(const uint8_t f[mode_len])
{
    uint16_t crc_calc = crc16_ccitt(f, 4);
    uint16_t crc_rc = (uint16_t)f[4] | ((uint16_t)f[5] << 8);

    if (crc_calc != crc_rc)
    {
        rx_data.crc_err_cnt++;
        return;
    }

    uint8_t mode = f[3];

    rx_data.mode = mode;

    rx_data.last_rx_tick = HAL_GetTick();
    rx_data.ok_cnt++;
    rx_data.link_ok = 1;
    rx_data.new_data = 1;

    switch (rx_data.mode)
    {
    case 1:
        //			channel_set(3, 1);
        ;
        break;

    case 2:
        //			channel_set(3, 0);

        break;

    case 4:
        //			channel_set(7, 1);

        break;

    case 8:
        //			channel_set(7, 0);
        break;

    default:
        break;
    }
}

// -------------------------- 遥控器输入处理 ------------------------------------------------------------------------------------------------

/**
 * @brief 处理遥控器数据，包含死区处理和一阶低通滤波
 */
void app_remote_data_process(Dog *dog)
{
    float angle = atan2f(rx_data.y, rx_data.x);
    float r2 = rx_data.x * rx_data.x + rx_data.y * rx_data.y;

    if (r2 > 100000.0f)
    {
        if (angle > PI / 4.0f && angle <= PI * 3.0f / 4.0f)
        {
            dog->state = WALK_FORWARD;
        }
        else if (angle >= -PI / 4.0f && angle <= PI / 4.0f)
        {
            dog->state = TURN_LEFT;
        }
        else if (angle >= -PI * 3.0f / 4.0f && angle < -PI / 4.0f)
        {
            dog->state = WALK_BACK;
        }
        else if ((angle >= -PI && angle < -PI * 3.0f / 4.0f) ||
                 (angle > PI * 3.0f / 4.0f && angle <= PI))
        {
            dog->state = TURN_RIGHT;
        }
        else
        {
            dog->state = STAND_UP_;
        }
    }
    else
    {
        dog->state = STAND_UP_;
    }
}