#include "stm32f1xx_hal.h"
#include "gpio.h"

#define BLACK  1
#define WHITE  0

/* ================================================================
 * 去抖滤波 — 每个传感器连读3次，取多数（2:1就算数）
 *
 * 为什么这么做？
 *   传感器在黑白边界处会疯狂抖动（1→0→1→0...），
 *   如果每5ms读到不同的值，电机会左右摇摆 → 时快时慢。
 *   连读3次取多数，过滤掉偶尔的毛刺。
 *
 * 时间开销：
 *   每次读取 ~3μs，3次约10μs。5ms的控制周期只占0.2%，完全不影响。
 * ================================================================ */

/* 短延时 ~1μs，只给GPIO电平稳定用的 */
static void tiny_delay(void)
{
    volatile uint8_t i = 10;
    while(i--);
}

/* 去抖核心：读3次，返回多数结果（0或1） */
static uint8_t debounce_read(GPIO_TypeDef *port, uint16_t pin)
{
    uint8_t r1, r2, r3;

    r1 = HAL_GPIO_ReadPin(port, pin);
    tiny_delay();
    r2 = HAL_GPIO_ReadPin(port, pin);
    tiny_delay();
    r3 = HAL_GPIO_ReadPin(port, pin);

    /* 多数表决：3次里至少2次是1 → 返回1，否则返回0 */
    return (r1 + r2 + r3 >= 2) ? 1 : 0;
}

/* ================================================================
 * 4个传感器读取函数
 *
 * 传感器排列（从左到右）：L1 → L2 → R2 → R1
 *
 * 坐标系：
 *    左偏 → error 为负（车偏左，需要右转）
 *    右偏 → error 为正（车偏右，需要左转）
 *
 * 权重：
 *    L1（最左）→ -3，L2（次左）→ -1
 *    R2（次右）→ +1，R1（最右）→ +3
 *
 * 越靠外侧的传感器权重越大——因为它意味着"偏得更多了"。
 * 比如只有最左边L1看到黑线 → 车已经很偏左了 → 需要-3的大幅度右转。
 * ================================================================ */

uint8_t Track_L1(void)
{
    return debounce_read(GPIOA, GPIO_PIN_2);  // PA2 — 最左边
}

uint8_t Track_L2(void)
{
    return debounce_read(GPIOA, GPIO_PIN_1);  // PA1 — 次左边
}

uint8_t Track_R2(void)
{
    return debounce_read(GPIOA, GPIO_PIN_0);  // PA0 — 次右边
}

uint8_t Track_R1(void)
{
    return debounce_read(GPIOA, GPIO_PIN_3);  // PA3 — 最右边
}
