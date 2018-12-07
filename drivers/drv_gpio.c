
 /*
 * File      : drv_gpio.c
 * 
 * This file is auto generate from cubemx xml for STM32H743IITx 
 *
 * Change Logs:
 * Date           Author            Notes
 * 2018-12-07     ZYH            auto generated
 */

#include <rthw.h>
#include <rtdevice.h>
#include <board.h>
#include <drv_gpio.h>

#ifdef RT_USING_PIN
#define __STM32_PIN(index, gpio, gpio_index) (gpio | gpio_index)
#define __STM32_PIN_DEFAULT 0

#define A   (1U << 8)
#define B   (2U << 8)
#define C   (3U << 8)
#define D   (4U << 8)
#define E   (5U << 8)
#define F   (6U << 8)
#define G   (7U << 8)
#define H   (8U << 8)
#define I   (9U << 8)
#define J   (10U << 8)
#define K   (11U << 8)

static GPIO_TypeDef * get_st_gpio(rt_uint16_t gpio_pin)
{
    switch(gpio_pin & 0xFF00)
    {
    case A:
        #ifdef GPIOA
        return GPIOA;
        #endif
    case B:
        #ifdef GPIOB
        return GPIOB;
        #endif
    case C:
        #ifdef GPIOC
        return GPIOC;
        #endif
    case D:
        #ifdef GPIOD
        return GPIOD;
        #endif
    case E:
        #ifdef GPIOE
        return GPIOE;
        #endif
    case F:
        #ifdef GPIOF
        return GPIOF;
        #endif
    case G:
        #ifdef GPIOG
        return GPIOG;
        #endif
    case H:
        #ifdef GPIOH
        return GPIOH;
        #endif
    case I:
        #ifdef GPIOI
        return GPIOI;
        #endif
    case J:
        #ifdef GPIOJ
        return GPIOJ;
        #endif
    case K:
        #ifdef GPIOK
        return GPIOK;
        #endif
    default:
        return RT_NULL;
    }
}

#define get_st_pin(gpio_pin) (0x01 << (gpio_pin&0xFF))

static void drv_clock_enable(rt_uint16_t gpio_pin)
{
    switch(gpio_pin & 0xFF00)
    {
    case A:
        #ifdef __HAL_RCC_GPIOA_CLK_ENABLE
        __HAL_RCC_GPIOA_CLK_ENABLE();
        #endif
        break;
    case B:
        #ifdef __HAL_RCC_GPIOB_CLK_ENABLE
        __HAL_RCC_GPIOB_CLK_ENABLE();
        #endif
        break;
    case C:
        #ifdef __HAL_RCC_GPIOC_CLK_ENABLE
        __HAL_RCC_GPIOC_CLK_ENABLE();
        #endif
        break;
    case D:
        #ifdef __HAL_RCC_GPIOD_CLK_ENABLE
        __HAL_RCC_GPIOD_CLK_ENABLE();
        #endif
        break;
    case E:
        #ifdef __HAL_RCC_GPIOE_CLK_ENABLE
        __HAL_RCC_GPIOE_CLK_ENABLE();
        #endif
        break;
    case F:
        #ifdef __HAL_RCC_GPIOF_CLK_ENABLE
        __HAL_RCC_GPIOF_CLK_ENABLE();
        #endif
        break;
    case G:
        #ifdef __HAL_RCC_GPIOG_CLK_ENABLE
        __HAL_RCC_GPIOG_CLK_ENABLE();
        #endif
        break;
    case H:
        #ifdef __HAL_RCC_GPIOH_CLK_ENABLE
        __HAL_RCC_GPIOH_CLK_ENABLE();
        #endif
        break;
    case I:
        #ifdef __HAL_RCC_GPIOI_CLK_ENABLE
        __HAL_RCC_GPIOI_CLK_ENABLE();
        #endif
        break;
    case J:
        #ifdef __HAL_RCC_GPIOJ_CLK_ENABLE
        __HAL_RCC_GPIOJ_CLK_ENABLE();
        #endif
        break;
    case K:
        #ifdef __HAL_RCC_GPIOK_CLK_ENABLE
        __HAL_RCC_GPIOK_CLK_ENABLE();
        #endif
        break;
    default:
        break;
    }
}

/* STM32 GPIO driver */
static const rt_uint16_t pins[] =
{
    __STM32_PIN_DEFAULT,
    __STM32_PIN(1, E, 2),
    __STM32_PIN(2, E, 3),
    __STM32_PIN(3, E, 4),
    __STM32_PIN(4, E, 5),
    __STM32_PIN(5, E, 6),
    __STM32_PIN_DEFAULT,
    __STM32_PIN(7, I, 8),
    __STM32_PIN(8, C, 13),
    __STM32_PIN(9, C, 14),
    __STM32_PIN(10, C, 15),
    __STM32_PIN(11, I, 9),
    __STM32_PIN(12, I, 10),
    __STM32_PIN(13, I, 11),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(16, F, 0),
    __STM32_PIN(17, F, 1),
    __STM32_PIN(18, F, 2),
    __STM32_PIN(19, F, 3),
    __STM32_PIN(20, F, 4),
    __STM32_PIN(21, F, 5),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(24, F, 6),
    __STM32_PIN(25, F, 7),
    __STM32_PIN(26, F, 8),
    __STM32_PIN(27, F, 9),
    __STM32_PIN(28, F, 10),
    __STM32_PIN(29, H, 0),
    __STM32_PIN(30, H, 1),
    __STM32_PIN_DEFAULT,
    __STM32_PIN(32, C, 0),
    __STM32_PIN(33, C, 1),
    __STM32_PIN(34, C, 2),
    __STM32_PIN(35, C, 3),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(40, A, 0),
    __STM32_PIN(41, A, 1),
    __STM32_PIN(42, A, 2),
    __STM32_PIN(43, H, 2),
    __STM32_PIN(44, H, 3),
    __STM32_PIN(45, H, 4),
    __STM32_PIN(46, H, 5),
    __STM32_PIN(47, A, 3),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(50, A, 4),
    __STM32_PIN(51, A, 5),
    __STM32_PIN(52, A, 6),
    __STM32_PIN(53, A, 7),
    __STM32_PIN(54, C, 4),
    __STM32_PIN(55, C, 5),
    __STM32_PIN(56, B, 0),
    __STM32_PIN(57, B, 1),
    __STM32_PIN(58, B, 2),
    __STM32_PIN(59, F, 11),
    __STM32_PIN(60, F, 12),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(63, F, 13),
    __STM32_PIN(64, F, 14),
    __STM32_PIN(65, F, 15),
    __STM32_PIN(66, G, 0),
    __STM32_PIN(67, G, 1),
    __STM32_PIN(68, E, 7),
    __STM32_PIN(69, E, 8),
    __STM32_PIN(70, E, 9),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(73, E, 10),
    __STM32_PIN(74, E, 11),
    __STM32_PIN(75, E, 12),
    __STM32_PIN(76, E, 13),
    __STM32_PIN(77, E, 14),
    __STM32_PIN(78, E, 15),
    __STM32_PIN(79, B, 10),
    __STM32_PIN(80, B, 11),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(83, H, 6),
    __STM32_PIN(84, H, 7),
    __STM32_PIN(85, H, 8),
    __STM32_PIN(86, H, 9),
    __STM32_PIN(87, H, 10),
    __STM32_PIN(88, H, 11),
    __STM32_PIN(89, H, 12),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(92, B, 12),
    __STM32_PIN(93, B, 13),
    __STM32_PIN(94, B, 14),
    __STM32_PIN(95, B, 15),
    __STM32_PIN(96, D, 8),
    __STM32_PIN(97, D, 9),
    __STM32_PIN(98, D, 10),
    __STM32_PIN(99, D, 11),
    __STM32_PIN(100, D, 12),
    __STM32_PIN(101, D, 13),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(104, D, 14),
    __STM32_PIN(105, D, 15),
    __STM32_PIN(106, G, 2),
    __STM32_PIN(107, G, 3),
    __STM32_PIN(108, G, 4),
    __STM32_PIN(109, G, 5),
    __STM32_PIN(110, G, 6),
    __STM32_PIN(111, G, 7),
    __STM32_PIN(112, G, 8),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(115, C, 6),
    __STM32_PIN(116, C, 7),
    __STM32_PIN(117, C, 8),
    __STM32_PIN(118, C, 9),
    __STM32_PIN(119, A, 8),
    __STM32_PIN(120, A, 9),
    __STM32_PIN(121, A, 10),
    __STM32_PIN(122, A, 11),
    __STM32_PIN(123, A, 12),
    __STM32_PIN(124, A, 13),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(128, H, 13),
    __STM32_PIN(129, H, 14),
    __STM32_PIN(130, H, 15),
    __STM32_PIN(131, I, 0),
    __STM32_PIN(132, I, 1),
    __STM32_PIN(133, I, 2),
    __STM32_PIN(134, I, 3),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(137, A, 14),
    __STM32_PIN(138, A, 15),
    __STM32_PIN(139, C, 10),
    __STM32_PIN(140, C, 11),
    __STM32_PIN(141, C, 12),
    __STM32_PIN(142, D, 0),
    __STM32_PIN(143, D, 1),
    __STM32_PIN(144, D, 2),
    __STM32_PIN(145, D, 3),
    __STM32_PIN(146, D, 4),
    __STM32_PIN(147, D, 5),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(150, D, 6),
    __STM32_PIN(151, D, 7),
    __STM32_PIN(152, G, 9),
    __STM32_PIN(153, G, 10),
    __STM32_PIN(154, G, 11),
    __STM32_PIN(155, G, 12),
    __STM32_PIN(156, G, 13),
    __STM32_PIN(157, G, 14),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(160, G, 15),
    __STM32_PIN(161, B, 3),
    __STM32_PIN(162, B, 4),
    __STM32_PIN(163, B, 5),
    __STM32_PIN(164, B, 6),
    __STM32_PIN(165, B, 7),
    __STM32_PIN_DEFAULT,
    __STM32_PIN(167, B, 8),
    __STM32_PIN(168, B, 9),
    __STM32_PIN(169, E, 0),
    __STM32_PIN(170, E, 1),
    __STM32_PIN_DEFAULT,
    __STM32_PIN_DEFAULT,
    __STM32_PIN(173, I, 4),
    __STM32_PIN(174, I, 5),
    __STM32_PIN(175, I, 6),
    __STM32_PIN(176, I, 7),

};

struct pin_irq_map
{
    rt_uint16_t pinbit;
    IRQn_Type irqno;
};

static const struct pin_irq_map pin_irq_map[] =
{
    {GPIO_PIN_0, EXTI0_IRQn},
    {GPIO_PIN_1, EXTI1_IRQn},
    {GPIO_PIN_2, EXTI2_IRQn},
    {GPIO_PIN_3, EXTI3_IRQn},
    {GPIO_PIN_4, EXTI4_IRQn},
    {GPIO_PIN_5, EXTI9_5_IRQn},
    {GPIO_PIN_6, EXTI9_5_IRQn},
    {GPIO_PIN_7, EXTI9_5_IRQn},
    {GPIO_PIN_8, EXTI9_5_IRQn},
    {GPIO_PIN_9, EXTI9_5_IRQn},
    {GPIO_PIN_10, EXTI15_10_IRQn},
    {GPIO_PIN_11, EXTI15_10_IRQn},
    {GPIO_PIN_12, EXTI15_10_IRQn},
    {GPIO_PIN_13, EXTI15_10_IRQn},
    {GPIO_PIN_14, EXTI15_10_IRQn},
    {GPIO_PIN_15, EXTI15_10_IRQn},
};

struct rt_pin_irq_hdr pin_irq_hdr_tab[] =
{
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
};

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])
static rt_uint16_t get_pin(uint8_t pin)
{
    rt_uint16_t gpio_pin = __STM32_PIN_DEFAULT;
    if (pin < ITEM_NUM(pins))
    {
        gpio_pin = pins[pin];
    }
    return gpio_pin;
};

static void stm32_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    rt_uint16_t gpio_pin;
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return;
    }
    HAL_GPIO_WritePin(get_st_gpio(gpio_pin), get_st_pin(gpio_pin), (GPIO_PinState)value);
}

static int stm32_pin_read(rt_device_t dev, rt_base_t pin)
{
    rt_uint16_t gpio_pin;
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return PIN_LOW;
    }
    return HAL_GPIO_ReadPin(get_st_gpio(gpio_pin), get_st_pin(gpio_pin));
}

static void stm32_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    rt_uint16_t gpio_pin;
    GPIO_InitTypeDef GPIO_InitStruct;
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return;
    }
    /* GPIO Periph clock enable */
    drv_clock_enable(gpio_pin);
    /* Configure GPIO_InitStructure */
    GPIO_InitStruct.Pin = get_st_pin(gpio_pin);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    if (mode == PIN_MODE_INPUT)
    {
        /* input setting: not pull. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    }
    else if (mode == PIN_MODE_INPUT_PULLUP)
    {
        /* input setting: pull up. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
    }
    else if (mode == PIN_MODE_INPUT_PULLDOWN)
    {
        /* input setting: pull down. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    }
    else if (mode == PIN_MODE_OUTPUT_OD)
    {
        /* output setting: od. */
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    }
    HAL_GPIO_Init(get_st_gpio(gpio_pin), &GPIO_InitStruct);
}

static const struct pin_irq_map *get_pin_irq_map(rt_uint16_t gpio_pin)
{
    rt_int32_t mapindex = gpio_pin & 0xFF;
    if (mapindex < 0 || mapindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_NULL;
    }
    return &pin_irq_map[mapindex];
};

static rt_err_t stm32_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                              rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    rt_uint16_t gpio_pin;
    rt_base_t level;
    rt_int32_t irqindex = -1;
    GPIO_InitTypeDef GPIO_InitStruct;
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return RT_ENOSYS;
    }
    irqindex = gpio_pin&0xFF;
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_ENOSYS;
    }
    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[irqindex].pin == pin &&
            pin_irq_hdr_tab[irqindex].hdr == hdr &&
            pin_irq_hdr_tab[irqindex].mode == mode &&
            pin_irq_hdr_tab[irqindex].args == args)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    if (pin_irq_hdr_tab[irqindex].pin != -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EBUSY;
    }
    pin_irq_hdr_tab[irqindex].pin = pin;
    pin_irq_hdr_tab[irqindex].hdr = hdr;
    pin_irq_hdr_tab[irqindex].mode = mode;
    pin_irq_hdr_tab[irqindex].args = args;
    rt_hw_interrupt_enable(level);
    
    /* GPIO Periph clock enable */
    drv_clock_enable(gpio_pin);
    /* Configure GPIO_InitStructure */
    GPIO_InitStruct.Pin = get_st_pin(gpio_pin);
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    switch (pin_irq_hdr_tab[irqindex].mode)
    {
    case PIN_IRQ_MODE_RISING:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        break;
    case PIN_IRQ_MODE_FALLING:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        break;
    case PIN_IRQ_MODE_RISING_FALLING:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        break;
    }
    HAL_GPIO_Init(get_st_gpio(gpio_pin), &GPIO_InitStruct);
    
    return RT_EOK;
}

static rt_err_t stm32_pin_detach_irq(struct rt_device *device, rt_int32_t pin)
{
    rt_uint16_t gpio_pin;
    rt_base_t level;
    rt_int32_t irqindex = -1;
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return RT_ENOSYS;
    }
    irqindex = gpio_pin&0xFF;
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_ENOSYS;
    }
    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[irqindex].pin == -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    pin_irq_hdr_tab[irqindex].pin = -1;
    pin_irq_hdr_tab[irqindex].hdr = RT_NULL;
    pin_irq_hdr_tab[irqindex].mode = 0;
    pin_irq_hdr_tab[irqindex].args = RT_NULL;
    rt_hw_interrupt_enable(level);
    return RT_EOK;
}

static rt_err_t stm32_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                              rt_uint32_t enabled)
{
    rt_uint16_t gpio_pin;
    const struct pin_irq_map *irqmap;
    rt_base_t level;
    rt_int32_t irqindex = -1;
    
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return RT_ENOSYS;
    }
    if (enabled == PIN_IRQ_ENABLE)
    {
        irqindex = gpio_pin&0xFF;
        if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
        {
            return RT_ENOSYS;
        }
        level = rt_hw_interrupt_disable();
        if (pin_irq_hdr_tab[irqindex].pin == -1)
        {
            rt_hw_interrupt_enable(level);
            return RT_ENOSYS;
        }
        irqmap = &pin_irq_map[irqindex];
        HAL_NVIC_ClearPendingIRQ(irqmap->irqno);
        __HAL_GPIO_EXTI_CLEAR_IT(irqindex);
        __HAL_GPIO_EXTI_CLEAR_FLAG(irqindex);
        HAL_NVIC_SetPriority(irqmap->irqno, 5, 0);
        HAL_NVIC_EnableIRQ(irqmap->irqno);
        rt_hw_interrupt_enable(level);
    }
    else if (enabled == PIN_IRQ_DISABLE)
    {
        irqmap = get_pin_irq_map(gpio_pin);
        if (irqmap == RT_NULL)
        {
            return RT_ENOSYS;
        }
        HAL_NVIC_DisableIRQ(irqmap->irqno);
        HAL_NVIC_ClearPendingIRQ(irqmap->irqno);
    }
    else
    {
        return RT_ENOSYS;
    }
    return RT_EOK;
}

const static struct rt_pin_ops _stm32_pin_ops =
{
    stm32_pin_mode,
    stm32_pin_write,
    stm32_pin_read,
    stm32_pin_attach_irq,
    stm32_pin_detach_irq,
    stm32_pin_irq_enable,
};

int rt_hw_pin_init(void)
{
    int result;
    result = rt_device_pin_register("pin", &_stm32_pin_ops, RT_NULL);
    return result;
}
INIT_BOARD_EXPORT(rt_hw_pin_init);

static void pin_irq_hdr(uint16_t GPIO_Pin)
{
    int irqno = 0;
    for(irqno = 0; irqno < 16; irqno ++)
    {
        if((0x01 << irqno) == GPIO_Pin)
        {
            break;
        }
    }
    if(irqno == 16)return;
    if (pin_irq_hdr_tab[irqno].hdr)
    {
        pin_irq_hdr_tab[irqno].hdr(pin_irq_hdr_tab[irqno].args);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    pin_irq_hdr(GPIO_Pin);
}

void EXTI0_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    rt_interrupt_leave();
}

void EXTI1_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    rt_interrupt_leave();
}

void EXTI2_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    rt_interrupt_leave();
}

void EXTI3_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    rt_interrupt_leave();
}

void EXTI4_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
    rt_interrupt_leave();
}

void EXTI9_5_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
    rt_interrupt_leave();
}

void EXTI15_10_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
    rt_interrupt_leave();
}
#endif
