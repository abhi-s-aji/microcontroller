/**
 ******************************************************************************
 * @file    main.c
 * @brief   STM32F103 Digital Door Lock with Password Authentication
 *          - Matrix Keypad (4x4) on PA0-PA7
 *          - 16x2 LCD via PCF8574 I2C Backpack on PB6 (SCL) & PB7 (SDA)
 *          - Green LED on PB0 (Access Granted / System Active)
 *          - Red LED on PB1 (Access Denied / Wrong Password)
 *          - Buzzer on PB10 (Alarm / Warning)
 *          - Non-volatile Flash storage for configured password persistence
 ******************************************************************************
 */

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */
#define PASSWORD_MAX_LENGTH         8
#define FLASH_STORAGE_EXPECTED_ADDR 0x0800FC00UL  /* Expected Page 63 of 64KB Flash */
#define FLASH_CONFIG_MAGIC          0x50415353UL  /* "PASS" in ASCII */

/* Linker-provided flash storage boundary symbols */
extern const uint8_t _flash_storage_start;
#define FLASH_STORAGE_PAGE_ADDR     ((uint32_t)&_flash_storage_start)

/* Keypad Keys */
#define KEY_CONFIRM                 '#'
#define KEY_CLEAR                   '*'
#define KEY_LOGOUT                  '*'
#define KEY_ALT_LOGOUT              'D'

/* Hardware Pin Definitions */
#define GREEN_LED_PORT              GPIOB
#define GREEN_LED_PIN               GPIO_PIN_0

#define RED_LED_PORT                GPIOB
#define RED_LED_PIN                 GPIO_PIN_1

#define BUZZER_PORT                 GPIOB
#define BUZZER_PIN                  GPIO_PIN_10

/* Onboard Status LED on STM32 Blue Pill (PC13, Active LOW) */
#define ONBOARD_LED_PORT            GPIOC
#define ONBOARD_LED_PIN             GPIO_PIN_13
#define ONBOARD_LED_ON()            HAL_GPIO_WritePin(ONBOARD_LED_PORT, ONBOARD_LED_PIN, GPIO_PIN_RESET)
#define ONBOARD_LED_OFF()           HAL_GPIO_WritePin(ONBOARD_LED_PORT, ONBOARD_LED_PIN, GPIO_PIN_SET)
#define ONBOARD_LED_TOGGLE()        HAL_GPIO_TogglePin(ONBOARD_LED_PORT, ONBOARD_LED_PIN)

/* I2C LCD Configuration */
#define LCD_DEFAULT_ADDR            0x27
#define LCD_ALT_ADDR                0x3F
#define LCD_RS                      0x01
#define LCD_RW                      0x02
#define LCD_EN                      0x04
#define LCD_BL                      0x08

/* LCD Pinout Profiles */
typedef enum {
    LCD_PROFILE_STANDARD = 0, /* Standard/YwRobot: P0=RS, P1=RW, P2=EN, P3=BL, P4-P7=D4-D7 */
    LCD_PROFILE_MJKDZ    = 1  /* MJKDZ: P0-P3=D4-D7, P4=RS, P5=RW, P6=EN, P7=BL */
} LcdPinProfile_t;

/* Diagnostic Status Codes */
typedef enum {
    LCD_STATUS_OK          = 0, /* Device ACKed and all commands transmitted with HAL_OK */
    LCD_STATUS_BUS_STUCK   = 1, /* SDA or SCL line held LOW prior to recovery */
    LCD_STATUS_NO_DEVICE   = 2, /* No I2C device acknowledged across scanned addresses */
    LCD_STATUS_WRITE_ERROR = 3  /* Device ACKed address, but data byte transmission failed */
} LcdStatus_t;

typedef struct {
    volatile LcdStatus_t status;
    volatile uint8_t     found_addr_7bit;  /* 7-bit address (e.g. 0x27 or 0x3F), 0 if none */
    volatile uint8_t     pin_profile;      /* 0 = Standard, 1 = MJKDZ */
    volatile uint32_t    tx_success_count; /* Number of successful I2C byte writes */
    volatile uint32_t    tx_fail_count;    /* Number of failed I2C byte writes */
    volatile HAL_StatusTypeDef last_i2c_status;
    volatile uint32_t    last_i2c_error_code;
} LcdDiagnostic_t;

/* System Authentication States */
typedef enum {
    STATE_SETUP_PASSWORD,
    STATE_CONFIRM_PASSWORD,
    STATE_ENTER_PASSWORD,
    STATE_AUTHENTICATED
} AuthState_t;

/* Flash Storage Structure (16-bit half-word aligned for STM32F1 Flash) */
typedef struct __attribute__((aligned(2))) {
    uint32_t magic;
    uint16_t length;
    char password[PASSWORD_MAX_LENGTH + 1];
    uint8_t reserved; /* padding to ensure even byte count */
} FlashConfig_t;

/* ============================================================================
 * Global & Static Variables
 * ============================================================================ */
static I2C_HandleTypeDef hi2c1;
static uint16_t lcd_i2c_addr = (LCD_DEFAULT_ADDR << 1);
static LcdPinProfile_t g_lcd_pin_profile = LCD_PROFILE_STANDARD;
static uint8_t g_lcd_backlight = 1;

/* Global diagnostic state - inspectable live via ST-Link / SWD debugger */
volatile LcdDiagnostic_t g_lcd_diag = {0};

/* Keypad Row & Column Pin Definitions */
static GPIO_TypeDef *rowPorts[4] = { GPIOA, GPIOA, GPIOA, GPIOA };
static const uint16_t rowPins[4] = { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3 };
static const uint16_t colPins[4] = { GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7 };

static const char keys[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

/* Password State Variables */
static AuthState_t current_state = STATE_ENTER_PASSWORD;
static char saved_password[PASSWORD_MAX_LENGTH + 1] = {0};
static char setup_temp_password[PASSWORD_MAX_LENGTH + 1] = {0};
static char entered_password[PASSWORD_MAX_LENGTH + 1] = {0};
static uint8_t entered_length = 0;

/* ============================================================================
 * Function Declarations
 * ============================================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void DWT_Init(void);
static void delay_us(uint32_t us);

/* Keypad Driver Functions */
char scanKeypad(void);
char getKey(void);

/* LCD Driver Functions */
void lcdSetBacklight(uint8_t state);
void lcdSetProfile(LcdPinProfile_t profile);
HAL_StatusTypeDef lcdExpanderWrite(uint8_t value);
HAL_StatusTypeDef lcdSendNibble(uint8_t nibble4, uint8_t rs);
HAL_StatusTypeDef lcdSend(uint8_t value, uint8_t dataMode);
HAL_StatusTypeDef lcdCommand(uint8_t command);
HAL_StatusTypeDef lcdPrintChar(char c);
HAL_StatusTypeDef lcdClear(void);
HAL_StatusTypeDef lcdSetCursor(uint8_t column, uint8_t row);
HAL_StatusTypeDef lcdPrint(const char *text);
HAL_StatusTypeDef lcdPrintMasked(uint8_t length);
uint8_t lcdInit(void);
void showScreen(const char *line1, const char *line2);
void showEntryScreen(const char *title, uint8_t length);
static uint8_t I2C_Bus_Recovery(void);
static uint8_t I2C_ProbeAddress(uint8_t addr7);
static uint8_t I2C_ScanForLCD(void);

/* Flash Storage Functions */
static uint8_t Flash_LoadPassword(char *out_pass, uint32_t max_len);
static HAL_StatusTypeDef Flash_SavePassword(const char *new_pass);

/* Protected Peripherals Control */
static void ProtectedPeripheral_Activate(void);
static void ProtectedPeripheral_Deactivate(void);

/* Password Manager Functions */
void Password_Start(void);
void Password_ProcessKey(char key);
uint8_t Password_Check(const char *entered);
void Password_GrantAccess(void);
void Password_DenyAccess(void);
void Password_Logout(void);
void Password_Setup(void);
void Password_ConfirmSetup(void);

/* ============================================================================
 * Cortex-M3 Core Exception Handlers
 * ============================================================================ */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ============================================================================
 * Main Function
 * ============================================================================ */
int main(void)
{
    HAL_Init();

#define RUN_HEARTBEAT_TEST 0
#if RUN_HEARTBEAT_TEST
    /* ========================================================================
     * TEMPORARY DIAGNOSTIC ISOLATION TEST:
     * Independent firmware heartbeat at the very beginning of main().
     * Toggles onboard PC13 LED every 500 ms indefinitely.
     * Does NOT initialize or access LCD, keypad, I2C, external LEDs, or buzzer.
     * This isolates firmware execution from all peripheral issues.
     *
     * To restore original door-lock firmware after this test:
     * Change RUN_HEARTBEAT_TEST from 1 to 0 and recompile.
     * ======================================================================== */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500);
    }
#endif

    SystemClock_Config();
    DWT_Init();
    MX_GPIO_Init();
    MX_I2C1_Init();

    /* Initialize LCD and scan for I2C backpack address */
    uint8_t lcd_ok = lcdInit();

    /* ========================================================================
     * Diagnostic Verification & Status Reporting:
     * Robust, bounded diagnostic reporting distinguishing:
     * - I2C peripheral/bus stuck (5 fast flashes)
     * - No device acknowledged (1 blink)
     * - Write error / command timeout (3 blinks)
     * - Success: Green LED ON, PC13 ON, test pattern displayed
     * ======================================================================== */
    if (lcd_ok)
    {
        /* Green LED ON + Onboard PC13 LED ON: Healthy I2C communication */
        HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
        ONBOARD_LED_ON();

        /* Display test pattern */
        showScreen("LCD TEST OK", "STM32 Blue Pill");

        /* Hold diagnostic screen for 3 seconds or until a key is pressed.
         * Interactive feature: Press 'D' to test MJKDZ profile, 'A' for Standard profile.
         */
        uint32_t t_start = HAL_GetTick();
        while (HAL_GetTick() - t_start < 3000)
        {
            char k = scanKeypad();
            if (k == 'D' || k == 'd')
            {
                /* Switch to MJKDZ profile and re-initialize */
                lcdSetProfile(LCD_PROFILE_MJKDZ);
                lcdInit();
                showScreen("MJKDZ PROFILE", "LCD TEST OK");
                HAL_Delay(500);
            }
            else if (k == 'A' || k == 'a')
            {
                /* Switch to Standard profile and re-initialize */
                lcdSetProfile(LCD_PROFILE_STANDARD);
                lcdInit();
                showScreen("STD PROFILE", "LCD TEST OK");
                HAL_Delay(500);
            }
            else if (k != '\0')
            {
                /* Any other key exits diagnostic immediately */
                break;
            }
            HAL_Delay(50);
        }

        HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
        ONBOARD_LED_OFF();
    }
    else
    {
        /* Communication failure: Enter bounded diagnostic blink pattern.
         * Pattern runs for 10 cycles (~15s) or until keypad '#' is pressed,
         * ensuring the user can clearly observe the exact fault code!
         */
        uint8_t blinks = 1;
        if (g_lcd_diag.status == LCD_STATUS_NO_DEVICE)
        {
            blinks = 1; /* 1 blink = No I2C device responded (check wiring/power) */
        }
        else if (g_lcd_diag.status == LCD_STATUS_BUS_STUCK)
        {
            blinks = 5; /* 5 fast blinks = SDA/SCL line stuck low */
        }
        else if (g_lcd_diag.status == LCD_STATUS_WRITE_ERROR)
        {
            blinks = 3; /* 3 blinks = Address ACKed, but data write failed */
        }

        for (int cycle = 0; cycle < 10; cycle++)
        {
            if (scanKeypad() == KEY_CONFIRM)
            {
                break; /* User pressed '#' to proceed to authentication */
            }

            for (int b = 0; b < blinks; b++)
            {
                HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
                ONBOARD_LED_ON();
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
                HAL_Delay(120);
                HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
                ONBOARD_LED_OFF();
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
                HAL_Delay(120);
            }
            HAL_Delay(800);
        }
    }

    Password_Start();

    while (1)
    {
        char key = getKey();
        if (key != '\0')
        {
            Password_ProcessKey(key);
        }
        HAL_Delay(10);
    }
}

/* ============================================================================
 * System Clock Configuration
 * ============================================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* If external crystal fails, fall back to internal HSI */
        memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
        RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
        HAL_RCC_OscConfig(&RCC_OscInitStruct);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    /* Update CMSIS SystemCoreClock variable to match current configuration */
    SystemCoreClockUpdate();
}

/* ============================================================================
 * Microsecond Delay (Calibrated to SystemCoreClock)
 * ============================================================================ */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    /* Accurate microsecond delay based on actual SystemCoreClock.
     * At 72 MHz, 72 cycles per microsecond.
     * Each loop iteration (__NOP x 4 + decrement + branch) takes ~8 CPU cycles.
     */
    uint32_t count = us * (SystemCoreClock / 8000000UL);
    if (count == 0) count = 1;
    while (count--)
    {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }
}

/* ============================================================================
 * GPIO Initialization
 * ============================================================================ */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO Clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 1. Keypad Rows: Outputs (PA0, PA1, PA2, PA3) */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Idle state for rows: HIGH */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);

    /* 2. Keypad Columns: Inputs with Pull-ups (PA4, PA5, PA6, PA7) */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 3. Indicators & Buzzer: Outputs (PB0 = Green LED, PB1 = Red LED, PB10 = Buzzer) */
    GPIO_InitStruct.Pin = GREEN_LED_PIN | RED_LED_PIN | BUZZER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Initialize both LEDs and Buzzer to OFF */
    HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);

    /* 4. Onboard Blue Pill User LED: Output (PC13, Active LOW) */
    GPIO_InitStruct.Pin = ONBOARD_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ONBOARD_LED_PORT, &GPIO_InitStruct);
    ONBOARD_LED_OFF();
}

/* ============================================================================
 * I2C1 Bus Recovery & Initialization
 * ============================================================================ */
static uint8_t I2C_Bus_Recovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t sda_was_low = 0;

    /* 1. De-initialize I2C1 peripheral and disable it.
     *    hi2c1.Instance must be set before __HAL_I2C_DISABLE or HAL_I2C_DeInit
     *    are called; both macros/functions dereference Instance. The handle is
     *    a static BSS variable (zero-initialized), so Instance is NULL until
     *    explicitly assigned — assigning it here prevents a NULL-deref HardFault. */
    hi2c1.Instance = I2C1;
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_I2C_DISABLE(&hi2c1);
    HAL_I2C_DeInit(&hi2c1);

    /* 2. Configure PB6 (SCL) and PB7 (SDA) as standard GPIO Open-Drain outputs */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Release both lines HIGH */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    delay_us(20);

    /* Check if SDA is held low by a slave device */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET)
    {
        sda_was_low = 1;
    }

    /* 3. Send clock pulses on SCL to clear any slave holding SDA */
    for (int i = 0; i < 16; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
        {
            break; /* SDA released */
        }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        delay_us(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        delay_us(10);
    }

    /* 4. Generate STOP condition: SDA goes LOW while SCL is HIGH, then SDA goes HIGH */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    delay_us(20);

    /* 5. Force and release reset of I2C1 peripheral to clear errata BUSY flag */
    __HAL_RCC_I2C1_FORCE_RESET();
    delay_us(10);
    __HAL_RCC_I2C1_RELEASE_RESET();
    delay_us(10);

    /* 6. Reconfigure pins to Alternate Function Open Drain */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 7. Re-initialize I2C1 peripheral */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);

    return sda_was_low;
}

static void MX_I2C1_Init(void)
{
    I2C_Bus_Recovery();
}

static uint8_t I2C_ProbeAddress(uint8_t addr7)
{
    uint16_t dev_addr = (uint16_t)(addr7 << 1);

    /* Clear any locked state or pending errors */
    __HAL_UNLOCK(&hi2c1);
    hi2c1.State = HAL_I2C_STATE_READY;
    hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF);
    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_BERR);

    if (hi2c1.Instance->SR2 & I2C_SR2_BUSY)
    {
        I2C_Bus_Recovery();
    }

    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, dev_addr, 2, 20);

    /* Always ensure handle is left unlocked and ready */
    __HAL_UNLOCK(&hi2c1);
    hi2c1.State = HAL_I2C_STATE_READY;
    hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF);
    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_BERR);

    if (status == HAL_OK)
    {
        return 1;
    }
    return 0;
}

static uint8_t I2C_ScanForLCD(void)
{
    /* Prioritized list of known PCF8574 / PCF8574A backpack addresses */
    static const uint8_t priority_addrs[] = {
        0x27, 0x3F,
        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26,
        0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E
    };

    for (size_t i = 0; i < sizeof(priority_addrs) / sizeof(priority_addrs[0]); i++)
    {
        if (I2C_ProbeAddress(priority_addrs[i]))
        {
            lcd_i2c_addr = (uint16_t)(priority_addrs[i] << 1);
            return 1;
        }
    }

    /* Fallback: full 7-bit standard I2C address range */
    for (uint8_t addr = 0x08; addr <= 0x77; addr++)
    {
        if (I2C_ProbeAddress(addr))
        {
            lcd_i2c_addr = (uint16_t)(addr << 1);
            return 1;
        }
    }

    lcd_i2c_addr = (LCD_DEFAULT_ADDR << 1);
    return 0;
}

/* ============================================================================
 * Keypad Driver
 * ============================================================================ */
char scanKeypad(void)
{
    for (int row = 0; row < 4; row++)
    {
        /* Drive current row LOW */
        HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_RESET);
        delay_us(5); /* Microsecond line settling delay (non-blocking) */

        for (int col = 0; col < 4; col++)
        {
            if (HAL_GPIO_ReadPin(GPIOA, colPins[col]) == GPIO_PIN_RESET)
            {
                /* Restore row to HIGH before returning */
                HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_SET);
                return keys[row][col];
            }
        }

        /* Restore row to HIGH */
        HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_SET);
    }

    return '\0';
}

char getKey(void)
{
    char key = scanKeypad();
    if (key == '\0')
    {
        return '\0';
    }

    HAL_Delay(25); /* Debounce delay */
    if (scanKeypad() != key)
    {
        return '\0';
    }

    /* Wait until key is released */
    while (scanKeypad() != '\0')
    {
        HAL_Delay(10);
    }

    return key;
}

/* ============================================================================
 * LCD Low-Level & High-Level Driver (PCF8574 I2C Backpack)
 * ============================================================================ */
void lcdSetBacklight(uint8_t state)
{
    g_lcd_backlight = state ? 1 : 0;
    lcdExpanderWrite(0);
}

void lcdSetProfile(LcdPinProfile_t profile)
{
    g_lcd_pin_profile = profile;
    g_lcd_diag.pin_profile = (uint8_t)profile;
}

HAL_StatusTypeDef lcdExpanderWrite(uint8_t value)
{
    uint8_t data = value;
    if (g_lcd_pin_profile == LCD_PROFILE_STANDARD)
    {
        data |= (g_lcd_backlight ? LCD_BL : 0x00);
    }
    else
    {
        data |= (g_lcd_backlight ? 0x80 : 0x00);
    }

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, lcd_i2c_addr, &data, 1, 25);
    if (status != HAL_OK)
    {
        g_lcd_diag.tx_fail_count++;
        g_lcd_diag.last_i2c_status = status;
        g_lcd_diag.last_i2c_error_code = hi2c1.ErrorCode;

        /* Attempt single bus recovery retry if line glitched */
        I2C_Bus_Recovery();
        status = HAL_I2C_Master_Transmit(&hi2c1, lcd_i2c_addr, &data, 1, 25);
        if (status == HAL_OK)
        {
            g_lcd_diag.tx_success_count++;
        }
    }
    else
    {
        g_lcd_diag.tx_success_count++;
    }
    return status;
}

HAL_StatusTypeDef lcdSendNibble(uint8_t nibble4, uint8_t rs)
{
    HAL_StatusTypeDef st;
    uint8_t val;

    if (g_lcd_pin_profile == LCD_PROFILE_STANDARD)
    {
        /* Standard/YwRobot: D4-D7 on P4-P7, RS on P0, EN on P2 */
        val = ((nibble4 & 0x0F) << 4) | (rs ? 0x01 : 0x00);
        st = lcdExpanderWrite(val);
        if (st != HAL_OK) return st;
        delay_us(5);
        st = lcdExpanderWrite(val | 0x04); /* EN = 1 */
        if (st != HAL_OK) return st;
        delay_us(10);
        st = lcdExpanderWrite(val & ~0x04); /* EN = 0 */
        if (st != HAL_OK) return st;
        delay_us(50);
    }
    else
    {
        /* MJKDZ: D4-D7 on P0-P3, RS on P4, EN on P6 */
        val = (nibble4 & 0x0F) | (rs ? 0x10 : 0x00);
        st = lcdExpanderWrite(val);
        if (st != HAL_OK) return st;
        delay_us(5);
        st = lcdExpanderWrite(val | 0x40); /* EN = 1 */
        if (st != HAL_OK) return st;
        delay_us(10);
        st = lcdExpanderWrite(val & ~0x40); /* EN = 0 */
        if (st != HAL_OK) return st;
        delay_us(50);
    }

    return st;
}

HAL_StatusTypeDef lcdSend(uint8_t value, uint8_t dataMode)
{
    HAL_StatusTypeDef st;
    /* High nibble first */
    st = lcdSendNibble((value >> 4) & 0x0F, dataMode);
    if (st != HAL_OK) return st;
    /* Low nibble second */
    st = lcdSendNibble(value & 0x0F, dataMode);
    return st;
}

HAL_StatusTypeDef lcdCommand(uint8_t command)
{
    return lcdSend(command, 0);
}

HAL_StatusTypeDef lcdPrintChar(char c)
{
    return lcdSend((uint8_t)c, 1);
}

HAL_StatusTypeDef lcdClear(void)
{
    HAL_StatusTypeDef st = lcdCommand(0x01);
    HAL_Delay(3);
    return st;
}

HAL_StatusTypeDef lcdSetCursor(uint8_t column, uint8_t row)
{
    const uint8_t rowOffsets[] = {0x00, 0x40};
    return lcdCommand(0x80 | (column + rowOffsets[row & 1]));
}

HAL_StatusTypeDef lcdPrint(const char *text)
{
    HAL_StatusTypeDef st = HAL_OK;
    while (*text)
    {
        st = lcdPrintChar(*text++);
        if (st != HAL_OK) return st;
    }
    return st;
}

HAL_StatusTypeDef lcdPrintMasked(uint8_t length)
{
    HAL_StatusTypeDef st = HAL_OK;
    for (uint8_t i = 0; i < length; i++)
    {
        st = lcdPrintChar('*');
        if (st != HAL_OK) return st;
    }
    return st;
}

uint8_t lcdInit(void)
{
    /* 1. Allow HD44780 controller & 5V power rail to stabilize */
    HAL_Delay(100);

    /* 2. Run I2C bus recovery in case slave held SDA low */
    uint8_t sda_stuck = I2C_Bus_Recovery();
    if (sda_stuck)
    {
        g_lcd_diag.status = LCD_STATUS_BUS_STUCK;
    }

    /* 3. Probe and detect PCF8574 address */
    uint8_t found = I2C_ScanForLCD();
    if (!found)
    {
        g_lcd_diag.status = LCD_STATUS_NO_DEVICE;
        return 0;
    }

    g_lcd_diag.found_addr_7bit = (uint8_t)(lcd_i2c_addr >> 1);
    g_lcd_diag.pin_profile = (uint8_t)g_lcd_pin_profile;

    /* 4. Visual diagnostic: toggle backlight */
    lcdSetBacklight(0);
    HAL_Delay(80);
    lcdSetBacklight(1);
    HAL_Delay(80);

    /* 5. HD44780 4-bit initialization sequence per Hitachi HD44780U datasheet */
    /* Step 1: Send 0x03 three times to force 8-bit mode */
    if (lcdSendNibble(0x03, 0) != HAL_OK) { g_lcd_diag.status = LCD_STATUS_WRITE_ERROR; return 0; }
    HAL_Delay(10);
    if (lcdSendNibble(0x03, 0) != HAL_OK) { g_lcd_diag.status = LCD_STATUS_WRITE_ERROR; return 0; }
    HAL_Delay(5);
    if (lcdSendNibble(0x03, 0) != HAL_OK) { g_lcd_diag.status = LCD_STATUS_WRITE_ERROR; return 0; }
    HAL_Delay(5);

    /* Step 2: Switch to 4-bit mode (send 0x02) */
    if (lcdSendNibble(0x02, 0) != HAL_OK) { g_lcd_diag.status = LCD_STATUS_WRITE_ERROR; return 0; }
    HAL_Delay(5);

    /* Step 3: Function Set: 4-bit mode, 2 lines, 5x8 font (0x28) */
    if (lcdCommand(0x28) != HAL_OK) { g_lcd_diag.status = LCD_STATUS_WRITE_ERROR; return 0; }
    HAL_Delay(2);

    /* Step 4: Display OFF (0x08) */
    if (lcdCommand(0x08) != HAL_OK) { g_lcd_diag.status = LCD_STATUS_WRITE_ERROR; return 0; }
    HAL_Delay(2);

    /* Step 5: Clear display (0x01) */
    if (lcdClear() != HAL_OK) { g_lcd_diag.status = LCD_STATUS_WRITE_ERROR; return 0; }
    HAL_Delay(5);

    /* Step 6: Entry mode set: increment cursor, no display shift (0x06) */
    if (lcdCommand(0x06) != HAL_OK) { g_lcd_diag.status = LCD_STATUS_WRITE_ERROR; return 0; }
    HAL_Delay(2);

    /* Step 7: Display ON, Cursor OFF, Blink OFF (0x0C) */
    if (lcdCommand(0x0C) != HAL_OK) { g_lcd_diag.status = LCD_STATUS_WRITE_ERROR; return 0; }
    HAL_Delay(2);

    g_lcd_diag.status = LCD_STATUS_OK;
    return 1;
}

void showScreen(const char *line1, const char *line2)
{
    lcdClear();
    lcdSetCursor(0, 0);
    lcdPrint(line1);
    lcdSetCursor(0, 1);
    lcdPrint(line2);
}

void showEntryScreen(const char *title, uint8_t length)
{
    lcdClear();
    lcdSetCursor(0, 0);
    lcdPrint(title);
    lcdSetCursor(0, 1);
    lcdPrintMasked(length);
}

/* ============================================================================
 * Non-Volatile Flash Storage
 * ============================================================================ */
static uint8_t Flash_LoadPassword(char *out_pass, uint32_t max_len)
{
    if (out_pass == NULL || max_len == 0)
    {
        return 0;
    }

    memset(out_pass, 0, max_len);

    const FlashConfig_t *record = (const FlashConfig_t *)FLASH_STORAGE_PAGE_ADDR;

    /* 1. Check Magic value: 0xFFFFFFFF means erased page */
    if (record->magic != FLASH_CONFIG_MAGIC)
    {
        return 0;
    }

    /* 2. Check Length: must be non-empty and fit within buffers */
    if (record->length == 0 || record->length > PASSWORD_MAX_LENGTH || record->length >= max_len)
    {
        return 0;
    }

    /* 3. Validate null termination within the storage structure */
    if (record->password[record->length] != '\0')
    {
        return 0;
    }

    /* 4. Validate character data: characters must be printable ASCII */
    for (uint16_t i = 0; i < record->length; i++)
    {
        char c = record->password[i];
        if (c < 0x20 || c > 0x7E)
        {
            return 0; /* corrupted or unprogrammed data */
        }
    }

    /* 5. Safe copy */
    memcpy(out_pass, record->password, record->length);
    out_pass[record->length] = '\0';

    return 1;
}

static HAL_StatusTypeDef Flash_SavePassword(const char *new_pass)
{
    if (new_pass == NULL)
    {
        return HAL_ERROR;
    }

    uint32_t pass_len = strlen(new_pass);
    if (pass_len == 0 || pass_len > PASSWORD_MAX_LENGTH)
    {
        return HAL_ERROR;
    }

    /* Check if current stored record is already identical to avoid unnecessary flash wear */
    char current_stored[PASSWORD_MAX_LENGTH + 1] = {0};
    if (Flash_LoadPassword(current_stored, sizeof(current_stored)) &&
        strcmp(current_stored, new_pass) == 0)
    {
        return HAL_OK;
    }

    /* Prepare record buffer aligned to 16-bit halfwords */
    FlashConfig_t record;
    memset(&record, 0, sizeof(record));
    record.magic = FLASH_CONFIG_MAGIC;
    record.length = (uint16_t)pass_len;
    strncpy(record.password, new_pass, PASSWORD_MAX_LENGTH);
    record.password[PASSWORD_MAX_LENGTH] = '\0';

    uint32_t halfwordCount = (sizeof(FlashConfig_t) + 1) / 2;
    uint16_t buffer[(sizeof(FlashConfig_t) + 1) / 2];
    memset(buffer, 0xFF, sizeof(buffer));
    memcpy(buffer, &record, sizeof(FlashConfig_t));

    /* Unlock Flash */
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Clear any pending error flags before operation */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

    /* Erase the single reserved page */
    FLASH_EraseInitTypeDef eraseInitStruct = {0};
    uint32_t pageError = 0;

    eraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = FLASH_STORAGE_PAGE_ADDR;
    eraseInitStruct.NbPages     = 1;

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
    if (status != HAL_OK || pageError != 0xFFFFFFFFU)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    /* Program halfwords (16-bit) sequentially */
    for (uint32_t i = 0; i < halfwordCount; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                   FLASH_STORAGE_PAGE_ADDR + (i * 2),
                                   buffer[i]);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return status;
        }
    }

    /* Lock Flash interface */
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Read-back verification to guarantee data is persistently written */
    const uint16_t *flash_ptr = (const uint16_t *)FLASH_STORAGE_PAGE_ADDR;
    for (uint32_t i = 0; i < halfwordCount; i++)
    {
        if (flash_ptr[i] != buffer[i])
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

/* ============================================================================
 * Protected Peripheral Control
 * ============================================================================ */
static void ProtectedPeripheral_Activate(void)
{
    /* Activate door lock relay/solenoid or system peripheral functions */
}

static void ProtectedPeripheral_Deactivate(void)
{
    /* Deactivate door lock relay/solenoid or system peripheral functions */
}

/* ============================================================================
 * Password Authentication Logic
 * ============================================================================ */

void Password_Start(void)
{
    /* Ensure outputs are OFF at startup */
    HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    ProtectedPeripheral_Deactivate();

    entered_length = 0;
    memset(entered_password, 0, sizeof(entered_password));
    memset(setup_temp_password, 0, sizeof(setup_temp_password));
    memset(saved_password, 0, sizeof(saved_password));

    if (Flash_LoadPassword(saved_password, sizeof(saved_password)))
    {
        current_state = STATE_ENTER_PASSWORD;
        showEntryScreen("ENTER PASSWORD", 0);
    }
    else
    {
        current_state = STATE_SETUP_PASSWORD;
        showEntryScreen("SET PASSWORD", 0);
    }
}

uint8_t Password_Check(const char *entered)
{
    if (strlen(saved_password) == 0)
    {
        return 0;
    }
    return (strcmp(entered, saved_password) == 0) ? 1 : 0;
}

void Password_GrantAccess(void)
{
    current_state = STATE_AUTHENTICATED;

    showScreen("ACCESS GRANTED", "SYSTEM ACTIVE");

    /* PB0 HIGH, PB1 LOW, Buzzer OFF */
    HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);

    ProtectedPeripheral_Activate();
}

void Password_DenyAccess(void)
{
    showScreen("ACCESS DENIED", "WRONG PASSWORD");

    /* PB1 HIGH, PB0 LOW, Buzzer ON */
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);

    /* Wait approximately 2 seconds (isolated from keypad scanning) */
    HAL_Delay(2000);

    /* Turn Red LED and Buzzer OFF */
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);

    entered_length = 0;
    memset(entered_password, 0, sizeof(entered_password));

    current_state = STATE_ENTER_PASSWORD;
    showEntryScreen("ENTER PASSWORD", 0);
}

void Password_Logout(void)
{
    /* Clear authentication state */
    current_state = STATE_ENTER_PASSWORD;

    /* PB0 OFF, PB1 OFF, Buzzer OFF */
    HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);

    ProtectedPeripheral_Deactivate();

    /* Clear password entry buffers */
    entered_length = 0;
    memset(entered_password, 0, sizeof(entered_password));

    /* Return to ENTER PASSWORD screen */
    showEntryScreen("ENTER PASSWORD", 0);
}

void Password_Setup(void)
{
    if (entered_length == 0)
    {
        return;
    }

    /* Temporarily store the first password */
    memcpy(setup_temp_password, entered_password, sizeof(setup_temp_password));

    /* Clear entry buffer */
    entered_length = 0;
    memset(entered_password, 0, sizeof(entered_password));

    current_state = STATE_CONFIRM_PASSWORD;
    showEntryScreen("CONFIRM PASSWORD", 0);
}

void Password_ConfirmSetup(void)
{
    if (entered_length == 0)
    {
        return;
    }

    if (strcmp(entered_password, setup_temp_password) == 0)
    {
        /* Save configured password to Flash and verify status */
        if (Flash_SavePassword(entered_password) == HAL_OK)
        {
            memcpy(saved_password, entered_password, sizeof(saved_password));

            /* Clear temporary and entry buffers */
            entered_length = 0;
            memset(entered_password, 0, sizeof(entered_password));
            memset(setup_temp_password, 0, sizeof(setup_temp_password));

            showScreen("PASSWORD SET", "SUCCESSFUL");
            HAL_Delay(2000);

            /* Automatically proceed to normal password-entry screen */
            current_state = STATE_ENTER_PASSWORD;
            showEntryScreen("ENTER PASSWORD", 0);
        }
        else
        {
            /* Flash persistence failed: show error and remain in setup mode */
            showScreen("STORAGE ERROR", "SAVE FAILED");
            HAL_Delay(2000);

            /* Clear buffers and return to password setup screen */
            entered_length = 0;
            memset(entered_password, 0, sizeof(entered_password));
            memset(setup_temp_password, 0, sizeof(setup_temp_password));

            current_state = STATE_SETUP_PASSWORD;
            showEntryScreen("SET PASSWORD", 0);
        }
    }
    else
    {
        showScreen("PASSWORD MISMATCH", "TRY AGAIN");
        HAL_Delay(2000);

        /* Clear both buffers and return to password setup screen */
        entered_length = 0;
        memset(entered_password, 0, sizeof(entered_password));
        memset(setup_temp_password, 0, sizeof(setup_temp_password));

        current_state = STATE_SETUP_PASSWORD;
        showEntryScreen("SET PASSWORD", 0);
    }
}

void Password_ProcessKey(char key)
{
    /* If already authenticated, check for logout */
    if (current_state == STATE_AUTHENTICATED)
    {
        if (key == KEY_LOGOUT || key == KEY_ALT_LOGOUT)
        {
            Password_Logout();
        }
        return;
    }

    /* Clear key ('*') clears current entry buffer */
    if (key == KEY_CLEAR)
    {
        entered_length = 0;
        memset(entered_password, 0, sizeof(entered_password));

        if (current_state == STATE_SETUP_PASSWORD)
        {
            showEntryScreen("SET PASSWORD", 0);
        }
        else if (current_state == STATE_CONFIRM_PASSWORD)
        {
            showEntryScreen("CONFIRM PASSWORD", 0);
        }
        else if (current_state == STATE_ENTER_PASSWORD)
        {
            showEntryScreen("ENTER PASSWORD", 0);
        }
        return;
    }

    /* Confirm / Enter key ('#') */
    if (key == KEY_CONFIRM)
    {
        if (entered_length == 0)
        {
            return; /* Require non-empty password */
        }

        if (current_state == STATE_SETUP_PASSWORD)
        {
            Password_Setup();
        }
        else if (current_state == STATE_CONFIRM_PASSWORD)
        {
            Password_ConfirmSetup();
        }
        else if (current_state == STATE_ENTER_PASSWORD)
        {
            if (Password_Check(entered_password))
            {
                Password_GrantAccess();
            }
            else
            {
                Password_DenyAccess();
            }

            /* Always clear entered buffer after authentication attempt */
            entered_length = 0;
            memset(entered_password, 0, sizeof(entered_password));
        }
        return;
    }

    /* Numeric / Character input: Prevent buffer overflow */
    if (entered_length < PASSWORD_MAX_LENGTH)
    {
        entered_password[entered_length++] = key;
        entered_password[entered_length] = '\0';

        if (current_state == STATE_SETUP_PASSWORD)
        {
            showEntryScreen("SET PASSWORD", entered_length);
        }
        else if (current_state == STATE_CONFIRM_PASSWORD)
        {
            showEntryScreen("CONFIRM PASSWORD", entered_length);
        }
        else if (current_state == STATE_ENTER_PASSWORD)
 
 {
            showEntryScreen("ENTER PASSWORD", entered_length);
        }
    }
}