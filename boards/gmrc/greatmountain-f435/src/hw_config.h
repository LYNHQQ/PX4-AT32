/*
 * hw_config.h
 *
 * Bootloader configuration for the GMRC GREATMOUNTAINF435 (AT32F435VM).
 *
 *   Constant                example                              Usage
 *   APP_LOAD_ADDRESS        0x08010000                           - The address in the
 *                                                                  linker script where the
 *                                                                  app firmware is org-ed
 *   BOOTLOADER_DELAY        5000                                 - Ms to wait while under
 *                                                                  bootloader request
 *   INTERFACE_USB           1                                    - (Optional) Scan and use
 *                                                                  the USB interface
 *   INTERFACE_USART         0                                    - (Optional) Scan and use
 *                                                                  the serial interface
 *   USBDEVICESTRING         "PX4 BL GMRC GREATMOUNTAINF435"      - USB id string
 *   USBPRODUCTID            0x0035                               - PID, matches defconfig
 *   BOARD_TYPE              0                                    - Must match .prototype board_id
 *   _FLASH_KBYTES           getreg16(AT32_SYSMEM_FSIZE)            - Run time flash size detection
 *   BOARD_FLASH_SECTORS     1005                                 - Last page erased during an
 *                                                                  upgrade (zero based)
 *   BOARD_FLASH_SIZE        (_FLASH_KBYTES * 1024)               - Total flash of device; the
 *                                                                  bootloader and app
 *                                                                  reservation are deducted
 *                                                                  programmatically
 *   BOARD_FIRST_FLASH_SECTOR_TO_ERASE 16                         - First page erased. Pages
 *                                                                  below it (the bootloader
 *                                                                  reservation) are never
 *                                                                  erased during an upgrade
 *   APP_RESERVATION_SIZE    8192                                 - Number of bytes reserved by
 *                                                                  the app firmware (the two
 *                                                                  parameter pages). This plus
 *                                                                  BOOTLOADER_RESERVATION_SIZE is
 *                                                                  deducted from BOARD_FLASH_SIZE
 *                                                                  to give the app firmware size
 *   USBMFGSTRING            "GMRC"                               - Optional USB MFG string
 *                                                                  (default is '3D Robotics')
 *
 * The board has no USB VBUS sense input: PA9 is UART1 TX, so USB VBUS sensing
 * and the OTG ID pin are disabled and cannot be used to select the bootloader.
 * The redundant parameter register signature (see board_configure_reset()) and
 * an unbootable application are the two ways into the bootloader. The product
 * string therefore carries the "PX4 BL " prefix so hosts can tell the two apart.
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

#include <nuttx/config.h>
#include <at32_gpio.h>
#include <arm_internal.h>
#include <hardware/at32f43xxx_memorymap.h>

/* Boot device selection list*/
#define USB0_DEV       0x01
#define SERIAL0_DEV    0x02
#define SERIAL1_DEV    0x04

#define APP_LOAD_ADDRESS               0x08010000
#define BOOTLOADER_DELAY               5000
#define INTERFACE_USB                  1
#define INTERFACE_USB_CONFIG           "/dev/ttyACM0"
#define BOARD_USB_VBUS_SENSE_DISABLED

/* No dedicated serial bootloader port exists on this board: UART1 is the RC
 * input and UART2/3/4/8 carry telemetry, GNSS and the console. USB only.
 */
#define INTERFACE_USART                0

#define BOARD_TYPE                     0
#define _FLASH_KBYTES                  getreg16(AT32_SYSMEM_FSIZE)
#define BOARD_FLASH_SECTORS            (1005)
#define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

/* LEDs are active low; PD15 and PD14, see board_config.h. */
#define BOARD_PIN_LED_ACTIVITY         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_DRV_STRONG|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN15)
#define BOARD_PIN_LED_BOOTLOADER       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_DRV_STRONG|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN14)
#define BOARD_LED_ON                   0
#define BOARD_LED_OFF                  1

/*
 * Uncommenting this allows to force the bootloader through a GPIO. As this can
 * accidentally initialize an ESC prematurely, this feature defaults to off.
 *
 * # define BOARD_FORCE_BL_PIN           (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN6)
 * # define BOARD_FORCE_BL_STATE         0
 */

#if !defined(ARCH_SN_MAX_LENGTH)
# define ARCH_SN_MAX_LENGTH 12
#endif

#if !defined(APP_RESERVATION_SIZE)
#  define APP_RESERVATION_SIZE 8192
#endif

#if !defined(BOARD_FIRST_FLASH_SECTOR_TO_ERASE)
#  define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 16
#endif

#if !defined(USB_DATA_ALIGN)
# define USB_DATA_ALIGN
#endif

#ifndef BOOT_DEVICES_SELECTION
#  define BOOT_DEVICES_SELECTION USB0_DEV
#endif

#ifndef BOOT_DEVICES_FILTER_ONUSB
#  define BOOT_DEVICES_FILTER_ONUSB USB0_DEV
#endif

#endif /* HW_CONFIG_H_ */
