PROJECT_NAME     := RJ100
TARGETS          := nrf51422_xxac
OUTPUT_DIRECTORY := _build


$(OUTPUT_DIRECTORY)/nrf51422_xxac.out: \
LINKER_SCRIPT  := ble_app_hrs_gcc_nrf51.ld

# Source files common to all targets
SRC_FILES += \
  MDK/nRF_Libraries/Src/nrf_log_backend_serial.c \
  MDK/nRF_Libraries/Src/nrf_log_frontend.c \
  MDK/nRF_Libraries/Src/app_error.c \
  MDK/components/libraries/button/app_button.c \
  MDK/nRF_Libraries/Src/app_error_weak.c \
  MDK/nRF_Libraries/Src/app_timer.c \
  MDK/nRF_Libraries/Src/app_timer_appsh.c \
  MDK/nRF_Libraries/Src/app_util_platform.c \
  MDK/nRF_Libraries/Src/crc16.c \
  MDK/nRF_Libraries/Src/hardfault_implementation.c \
  MDK/nRF_Libraries/Src/nrf_assert.c \
  MDK/nRF_Libraries/Src/sdk_errors.c \
  MDK/nRF_Libraries/Src/sdk_mapped_flags.c \
  MDK/Board_Support/Src/boards.c \
  MDK/nRF_BLE_Services/Src/ble_cts_c.c \
  Drivers/HAL/Src/hal_drv_clock.c \
  Drivers/HAL/Src/hal_drv_common.c \
  Drivers/HAL/Src/hal_drv_adc.c \
  Drivers/HAL/Src/hal_adc.c \
  Drivers/HAL/Src/hal_drv_gpiote.c \
  Drivers/HAL/Src/hal_drv_uart.c \
  Drivers/HAL/Src/hal_drv_rtc.c \
  Drivers/HAL/Src/hal_drv_twi.c\
  Application/Src/main.c \
  Drivers/CPT/Src/cpt_adc.c \
  Drivers/CPT/Src/cpt_lis2hh12.c \
  Drivers/CPT/Src/cpt_max30110.c \
  Drivers/CPT/Src/cpt_uart.c \
  Drivers/CPT/Src/cpt_rtc.c \
  Drivers/BSP/Src/bsp_accl.c \
  Drivers/BSP/Src/bsp_adc.c \
  Drivers/BSP/Src/bsp_afe.c \
  Application/Src/analog_interface.c \
  Application/Src/ble_interface.c \
  Application/Src/hrm_interface.c \
  Application/Src/rtc_interface.c \
  MDK/nRF_BLE/Src/ble_advdata.c \
  MDK/nRF_BLE/Src/ble_advertising.c \
  MDK/nRF_BLE/Src/ble_conn_params.c \
  MDK/nRF_BLE/Src/ble_conn_state.c \
  MDK/nRF_BLE/Src/ble_srv_common.c \
  MDK/nRF_BLE/Src/gatt_cache_manager.c \
  MDK/nRF_BLE/Src/gatts_cache_manager.c \
  MDK/nRF_BLE/Src/id_manager.c \
  MDK/nRF_BLE/Src/nrf_ble_gatt.c \
  MDK/nRF_BLE/Src/peer_data.c \
  MDK/nRF_BLE/Src/peer_data_storage.c \
  MDK/nRF_BLE/Src/ble_db_discovery.c \
  MDK/nRF_BLE/Src/peer_database.c \
  MDK/nRF_BLE/Src/peer_id.c \
  MDK/nRF_BLE/Src/peer_manager.c \
  MDK/nRF_BLE/Src/pm_buffer.c \
  MDK/nRF_BLE/Src/pm_mutex.c \
  MDK/nRF_BLE/Src/security_dispatcher.c \
  MDK/nRF_BLE/Src/security_manager.c \
  MDK/startup/gcc_startup_nrf51.S \
  MDK/startup/system_nrf51.c \
  MDK/components/libraries/scheduler/app_scheduler.c \
  MDK/nRF_BLE_Services/Src/ble_bas.c \
  MDK/nRF_BLE_Services/Src/ble_dis.c \
  MDK/nRF_BLE_Services/Src/ble_hrs.c \
  MDK/components/libraries/fds/fds.c \
  MDK/components/libraries/bsp/bsp.c \
  MDK/components/libraries/bsp/bsp_btn_ble.c \
  MDK/components/libraries/fstorage/fstorage.c \
  MDK/nRF_softdevices/common/softdevice_handler/softdevice_handler.c \
  MDK/nRF_softdevices/common/softdevice_handler/softdevice_handler_appsh.c \
  segger_rtt/SEGGER_RTT.c \
  segger_rtt/SEGGER_RTT_printf.c \
  segger_rtt/RTT_Syscalls_GCC.c \

# Include folders common to all targets
INC_FOLDERS += \
  MDK/components/ble/ble_services/ble_ancs_c \
  MDK/components/ble/ble_services/ble_ias_c \
  MDK/Board_Support/Inc \
  MDK/components/libraries/usbd/class/cdc/acm \
  MDK/components/libraries/usbd/class/hid/generic \
  MDK/components/libraries/usbd/class/msc \
  MDK/components/libraries/usbd/class/hid \
  MDK/components/libraries/scheduler \
  MDK/components/libraries/log \
  MDK/components/ble/ble_services/ble_gls \
  MDK/components/libraries/fstorage \
  MDK/components/libraries/gpiote \
  MDK/components/boards \
  MDK/components/drivers_nrf/common \
  MDK/components/ble/ble_advertising \
  MDK/components/ble/ble_services/ble_bas_c \
  MDK/components/ble/ble_services/ble_hrs_c \
  MDK/components/libraries/queue \
  MDK/components/ble/ble_dtm \
  toolchain/cmsis/include \
  MDK/components/ble/ble_services/ble_rscs_c \
  MDK/components/ble/common \
  MDK/components/ble/ble_services/ble_lls \
  MDK/components/libraries/bsp \
  MDK/components/ble/ble_services/ble_bas \
  MDK/components/libraries/experimental_section_vars \
  MDK/components/ble/ble_services/ble_ans_c \
  MDK/components/libraries/slip \
  MDK/components/libraries/mem_manager \
  MDK/components/libraries/usbd/class/cdc \
  MDK/components/drivers_nrf/hal \
  MDK/components/ble/ble_services/ble_nus_c \
  MDK/components/ble/ble_services/ble_ias \
  MDK/components/libraries/usbd/class/hid/mouse \
  MDK/components/drivers_nrf/ppi \
  MDK/components/ble/ble_services/ble_dfu \
  MDK/components/drivers_nrf/twis_slave \
  MDK/components \
  MDK/components/libraries/scheduler \
  MDK/components/ble/ble_services/ble_lbs \
  MDK/components/ble/ble_services/ble_hts \
  MDK/components/drivers_nrf/delay \
  MDK/components/libraries/crc16 \
  MDK/components/drivers_nrf/timer \
  MDK/components/libraries/util \
  MDK/Device \
  MDK/nRF_Libraries/Inc \
  MDK/nRF_softdevices/common \
  MDK/nRF_softdevices/s130/headers \
  MDK/nRF_softdevices/s130/headers/nrf51 \
  Drivers/HAL/Inc \
  MDK/nRF_BLE/Inc \
  segger_rtt \
  config \
  Drivers/CPT/Inc \
  Drivers/BSP/Inc \
  Application/Inc \
  MDK/components/libraries/low_power_pwm \
  MDK/components/libraries/hardfault \
  MDK/components/ble/ble_services/ble_cscs \
  MDK/components/libraries/uart \
  MDK/components/libraries/hci \
  MDK/components/libraries/usbd/class/hid/kbd \
  MDK/components/drivers_nrf/spi_slave \
  MDK/components/drivers_nrf/lpcomp \
  MDK/components/libraries/timer \
  MDK/components/drivers_nrf/power \
  MDK/components/libraries/usbd/config \
  toolchain \
  MDK/components/ble/ble_services/ble_rscs \
  MDK/components/ble/ble_services/ble_lbs_c \
  MDK/components/ble/ble_services/ble_tps \
  MDK/components/ble/ble_services/ble_nus \
  MDK/components/ble/ble_services/ble_hids \
  MDK/components/ble/ble_services/ble_cscs \
  MDK/components/ble/ble_services/ble_lbs \
  MDK/components/ble/ble_services/ble_hts \
  MDK/components/ble/ble_services/ble_dfu \
  MDK/components/ble/ble_services/ble_ias \
  MDK/components/ble/ble_services/ble_nus_c \
  MDK/components/ble/ble_services/ble_ans_c \
  MDK/components/ble/ble_services/ble_lls \
  MDK/components/ble/ble_services/ble_rscs_c \
  MDK/nRF_BLE_Services/Inc \
  MDK/components/libraries/led_softblink \
  MDK/components/ble/ble_services/ble_nus \
  MDK/components/ble/ble_services/ble_hids \
  MDK/components/libraries/crc32 \
  MDK/components/libraries/usbd/class/audio \
  MDK/components/ble/peer_manager \
  MDK/components/ble/ble_services/ble_tps \
  MDK/components/ble/ble_services/ble_dis \
  MDK/components/device \
  MDK/components/ble/nrf_ble_gatt \
  MDK/components/ble/nrf_ble_qwr \
  MDK/components/libraries/button \
  MDK/components/libraries/usbd \
  MDK/components/libraries/fds \
  MDK/components/libraries/fstorage \
  MDK/components/ble/ble_services/ble_lbs_c \
  MDK/components/ble/ble_racp \
  toolchain/gcc \
  MDK/components/libraries/twi \
  MDK/components/ble/ble_services/ble_rscs \
  MDK/components/drivers_nrf/usbd \
  MDK/nRF_softdevices/common/softdevice_handler \
  MDK/components/ble/ble_services/ble_hrs \
  MDK/components/libraries/log/src \

# Libraries common to all targets
LIB_FILES += \

# C flags common to all targets
CFLAGS += -DBOARD_RJ100
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DSWI_DISABLE0
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DNRF51422
CFLAGS += -DNRF_SD_BLE_API_VERSION=2
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs

CFLAGS += -mfloat-abi=soft
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 

# C++ flags common to all targets
CXXFLAGS += \

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_RJ100
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -D__HEAP_SIZE=0
ASMFLAGS += -DNRF51
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DNRF51422
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=2

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys


.PHONY: $(TARGETS) default all clean help flash flash_softdevice

# Default target - first one defined
default: nrf51422_xxac

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac

TEMPLATE_PATH := toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf51422_xxac.hex
	@echo Flashing: $<
	nrfjprog --program $< -f nrf51 --sectorerase
	nrfjprog --reset -f nrf51

# Flash softdevice
flash_softdevice:
	@echo Flashing: s130_nrf51_2.0.1_softdevice.hex
	nrfjprog --program MDK/nRF_softdevices/s130/hex/s130_nrf51_2.0.1_softdevice.hex -f nrf51 --sectorerase 
	nrfjprog --reset -f nrf51

erase:
	nrfjprog --eraseall -f nrf52
