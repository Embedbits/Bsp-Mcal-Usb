#======================== Unit tests configuration ============================#
# Unit tests of Usb module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).
#
# The test set is independent - it lists everything it needs: code under test,
# mocked headers, host emulation of the target and libraries built for host.

get_filename_component(USB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${USB_DIR}/.." ABSOLUTE)

# STM32H5E4 / H5E5 / H5F4 / H5F5 have the USB OTG controller (not supported yet - the module returns errors)
if(TARGET_MCU_FULL_NAME MATCHES "^STM32H5[EF]")
    set(USB_TEST_SOURCE Test_UsbUnsupported.c)
else()
    set(USB_TEST_SOURCE Test_Usb.c)
endif()

UnitTesting_Add_Test(
    NAME            Usb
    TEST_SOURCE     ${USB_TEST_SOURCE}
    UUT_SOURCES     ${USB_DIR}/Usb.c
                    ${USB_DIR}/Usb_Ep.c
                    ${USB_DIR}/Usb_Pma.c
                    ${USB_DIR}/Usb_Xfer.c
                    ${USB_DIR}/Usb_Host.c
                    ${USB_DIR}/Usb_Unsupported.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Gpio/Gpio_Port.h
    INCLUDE_DIRS    ${USB_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Gpio
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
)
