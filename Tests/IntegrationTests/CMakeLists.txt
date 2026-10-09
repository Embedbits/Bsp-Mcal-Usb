#===================== Integration tests configuration ========================#
# Integration tests of Usb module on target. Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_Usb.c and
# all libraries of the firmware are listed here.

# StartUp calls BspMain() - the entry point of the test firmware is provided by
# the test set (BspMain.h, ItTarget_Usb.c)
if(NOT TARGET BspMain_Lib)
    add_library(BspMain_Lib INTERFACE)
    target_include_directories(BspMain_Lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

IntegrationTesting_Add_Test(
    NAME            Usb
    TEST_SOURCE     ItTest_Usb.c
    SOURCES         ItTarget_Usb.c    # Target interface of the test set
    LINK_LIBS       StartUp_Lib                 # Startup code (calls BspMain)
                    Nvic_Lib                    # Fault handlers, system reset (ItTarget)
                    Rcc_Lib                     # Kernel clock check
                    Tim_Lib                     # Reference time base (TIM2)
                    Usb_Lib                     # Module under test
    TIMEOUT         60
)

# Host mode: needs a full speed device (flash drive, hub) on the user USB connector with VBUS (device test
# is ignored without it), see the description in ItTest_UsbHost.c
IntegrationTesting_Add_Test(
    NAME            UsbHost
    TEST_SOURCE     ItTest_UsbHost.c
    SOURCES         ItTarget_Usb.c    # Target interface of the test set
    LINK_LIBS       StartUp_Lib                 # Startup code (calls BspMain)
                    Nvic_Lib                    # Fault handlers, system reset (ItTarget)
                    Rcc_Lib                     # Kernel clock check
                    Tim_Lib                     # Reference time base (TIM2)
                    Gpio_Lib                    # VBUS switch of the board
                    Usb_Lib                     # Module under test
    TIMEOUT         60
)
