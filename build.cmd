@echo off
setlocal
set "ROOT=%~dp0"
set "SOURCE=%ROOT%music_controller"
set "BUILD=%SOURCE%\build"
set "TOOLCHAIN=C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
set "CC=%TOOLCHAIN%\arm-none-eabi-gcc.exe"
set "OBJCOPY=%TOOLCHAIN%\arm-none-eabi-objcopy.exe"
set "SIZE=%TOOLCHAIN%\arm-none-eabi-size.exe"

if not exist "%CC%" (
    echo STM32 GNU toolchain not found.
    exit /b 1
)

if not exist "%BUILD%" mkdir "%BUILD%"
set "CPU=-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard"
set "FLAGS=%CPU% -DSTM32H743xx -O2 -g -Wall -Wextra -ffunction-sections -fdata-sections -I%ROOT%Drivers\CMSIS\Device\ST\STM32H7xx\Include -I%ROOT%Drivers\CMSIS\Include -I%SOURCE%\ThirdParty\minimp3"

"%CC%" %FLAGS% -c "%SOURCE%\Src\main.c" -o "%BUILD%\main.o" || exit /b 1
"%CC%" %FLAGS% -c "%SOURCE%\Src\sdram.c" -o "%BUILD%\sdram.o" || exit /b 1
"%CC%" %FLAGS% -c "%SOURCE%\Src\wm8978_playback.c" -o "%BUILD%\wm8978_playback.o" || exit /b 1
"%CC%" %FLAGS% -c "%SOURCE%\Src\minimp3_impl.c" -o "%BUILD%\minimp3_impl.o" || exit /b 1
pushd "%SOURCE%" || exit /b 1
"%OBJCOPY%" -I binary -O elf32-littlearm -B arm --rename-section .data=.rodata.cjk,alloc,load,readonly,data,contents cjk16.bin build\cjk16.o
set "RESULT=%ERRORLEVEL%"
popd
if not "%RESULT%"=="0" exit /b %RESULT%
"%CC%" %FLAGS% -c "%ROOT%Drivers\CMSIS\Device\ST\STM32H7xx\Source\Templates\system_stm32h7xx.c" -o "%BUILD%\system.o" || exit /b 1
"%CC%" %FLAGS% -c "%ROOT%Drivers\CMSIS\Device\ST\STM32H7xx\Source\Templates\gcc\startup_stm32h743xx.s" -o "%BUILD%\startup.o" || exit /b 1
"%CC%" %CPU% -T"%ROOT%STM32H743IITX_FLASH.ld" -Wl,-gc-sections --specs=nosys.specs "%BUILD%\main.o" "%BUILD%\sdram.o" "%BUILD%\wm8978_playback.o" "%BUILD%\minimp3_impl.o" "%BUILD%\cjk16.o" "%BUILD%\system.o" "%BUILD%\startup.o" -o "%BUILD%\music_controller.elf" || exit /b 1
"%SIZE%" "%BUILD%\music_controller.elf" || exit /b 1
"%OBJCOPY%" -O ihex "%BUILD%\music_controller.elf" "%BUILD%\music_controller.hex" || exit /b 1
"%OBJCOPY%" -O binary "%BUILD%\music_controller.elf" "%BUILD%\music_controller.bin" || exit /b 1

echo.
echo Firmware HEX: %BUILD%\music_controller.hex
echo Firmware BIN: %BUILD%\music_controller.bin
exit /b 0
