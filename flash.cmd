@echo off
setlocal
set "ROOT=%~dp0"
set "HEX=%ROOT%music_controller\build\music_controller.hex"
set "NO_BUILD=0"

if /I "%~1"=="--no-build" set "NO_BUILD=1"
if not "%~1"=="" if /I not "%~1"=="--no-build" (
    echo Usage: flash.cmd [--no-build]
    exit /b 2
)

if "%NO_BUILD%"=="0" (
    call "%ROOT%build.cmd" || exit /b 1
) else if not exist "%HEX%" (
    echo Firmware not found: %HEX%
    echo Run build.cmd first.
    exit /b 1
)

if defined STM32_PROGRAMMER_CLI (
    set "PROGRAMMER=%STM32_PROGRAMMER_CLI%"
) else if exist "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" (
    set "PROGRAMMER=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
) else if exist "C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe" (
    set "PROGRAMMER=C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
) else (
    echo STM32_Programmer_CLI.exe not found.
    echo Set STM32_PROGRAMMER_CLI to its full path and try again.
    exit /b 1
)

echo.
echo Flashing %HEX%
echo Attempt 1: connect under hardware reset at 950 kHz...
"%PROGRAMMER%" -c port=SWD mode=UR reset=HWrst freq=950 -w "%HEX%" -v -rst
if not errorlevel 1 goto :success

echo.
echo Attempt 1 failed. Retrying with Hot Plug at 400 kHz...
"%PROGRAMMER%" -c port=SWD mode=Hotplug freq=400 -w "%HEX%" -v
if not errorlevel 1 goto :hotplug_success

echo.
echo Flash failed with both connection modes.
echo Check SWDIO, SWCLK, GND, 3.3V reference and NRST.
echo Temporarily disable ESP8266 by pulling EN low, then retry:
echo   flash.cmd --no-build
exit /b 1

:hotplug_success
echo.
echo Programming and verification succeeded in Hot Plug mode.
echo Press the STM32 RESET button once to run the new firmware.
exit /b 0

:success
echo.
echo Flash and verification completed successfully.
exit /b 0
