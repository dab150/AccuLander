setlocal enabledelayedexpansion

set buildnum=


for /f "delims=;" %%i in (.\deploy\build_number.txt) do set buildnum=%%i


@echo.
@echo.
@echo.
@echo ===========================
@echo Compiling build #%buildnum%
@echo ===========================


@echo #define __BUILD %buildnum%> .\build.h