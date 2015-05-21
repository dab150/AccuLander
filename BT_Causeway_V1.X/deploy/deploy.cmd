setlocal enabledelayedexpansion

set major=1
set minor=24
set buildnum=
set filename=BT_Causeway


for /f "delims=;" %%i in (.\deploy\build_number.txt) do set buildnum=%%i

mkdir ..\..\hex\
copy dist\default\production\%filename%.X.production.hex ..\..\hex\builds\%filename%___build.%buildnum%.hex

@echo.
@echo.
@echo.
@echo ===============================================================
@echo Created new firmware version: %filename%____build.%buildnum%.hex
@echo ===============================================================

set /a buildnum=%buildnum% + 1

@echo %buildnum%;> .\deploy\build_number.txt