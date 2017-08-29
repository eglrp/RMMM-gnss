::
:: Arguments:
:: 1: Ozone exe
:: 2: Ozone project
::

@echo off
set PATH=c:\windows\system32
tasklist.exe | find "Ozone.exe" >NUL
if errorlevel 1 goto start_ozone
echo "Ozone already running!"
exit /b 1

:start_ozone
%1 %2
