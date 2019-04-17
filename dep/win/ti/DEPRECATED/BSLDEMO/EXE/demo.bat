@echo off
rem ---- initialize environment variables -----------------------------

setlocal
set workdir=%CD%

set HANDLER="%workdir%\BSLDEMO2.exe"
set BSL="%workdir%\bsl_150.txt"
set BSL130="%workdir%\BL_130V.txt"
set BSL150="%workdir%\BL_150S_14x.txt"
set TXT="%workdir%\test.txt"
set ComPort=COM70  
goto test1 
 

:test1
echo --- test F413 (V1.30), F123 (V1.40), both 1 mass erase cycle ----
echo.
%HANDLER% -c%ComPort% -m1 +epvw  %TXT%
goto end


:test2
%HANDLER% -c%ComPort% +pvrw     %TXT%
%HANDLER% -c%ComPort% -w        %TXT%
%HANDLER% -c%ComPort% -s2 -w    %TXT%
%HANDLER% -c%ComPort% -w +vr  -pint_vect.txt %TXT%
goto end


rem ---- clear environment variables ---------------------------------
:end
pause
set HANDLER=
set BSL=
set BSL130=
set BSL150=
set TXT=
set ComPort=


