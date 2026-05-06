@echo off
setlocal

pushd "%~dp0"

if not exist "vendor\bin\premake5.exe" (
    echo [Setup] ERROR: vendor\bin\premake5.exe was not found.
    echo [Setup] Please ensure premake5.exe is committed under vendor\bin\.
    popd
    exit /b 1
)

set ACTION=%1
if "%ACTION%"=="" set ACTION=vs2022

echo [Setup] Generating project files with action: %ACTION%
"vendor\bin\premake5.exe" %ACTION%
set EXITCODE=%ERRORLEVEL%

if %EXITCODE% neq 0 (
    echo [Setup] Premake failed with exit code %EXITCODE%.
    popd
    exit /b %EXITCODE%
)

echo [Setup] Done. Open Axiom-Physics.sln in Visual Studio.
popd
endlocal
