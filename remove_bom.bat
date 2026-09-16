@echo off
REM 批量移除 code 目录下所有 .c/.h 文件的 UTF-8 BOM
setlocal enabledelayedexpansion
set "CODE_DIR=%~dp0code"
set count=0
for /r "%CODE_DIR%" %%f in (*.c *.h) do (
    set "file=%%f"
    for /f "delims=" %%a in ('powershell -Command "$b=[System.IO.File]::ReadAllBytes('!file!'); if($b.Length -ge 3 -and $b[0] -eq 0xEF -and $b[1] -eq 0xBB -and $b[2] -eq 0xBF){[System.IO.File]::WriteAllBytes('!file!',$b[3..($b.Length-1)]); Write-Output 1}else{Write-Output 0}"') do (
        if "%%a"=="1" (
            echo Fixed: %%~nxf
            set /a count+=1
        )
    )
)
echo.
echo Total fixed: !count! file(s)
pause
