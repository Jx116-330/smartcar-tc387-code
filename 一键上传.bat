@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

cd /d %~dp0

echo 当前仓库：%cd%
echo.

set /p MSG=请输入本次更新说明: 
if "%MSG%"=="" (
    echo 更新说明不能为空。
    pause
    exit /b 1
)

echo.
echo [1/4] 查看仓库状态...
git status --short
if errorlevel 1 goto :fail

echo.
echo [2/4] 添加变更...
git add .
if errorlevel 1 goto :fail

echo.
echo [3/4] 提交变更...
git commit -m "%MSG%"
if errorlevel 1 (
    echo.
    echo 提交失败。可能是没有可提交的变更，或提交说明含有特殊字符导致命令失败。
    echo 你可以先检查 git status，必要时换一个简单一点的提交说明再试。
    pause
    exit /b 1
)

echo.
echo [4/4] 推送到 GitHub...
git push
if errorlevel 1 goto :fail

echo.
echo 上传完成。
pause
exit /b 0

:fail
echo.
echo 执行失败，请检查上面的 git 输出。
pause
exit /b 1
