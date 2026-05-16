@echo off
chcp 65001 >nul
set PYTHONUTF8=1

REM Создаём папку для логов и графиков, если её нет
if not exist "graphics" mkdir graphics

echo ====================================================
echo [1/3] Убедитесь, что Unity запущен и нажат ▶ Play!
echo ====================================================
pause

echo [2/3] Запуск AUV Brain... (логи в graphics\logs.txt)
python -u auv_brain.py > graphics\logs.txt 2>&1

if %errorlevel% neq 0 (
    echo.
    echo Ошибка в auv_brain.py! Содержимое logs.txt:
    type graphics\logs.txt
    echo.
    pause
    exit /b
)

echo [3/3] Маршрут завершён. Строю графики...
python plot_auv_logs.py graphics\logs.txt --output graphics\diploma

echo.
echo Готово! Все файлы сохранены в папку .\graphics\
pause