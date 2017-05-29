@echo off
del "OUTPUTFILE.TXT"
rem set PATH=d:\Keil_v5\UV4
rem UV4.exe -b D:\STM32\Cube\LCD_ST7735_BMP280\SlickEdit\\..\MDK-ARM\LCD_ST7735_BMP280.uvprojx -o"OUTPUTFILE.TXT"
UV4.exe -b LCD_ST7735_BMP280.uvprojx -j0 -o"OUTPUTFILE.TXT"
echo.
echo ================================================================ BUILD LOG ================================================================
type OUTPUTFILE.TXT
