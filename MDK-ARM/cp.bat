@echo off
for /F %%i in ('git symbolic-ref --short -q HEAD') do (set branch=%%i)
copy update.bin %branch%-update.bin
copy %branch%-update.bin \\192.168.1.116\share\car\main\