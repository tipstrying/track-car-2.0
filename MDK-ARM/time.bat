@echo off
for /F %%i in ('git rev-parse --short HEAD') do (set commitid=%%i)
echo #define BUILDTIME "build time: %date% %time%\r\nbuild by: %username%\r\ncommitid: %commitid%"

for /F %%i in ('git rev-parse --short HEAD') do (set commitid=%%i)
echo #define BUILDTIMEJSON "{\"build-time\": \"%date% %time%\",\"builder\": \"%username%\",\"commitid\": \"%commitid%\"}"