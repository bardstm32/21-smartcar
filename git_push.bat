@echo off
cd /d "%~dp0"

echo Adding files...
git add *

echo Committing...
git commit -m "update"

echo Pushing to GitHub...
git push

echo.
echo Done!
pause
