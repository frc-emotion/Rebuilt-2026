@echo off
set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk
cd /d C:\Users\FRCTeam2658\Documents\Rebuilt-2026
echo Deploying to roboRIO (team 2658)...
.\gradlew.bat deploy -PteamNumber=2658
pause
