@echo off
rem Double-click to show the hand mapping model chosen in the launcher in a simulation window, driven by the glove
title Hand mapping viewer
tasklist /FI "IMAGENAME eq vcxsrv.exe" | find /I "vcxsrv.exe" >nul || start "" "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -wgl -ac
rem Only one program can listen to the glove: close a viewer left open from before
docker exec dexcap pkill -f view_hand_mapping.py >nul 2>&1
echo Opening the simulation window. Move your right hand slowly (it updates about 3 times a second).
echo Close the simulation window, or press Ctrl+C here, to stop.
echo.
docker exec -it -w /workspace/DexCap -e DISPLAY=host.docker.internal:0.0 -e PYTHONPATH=/workspace/DexCap dexcap /home/.venv/bin/python -W ignore scripts/view_hand_mapping.py
echo.
pause
