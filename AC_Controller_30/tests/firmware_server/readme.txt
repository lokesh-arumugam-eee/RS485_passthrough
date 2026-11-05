python server.py --host 0.0.0.0 --port 8000 --dir .\firmware


pyinstaller --onefile --name firmware_server server.py

.\dist\firmware_server.exe --host 0.0.0.0 --port 8000 --dir .\firmware