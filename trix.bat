@ECHO OFF
SETLOCAL EnableDelayedExpansion

:: ====================================================
:: 1. DEFINICIÓN SEGURA DE COLORES (Sin caracteres invisibles)
:: ====================================================
:init
:: Truco para generar el caracter ESC sin corromper el archivo
FOR /F "tokens=1,2 delims=#" %%a IN ('"prompt #$H#$E# & echo on & for %%b in (1) do rem"') DO (
  SET "ESC=%%b"
)

:: Definición de Colores
SET "RED=%ESC%[91m"
SET "GREEN=%ESC%[92m"
SET "YELLOW=%ESC%[93m"
SET "BLUE=%ESC%[94m"
SET "CYAN=%ESC%[96m"
SET "NC=%ESC%[0m"

:: Verificar Docker
docker info >nul 2>&1
IF %ERRORLEVEL% NEQ 0 (
    ECHO %RED%[ERROR] Docker no esta corriendo. Inicia Docker Desktop primero.%NC%
    EXIT /B 1
)

:: Definir comando Compose
docker compose version >nul 2>&1
IF %ERRORLEVEL% EQU 0 (
    SET "COMPOSE_CMD=docker compose"
) ELSE (
    SET "COMPOSE_CMD=docker-compose"
)

:: Definir archivos del stack (Todo en una linea para evitar errores)
SET "COMPOSE=%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/10-data.yml -f compose/20-messaging.yml -f compose/30-apps.yml -f compose/40-observability.yml -f compose/50-edge.yml -f compose/60-docs-build.yml -f compose/70-robotics.yml"

:: ====================================================
:: 2. BANNER (ASCII ESTANDAR - A PRUEBA DE FALLOS)
:: ====================================================
:banner
CLS
ECHO.
ECHO %CYAN%===================================%NC%
ECHO %CYAN%  T R I X   A U T O M A T I O N   %NC%
ECHO %CYAN%===================================%NC%
ECHO.

:: Si no hay argumentos, mostrar ayuda
IF "%~1"=="" GOTO :help

:: ====================================================
:: 3. DISPATCHER (ENRUTADOR)
:: ====================================================

:: Comandos Principales
IF /I "%~1"=="up" GOTO :up
IF /I "%~1"=="run" GOTO :up
IF /I "%~1"=="down" GOTO :down
IF /I "%~1"=="restart" GOTO :restart
IF /I "%~1"=="ps" GOTO :ps
IF /I "%~1"=="logs" GOTO :logs
IF /I "%~1"=="build" GOTO :build
IF /I "%~1"=="pull" GOTO :pull
IF /I "%~1"=="pull-all" GOTO :pull-all

:: Perfiles
IF /I "%~1"=="up-minimal" GOTO :up-minimal
IF /I "%~1"=="up-edge" GOTO :up-edge
IF /I "%~1"=="up-foundation" GOTO :up-foundation
IF /I "%~1"=="up-data" GOTO :up-data
IF /I "%~1"=="up-messaging" GOTO :up-messaging
IF /I "%~1"=="up-apps" GOTO :up-apps
IF /I "%~1"=="up-observability" GOTO :up-observability
IF /I "%~1"=="up-robotics" GOTO :up-robotics
IF /I "%~1"=="up-docs" GOTO :up-docs

:: Utilerias
IF /I "%~1"=="up-sequential" GOTO :up-sequential
IF /I "%~1"=="down-stage" GOTO :down-stage
IF /I "%~1"=="clean-all" GOTO :clean-all
IF /I "%~1"=="fix-ports" GOTO :fix-ports
IF /I "%~1"=="doctor" GOTO :doctor
IF /I "%~1"=="config" GOTO :config
IF /I "%~1"=="nuke" GOTO :nuke
IF /I "%~1"=="help" GOTO :help

:: Error catch-all
ECHO %RED%Error: Comando '%~1' no reconocido.%NC%
ECHO Usa 'trix help' para ver los comandos.
EXIT /B 1

:: ====================================================
:: 4. FUNCIONES
:: ====================================================

:help
ECHO %YELLOW%--- COMANDOS DISPONIBLES ---%NC%
ECHO trix up             : Iniciar todo
ECHO trix down           : Detener todo
ECHO trix build          : Construir imagenes
ECHO trix ps             : Ver estado
ECHO trix up-sequential  : Inicio ordenado (Recomendado)
ECHO trix nuke           : Limpieza profunda (Peligro)
EXIT /B 0

:up
ECHO %BLUE%Iniciando servicios...%NC%
%COMPOSE% up -d
EXIT /B 0

:down
ECHO %YELLOW%Deteniendo servicios...%NC%
%COMPOSE% down -v --remove-orphans
EXIT /B 0

:restart
%COMPOSE% restart
EXIT /B 0

:ps
%COMPOSE% ps
EXIT /B 0

:logs
IF "%~2"=="" (
    %COMPOSE% logs -f --tail=100
) ELSE (
    %COMPOSE% logs -f --tail=100 %~2
)
EXIT /B 0

:build
ECHO %BLUE%Construyendo imagenes...%NC%
%COMPOSE% build
EXIT /B 0

:pull
IF "%~2"=="" (
    ECHO Error: Indica el archivo (ej: trix pull messaging)
    EXIT /B 1
)
%COMPOSE_CMD% -f compose/%~2.yml pull
EXIT /B 0

:pull-all
ECHO %BLUE%Descargando todo...%NC%
FOR %%F IN (foundation data messaging apps observability edge robotics docs-build) DO (
    ECHO Descargando %%F...
    %COMPOSE_CMD% -f compose/%%F.yml pull
)
EXIT /B 0

:: --- PERFILES ---
:up-minimal
%COMPOSE% up -d redis postgres emqx micro_ros_agent_hmi ui springboot_api node_red trix-erp trixweb
EXIT /B 0

:up-edge
%COMPOSE% up -d kong caddy cloudflare_tunnel knot wireguard
EXIT /B 0

:up-foundation
%COMPOSE_CMD% -f compose/00-foundation.yml up -d
EXIT /B 0

:up-data
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/10-data.yml up -d
EXIT /B 0

:up-messaging
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/20-messaging.yml up -d
EXIT /B 0

:up-apps
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/10-data.yml -f compose/20-messaging.yml -f compose/30-apps.yml up -d
EXIT /B 0

:up-observability
docker stop cadvisor knot 2>nul
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/10-data.yml -f compose/20-messaging.yml -f compose/40-observability.yml up -d
EXIT /B 0

:up-robotics
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/70-robotics.yml up -d
EXIT /B 0

:up-docs
%COMPOSE_CMD% -f compose/60-docs-build.yml up -d
EXIT /B 0

:: --- SECUENCIAL ---
:up-sequential
ECHO %BLUE%Inicio Secuencial...%NC%
CALL :up-foundation
timeout /t 5 >nul
CALL :up-data
timeout /t 5 >nul
CALL :up-messaging
timeout /t 5 >nul
CALL :up-apps
timeout /t 5 >nul
CALL :up-observability
CALL :up-edge
CALL :up-robotics
CALL :up-docs
ECHO %GREEN%Sistema Iniciado.%NC%
EXIT /B 0

:down-stage
IF "%~2"=="" EXIT /B 1
ECHO Deteniendo etapa %~2...
IF "%~2"=="1" %COMPOSE_CMD% -f compose/00-foundation.yml down -v
IF "%~2"=="2" %COMPOSE_CMD% -f compose/10-data.yml down -v
IF "%~2"=="3" %COMPOSE_CMD% -f compose/20-messaging.yml down -v
IF "%~2"=="4" %COMPOSE_CMD% -f compose/30-apps.yml down -v
IF "%~2"=="5" %COMPOSE_CMD% -f compose/40-observability.yml down -v
IF "%~2"=="6" %COMPOSE_CMD% -f compose/50-edge.yml down -v
IF "%~2"=="7" %COMPOSE_CMD% -f compose/70-robotics.yml down -v
IF "%~2"=="8" %COMPOSE_CMD% -f compose/60-docs-build.yml down -v
EXIT /B 0

:: --- MANTENIMIENTO ---
:clean-all
docker stop cadvisor knot 2>nul
docker rm cadvisor knot 2>nul
FOR /F "tokens=*" %%i IN ('docker ps -aq --filter "status=exited"') DO docker rm %%i 2>nul
EXIT /B 0

:fix-ports
netstat -ano | findstr :8080
netstat -ano | findstr :3000
ECHO Usa taskkill /PID [NUMERO] /F para liberar.
EXIT /B 0

:doctor
docker --version
%COMPOSE_CMD% version
docker system df
EXIT /B 0

:config
%COMPOSE% config
EXIT /B 0

:: --- NUKE ---
:nuke
CLS
ECHO %RED%!!! ADVERTENCIA DE LIMPIEZA PROFUNDA !!!%NC%
ECHO Esto eliminara TODOS los contenedores, imagenes y volumenes.
SET /P "confirm=Escribe 'NUKE' para confirmar: "
IF /I NOT "!confirm!"=="NUKE" EXIT /B 0

ECHO %YELLOW%Deteniendo todo...%NC%
%COMPOSE% down -v --remove-orphans 2>nul
FOR /F "tokens=*" %%i IN ('docker ps -aq') DO (
    docker stop %%i >nul 2>&1
    docker kill %%i >nul 2>&1
)

ECHO %YELLOW%Borrando contenedores y volumenes...%NC%
docker volume prune -f >nul 2>&1
FOR /F "tokens=*" %%i IN ('docker ps -aq') DO docker rm -f %%i >nul 2>&1
FOR /F "tokens=*" %%i IN ('docker volume ls -q') DO docker volume rm -f %%i >nul 2>&1

ECHO %YELLOW%Borrando imagenes...%NC%
docker image prune -a -f >nul 2>&1
FOR /F "tokens=*" %%i IN ('docker images -aq') DO docker rmi -f %%i >nul 2>&1

ECHO %YELLOW%Limpiando redes y cache...%NC%
docker network prune -f >nul 2>&1
docker builder prune --all --force >nul 2>&1
docker system prune -a -f --volumes >nul 2>&1
ipconfig /flushdns >nul 2>&1

IF EXIST "wsl.exe" wsl --compact-memory >nul 2>&1

ECHO %RED%Quieres ejecutar borrado seguro de espacio libre? (Tarda mucho)%NC%
SET /P "sw=S/N: "
IF /I "!sw!"=="S" cipher /w:%CD%

ECHO %GREEN%Limpieza terminada.%NC%
EXIT /B 0