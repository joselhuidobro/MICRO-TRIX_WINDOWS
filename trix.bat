@ECHO OFF
SETLOCAL

:: ====================================================
:: 1. INICIALIZACIÓN
:: ====================================================
:init
FOR /F %%a IN ('echo prompt $E ^| cmd') DO SET "ESC=%%a"
SET "RED=%ESC%[91m"
SET "GREEN=%ESC%[92m"
SET "YELLOW=%ESC%[93m"
SET "BLUE=%ESC%[94m"
SET "CYAN=%ESC%[96m"
SET "NC=%ESC%[0m"

docker info >nul 2>&1
IF %ERRORLEVEL% NEQ 0 (
    ECHO %RED%[ERROR] Docker no esta corriendo.%NC%
    PAUSE
    EXIT /B 1
)

docker compose version >nul 2>&1
IF %ERRORLEVEL% EQU 0 ( SET "COMPOSE_CMD=docker compose" ) ELSE ( SET "COMPOSE_CMD=docker-compose" )
SET "COMPOSE=%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/10-data.yml -f compose/20-messaging.yml -f compose/30-apps.yml -f compose/40-observability.yml -f compose/50-edge.yml -f compose/60-docs-build.yml -f compose/70-robotics.yml"

:: ====================================================
:: 2. ENRUTADOR
:: ====================================================
:banner
CLS
ECHO.
ECHO %CYAN%===================================%NC%
ECHO %CYAN%  I N G E N I E R Í A   T R I X    %NC%
ECHO %CYAN%===================================%NC%
ECHO.

IF "%~1"=="" GOTO :help

IF /I "%~1"=="up" GOTO :up
IF /I "%~1"=="down" GOTO :down
IF /I "%~1"=="ps" GOTO :ps
IF /I "%~1"=="logs" GOTO :logs
IF /I "%~1"=="build" GOTO :build
IF /I "%~1"=="pull" GOTO :pull
IF /I "%~1"=="up-sequential" GOTO :up-sequential
IF /I "%~1"=="doctor" GOTO :doctor
IF /I "%~1"=="help" GOTO :help

:: --- COMANDOS DE LIMPIEZA ---
IF /I "%~1"=="clean" GOTO :clean
IF /I "%~1"=="nuke" GOTO :nuke
IF /I "%~1"=="super_nuke" GOTO :super_nuke

IF /I "%~1"=="up-minimal" GOTO :up-minimal

ECHO %RED%Comando desconocido.%NC%
EXIT /B 1

:: ====================================================
:: 3. AYUDA Y COMANDOS BASICOS
:: ====================================================

:help
ECHO %YELLOW%--- COMANDOS DISPONIBLES ---%NC%
ECHO trix up             : Iniciar todo
ECHO trix down           : Detener todo
ECHO trix up-sequential  : Inicio ordenado
ECHO.
ECHO %GREEN%trix clean          : Reinicio Rapido (Mantiene imagenes)%NC%
ECHO %YELLOW%trix nuke           : Reinicio TOTAL (Limpia cache oculta)%NC%
ECHO %RED%trix super_nuke     : Borrado Forense (Desgasta SSD)%NC%
EXIT /B 0

:up
%COMPOSE% up -d
EXIT /B 0

:down
%COMPOSE% down -v --remove-orphans
EXIT /B 0

:ps
%COMPOSE% ps
EXIT /B 0

:logs
IF "%~2"=="" ( %COMPOSE% logs -f --tail=100 ) ELSE ( %COMPOSE% logs -f --tail=100 %~2 )
EXIT /B 0

:build
%COMPOSE% build
EXIT /B 0

:up-sequential
ECHO %BLUE%Inicio Secuencial...%NC%
%COMPOSE_CMD% -f compose/00-foundation.yml up -d
timeout /t 5 >nul
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/10-data.yml up -d
timeout /t 5 >nul
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/20-messaging.yml up -d
timeout /t 5 >nul
%COMPOSE% up -d
ECHO %GREEN%Sistema Listo.%NC%
EXIT /B 0

:up-minimal
%COMPOSE% up -d redis postgres emqx micro_ros_agent_hmi ui springboot_api node_red trix-erp trixweb
EXIT /B 0

:doctor
docker system df
EXIT /B 0

:: ====================================================
:: 4. CLEAN (SEMI-NUKE) - USO DIARIO
:: ====================================================
:clean
ECHO.
ECHO %GREEN%--- LIMPIEZA RAPIDA (CLEAN) ---%NC%
ECHO - Borra contenedores y volumenes (DB reset).
ECHO - Mantiene imagenes (Sin descargas).
ECHO.
SET "confirm="
SET /P "confirm=Presiona ENTER para limpiar (o escribe N para salir): "
IF /I "%confirm%"=="N" EXIT /B 0

ECHO %BLUE%Limpiando...%NC%
%COMPOSE% down -v --remove-orphans
docker system prune -f
ECHO %GREEN%Listo.%NC%
EXIT /B 0

:: ====================================================
:: 5. NUKE (PROFUNDO MEJORADO) - REINSTALACION
:: ====================================================
:nuke
CLS
ECHO.
ECHO %YELLOW%*****************************************%NC%
ECHO %YELLOW%** PROTOCOLO NUKE (LIMPIEZA PROFUNDA) **%NC%
ECHO %YELLOW%*****************************************%NC%
ECHO.
ECHO %YELLOW%Se eliminara TODO:%NC%
ECHO  - Contenedores, Volumenes, Redes.
ECHO  - Imagenes, Cache de BuildKit (Oculta).
ECHO  - Builders zombis.
ECHO.
SET "confirm="
SET /P "confirm=Escribe NUKE para confirmar: "

IF /I NOT "%confirm%"=="NUKE" (
    ECHO %GREEN%Operacion cancelada.%NC%
    EXIT /B 0
)

GOTO :run_nuke_logic

:: ====================================================
:: 6. SUPER NUKE (FORENSE) - BORRADO SEGURO
:: ====================================================
:super_nuke
CLS
ECHO.
ECHO %RED%*****************************************%NC%
ECHO %RED%** SUPER NUKE (LIMPIEZA FORENSE)      **%NC%
ECHO %RED%*****************************************%NC%
ECHO.
ECHO %RED%1. Ejecutara NUKE completo.%NC%
ECHO %RED%2. Sobrescribira el disco con ceros (Cipher).%NC%
ECHO %YELLOW%ADVERTENCIA: Desgaste de SSD.%NC%
ECHO.

SET "confirm="
SET /P "confirm=Escribe SUPER para confirmar: "

IF /I NOT "%confirm%"=="SUPER" (
    ECHO %GREEN%Operacion cancelada.%NC%
    EXIT /B 0
)

:: CORRECCION: Usamos simbolos seguros (===) en vez de flechas
ECHO %RED%=== FASE 1: LIMPIEZA LOGICA === %NC%
CALL :run_nuke_logic_no_exit

ECHO.
ECHO %RED%=== FASE 2: LIMPIEZA FISICA (CIPHER) === %NC%
ECHO No cierres la ventana...
cipher /w:"%CD%"

ECHO.
ECHO %GREEN%SUPER NUKE COMPLETADO.%NC%
EXIT /B 0


:: ====================================================
:: LOGICA DE BORRADO (MEJORADA)
:: ====================================================
:run_nuke_logic
CALL :run_nuke_logic_no_exit
ECHO %GREEN%Limpieza profunda terminada.%NC%
EXIT /B 0

:run_nuke_logic_no_exit
ECHO %RED%1. Deteniendo todo...%NC%
%COMPOSE% down -v --remove-orphans >nul 2>&1
FOR /F "tokens=*" %%i IN ('docker ps -aq') DO (
    docker stop %%i >nul 2>&1
    docker rm -f %%i >nul 2>&1
)

ECHO %RED%2. Borrando infraestructura...%NC%
docker volume prune -f >nul 2>&1
docker network prune -f >nul 2>&1
FOR /F "tokens=*" %%i IN ('docker volume ls -q') DO docker volume rm -f %%i >nul 2>&1

ECHO %RED%3. Borrando imagenes y cache oculta...%NC%
:: Aqui esta la mejora: Buildx prune es mas agresivo que system prune
docker builder prune --all --force >nul 2>&1
docker image prune -a -f >nul 2>&1
docker system prune -a -f --volumes >nul 2>&1

ECHO %RED%4. Limpiando entorno Host...%NC%
ipconfig /flushdns >nul 2>&1
IF EXIST "wsl.exe" wsl --compact-memory >nul 2>&1
EXIT /B 0