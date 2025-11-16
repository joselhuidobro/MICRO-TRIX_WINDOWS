@ECHO OFF
SETLOCAL EnableDelayedExpansion

REM ========================================
REM   TRIX - Script de gestión Docker
REM   Uso: trix.bat [comando]
REM ========================================

REM Colores (usando PowerShell para mejor visualización)
SET "RED=[91m"
SET "GREEN=[92m"
SET "YELLOW=[93m"
SET "BLUE=[94m"
SET "NC=[0m"

REM Detectar docker compose (plugin v2 o standalone v1)
docker compose version >nul 2>&1
IF %ERRORLEVEL% EQU 0 (
    SET "COMPOSE_CMD=docker compose"
) ELSE (
    SET "COMPOSE_CMD=docker-compose"
)

REM Configuración de archivos compose
SET "COMPOSE=%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/10-data.yml -f compose/20-messaging.yml -f compose/30-apps.yml -f compose/40-observability.yml -f compose/50-edge.yml -f compose/60-docs-build.yml -f compose/70-robotics.yml"

REM Si no hay argumentos, mostrar ayuda
IF "%~1"=="" GOTO :help

REM Dispatcher de comandos
IF /I "%~1"=="up" GOTO :up
IF /I "%~1"=="run" GOTO :up
IF /I "%~1"=="down" GOTO :down
IF /I "%~1"=="restart" GOTO :restart
IF /I "%~1"=="ps" GOTO :ps
IF /I "%~1"=="logs" GOTO :logs
IF /I "%~1"=="build" GOTO :build
IF /I "%~1"=="pull" GOTO :pull
IF /I "%~1"=="pull-all" GOTO :pull-all
IF /I "%~1"=="up-minimal" GOTO :up-minimal
IF /I "%~1"=="up-edge" GOTO :up-edge
IF /I "%~1"=="up-foundation" GOTO :up-foundation
IF /I "%~1"=="up-data" GOTO :up-data
IF /I "%~1"=="up-messaging" GOTO :up-messaging
IF /I "%~1"=="up-apps" GOTO :up-apps
IF /I "%~1"=="up-observability" GOTO :up-observability
IF /I "%~1"=="up-robotics" GOTO :up-robotics
IF /I "%~1"=="up-docs" GOTO :up-docs
IF /I "%~1"=="up-sequential" GOTO :up-sequential
IF /I "%~1"=="down-stage" GOTO :down-stage
IF /I "%~1"=="clean-all" GOTO :clean-all
IF /I "%~1"=="fix-ports" GOTO :fix-ports
IF /I "%~1"=="nuke" GOTO :nuke
IF /I "%~1"=="doctor" GOTO :doctor
IF /I "%~1"=="config" GOTO :config
IF /I "%~1"=="help" GOTO :help

ECHO %RED%Error: Comando desconocido '%~1'%NC%
GOTO :help

REM ========================================
REM   COMANDOS
REM ========================================

:help
ECHO.
ECHO %BLUE%═══════════════════════════════════════════════════════%NC%
ECHO %GREEN%            🤖 TRIX - Comandos disponibles             %NC%
ECHO %BLUE%═══════════════════════════════════════════════════════%NC%
ECHO.
ECHO %YELLOW%Comandos básicos:%NC%
ECHO   %GREEN%trix up%NC% / %GREEN%trix run%NC%      Iniciar todos los servicios
ECHO   %GREEN%trix down%NC%             Detener todos los servicios
ECHO   %GREEN%trix restart%NC%          Reiniciar servicios
ECHO   %GREEN%trix ps%NC%               Ver estado de servicios
ECHO   %GREEN%trix logs [SERVICE]%NC%   Ver logs
ECHO.
ECHO %YELLOW%Inicio por etapas (recomendado):%NC%
ECHO   %GREEN%trix up-sequential%NC%    🚀 Iniciar TODO en orden automático
ECHO   %GREEN%trix up-foundation%NC%    1️⃣  Redis, Networks básicas
ECHO   %GREEN%trix up-data%NC%          2️⃣  PostgreSQL, PgAdmin
ECHO   %GREEN%trix up-messaging%NC%     3️⃣  EMQX, Kafka, RabbitMQ
ECHO   %GREEN%trix up-apps%NC%          4️⃣  APIs, Node-RED, ERP, Web
ECHO   %GREEN%trix up-observability%NC% 5️⃣  Grafana, Prometheus, Loki
ECHO   %GREEN%trix up-edge%NC%          6️⃣  Kong, Caddy, Wireguard
ECHO   %GREEN%trix up-robotics%NC%      7️⃣  ROS2, Micro-ROS
ECHO   %GREEN%trix up-docs%NC%          8️⃣  Hugo, Doxygen, Sphinx
ECHO.
ECHO %YELLOW%Configuraciones rápidas:%NC%
ECHO   %GREEN%trix up-minimal%NC%       Solo servicios core esenciales
ECHO   %GREEN%trix down-stage [NUM]%NC% Detener etapa específica (1-8)
ECHO.
ECHO %YELLOW%Construcción y actualización:%NC%
ECHO   %GREEN%trix build%NC%            Construir imágenes locales
ECHO   %GREEN%trix pull%NC%             Descargar imagen específica
ECHO   %GREEN%trix pull-all%NC%         Descargar TODAS las imágenes (pesado)
ECHO.
ECHO %YELLOW%Diagnóstico y limpieza:%NC%
ECHO   %GREEN%trix doctor%NC%           Diagnóstico del sistema
ECHO   %GREEN%trix config%NC%           Validar configuración compose
ECHO   %GREEN%trix clean-all%NC%        Limpiar contenedores conflictivos
ECHO   %GREEN%trix fix-ports%NC%        Diagnosticar puertos ocupados
ECHO.
ECHO %RED%Limpieza profunda (¡PELIGROSO!):%NC%
ECHO   %RED%trix nuke%NC%             %YELLOW%⚠️  Eliminar TODO de Docker%NC%
ECHO.
ECHO %BLUE%═══════════════════════════════════════════════════════%NC%
ECHO.
EXIT /B 0

:up
ECHO %BLUE%🧹 Pre-limpieza de contenedores conflictivos...%NC%
docker stop cadvisor knot 2>nul
docker rm cadvisor knot 2>nul
ECHO %BLUE%🚀 Iniciando servicios Trix...%NC%
%COMPOSE% up -d
IF %ERRORLEVEL% EQU 0 (
    ECHO %GREEN%✓ Servicios iniciados correctamente%NC%
) ELSE (
    ECHO %RED%✗ Error al iniciar servicios%NC%
    EXIT /B 1
)
ECHO.
ECHO %YELLOW%📊 Estado de servicios:%NC%
%COMPOSE% ps
EXIT /B 0

:down
ECHO %YELLOW%⏹️  Deteniendo servicios...%NC%
%COMPOSE% down -v --remove-orphans
IF %ERRORLEVEL% EQU 0 (
    ECHO %GREEN%✓ Servicios detenidos%NC%
) ELSE (
    ECHO %RED%✗ Error al detener servicios%NC%
)
EXIT /B 0

:restart
ECHO %YELLOW%🔄 Reiniciando servicios...%NC%
%COMPOSE% restart
ECHO %GREEN%✓ Servicios reiniciados%NC%
EXIT /B 0

:up-minimal
ECHO %BLUE%🚀 Iniciando configuración mínima...%NC%
%COMPOSE% up -d redis postgres emqx micro_ros_agent_hmi ui springboot_api node_red trix-erp trixweb
ECHO %GREEN%✓ Configuración mínima iniciada%NC%
EXIT /B 0

:up-edge
ECHO %BLUE%🌐 Iniciando servicios edge...%NC%
%COMPOSE% up -d kong caddy cloudflare_tunnel knot wireguard
ECHO %GREEN%✓ Servicios edge iniciados%NC%
EXIT /B 0

:build
ECHO %BLUE%🔨 Construyendo imágenes...%NC%
%COMPOSE% build
ECHO %GREEN%✓ Imágenes construidas%NC%
EXIT /B 0

:pull
IF "%~2"=="" (
    ECHO %RED%Error: Especifica el compose a descargar%NC%
    ECHO Uso: trix pull [foundation^|data^|messaging^|apps^|observability^|edge^|robotics^|docs]
    ECHO O usa: trix pull-all para descargar todo
    EXIT /B 1
)
ECHO %BLUE%📥 Descargando imágenes de: %~2%NC%
%COMPOSE_CMD% -f compose/%~2.yml pull
ECHO %GREEN%✓ Imágenes de %~2 descargadas%NC%
EXIT /B 0

:pull-all
ECHO %BLUE%📥 Descargando TODAS las imágenes (esto puede tardar)...%NC%
ECHO %YELLOW%⚠️  Esto descargará varios GB. Presiona Ctrl+C para cancelar.%NC%
timeout /t 5
ECHO.
ECHO %YELLOW%Etapa 1/8: Foundation...%NC%
%COMPOSE_CMD% -f compose/00-foundation.yml pull
ECHO %YELLOW%Etapa 2/8: Data...%NC%
%COMPOSE_CMD% -f compose/10-data.yml pull
ECHO %YELLOW%Etapa 3/8: Messaging...%NC%
%COMPOSE_CMD% -f compose/20-messaging.yml pull
ECHO %YELLOW%Etapa 4/8: Apps...%NC%
%COMPOSE_CMD% -f compose/30-apps.yml pull
ECHO %YELLOW%Etapa 5/8: Observability...%NC%
%COMPOSE_CMD% -f compose/40-observability.yml pull
ECHO %YELLOW%Etapa 6/8: Edge...%NC%
%COMPOSE_CMD% -f compose/50-edge.yml pull
ECHO %YELLOW%Etapa 7/8: Robotics...%NC%
%COMPOSE_CMD% -f compose/70-robotics.yml pull
ECHO %YELLOW%Etapa 8/8: Docs...%NC%
%COMPOSE_CMD% -f compose/60-docs-build.yml pull
ECHO %GREEN%✓ Todas las imágenes descargadas%NC%
EXIT /B 0

:up-foundation
ECHO %BLUE%1️⃣  Iniciando capa Foundation (Redis, Networks)...%NC%
%COMPOSE_CMD% -f compose/00-foundation.yml up -d
IF %ERRORLEVEL% EQU 0 (
    ECHO %GREEN%✓ Foundation iniciado%NC%
) ELSE (
    ECHO %RED%✗ Error en Foundation%NC%
    EXIT /B 1
)
EXIT /B 0

:up-data
ECHO %BLUE%2️⃣  Iniciando capa Data (PostgreSQL, PgAdmin)...%NC%
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/10-data.yml up -d
IF %ERRORLEVEL% EQU 0 (
    ECHO %GREEN%✓ Data iniciado%NC%
) ELSE (
    ECHO %RED%✗ Error en Data%NC%
    EXIT /B 1
)
EXIT /B 0

:up-messaging
ECHO %BLUE%3️⃣  Iniciando capa Messaging (EMQX, Kafka, RabbitMQ)...%NC%
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/20-messaging.yml up -d
IF %ERRORLEVEL% EQU 0 (
    ECHO %GREEN%✓ Messaging iniciado%NC%
) ELSE (
    ECHO %RED%✗ Error en Messaging%NC%
    EXIT /B 1
)
EXIT /B 0

:up-apps
ECHO %BLUE%4️⃣  Iniciando capa Apps (APIs, Node-RED, ERP, Web)...%NC%
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/10-data.yml -f compose/20-messaging.yml -f compose/30-apps.yml up -d
IF %ERRORLEVEL% EQU 0 (
    ECHO %GREEN%✓ Apps iniciado%NC%
) ELSE (
    ECHO %RED%✗ Error en Apps%NC%
    EXIT /B 1
)
EXIT /B 0

:up-observability
ECHO %BLUE%5️⃣  Iniciando capa Observability (Grafana, Prometheus, Loki)...%NC%
docker stop cadvisor knot 2>nul
docker rm cadvisor knot 2>nul
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/10-data.yml -f compose/20-messaging.yml -f compose/40-observability.yml up -d
IF %ERRORLEVEL% EQU 0 (
    ECHO %GREEN%✓ Observability iniciado%NC%
) ELSE (
    ECHO %RED%✗ Error en Observability%NC%
    EXIT /B 1
)
EXIT /B 0

:up-robotics
ECHO %BLUE%7️⃣  Iniciando capa Robotics (ROS2, Micro-ROS)...%NC%
%COMPOSE_CMD% -f compose/00-foundation.yml -f compose/70-robotics.yml up -d
IF %ERRORLEVEL% EQU 0 (
    ECHO %GREEN%✓ Robotics iniciado%NC%
) ELSE (
    ECHO %RED%✗ Error en Robotics%NC%
    EXIT /B 1
)
EXIT /B 0

:up-docs
ECHO %BLUE%8️⃣  Iniciando capa Docs (Hugo, Doxygen, Sphinx)...%NC%
%COMPOSE_CMD% -f compose/60-docs-build.yml up -d
IF %ERRORLEVEL% EQU 0 (
    ECHO %GREEN%✓ Docs iniciado%NC%
) ELSE (
    ECHO %RED%✗ Error en Docs%NC%
    EXIT /B 1
)
EXIT /B 0

:up-sequential
ECHO %BLUE%🚀 Iniciando TRIX en secuencia optimizada...%NC%
ECHO %YELLOW%Esto levantará todos los servicios en orden de dependencias%NC%
ECHO.
timeout /t 3
ECHO %BLUE%════════════════════════════════════════════════════════%NC%
ECHO %YELLOW%Etapa 1/8: Foundation (Redis, Networks)%NC%
CALL :up-foundation
IF %ERRORLEVEL% NEQ 0 EXIT /B 1
ECHO %GREEN%✓ Esperando 5s para estabilización...%NC%
timeout /t 5 >nul
ECHO.
ECHO %BLUE%════════════════════════════════════════════════════════%NC%
ECHO %YELLOW%Etapa 2/8: Data (PostgreSQL, PgAdmin)%NC%
CALL :up-data
IF %ERRORLEVEL% NEQ 0 EXIT /B 1
ECHO %GREEN%✓ Esperando 10s para estabilización...%NC%
timeout /t 10 >nul
ECHO.
ECHO %BLUE%════════════════════════════════════════════════════════%NC%
ECHO %YELLOW%Etapa 3/8: Messaging (EMQX, Kafka)%NC%
CALL :up-messaging
IF %ERRORLEVEL% NEQ 0 EXIT /B 1
ECHO %GREEN%✓ Esperando 10s para estabilización...%NC%
timeout /t 10 >nul
ECHO.
ECHO %BLUE%════════════════════════════════════════════════════════%NC%
ECHO %YELLOW%Etapa 4/8: Apps (APIs, Node-RED, ERP)%NC%
CALL :up-apps
IF %ERRORLEVEL% NEQ 0 EXIT /B 1
ECHO %GREEN%✓ Esperando 10s para estabilización...%NC%
timeout /t 10 >nul
ECHO.
ECHO %BLUE%════════════════════════════════════════════════════════%NC%
ECHO %YELLOW%Etapa 5/8: Observability (Grafana, Prometheus, Loki)%NC%
CALL :up-observability
IF %ERRORLEVEL% NEQ 0 EXIT /B 1
ECHO %GREEN%✓ Esperando 10s para estabilización...%NC%
timeout /t 10 >nul
ECHO.
ECHO %BLUE%════════════════════════════════════════════════════════%NC%
ECHO %YELLOW%Etapa 6/8: Edge (Kong, Caddy, Wireguard)%NC%
CALL :up-edge
IF %ERRORLEVEL% NEQ 0 EXIT /B 1
ECHO %GREEN%✓ Esperando 5s para estabilización...%NC%
timeout /t 5 >nul
ECHO.
ECHO %BLUE%════════════════════════════════════════════════════════%NC%
ECHO %YELLOW%Etapa 7/8: Robotics (ROS2, Micro-ROS)%NC%
CALL :up-robotics
IF %ERRORLEVEL% NEQ 0 EXIT /B 1
ECHO %GREEN%✓ Esperando 5s para estabilización...%NC%
timeout /t 5 >nul
ECHO.
ECHO %BLUE%════════════════════════════════════════════════════════%NC%
ECHO %YELLOW%Etapa 8/8: Docs (Hugo, Doxygen, Sphinx)%NC%
CALL :up-docs
IF %ERRORLEVEL% NEQ 0 EXIT /B 1
ECHO.
ECHO %BLUE%════════════════════════════════════════════════════════%NC%
ECHO %GREEN%✓✓✓ TRIX iniciado completamente ✓✓✓%NC%
ECHO %BLUE%════════════════════════════════════════════════════════%NC%
ECHO.
ECHO %YELLOW%📊 Estado final:%NC%
%COMPOSE% ps
ECHO.
ECHO %GREEN%🎉 Sistema listo para usar%NC%
EXIT /B 0

:down-stage
IF "%~2"=="" (
    ECHO %RED%Error: Especifica la etapa a detener (1-8)%NC%
    ECHO Etapas: 1=foundation, 2=data, 3=messaging, 4=apps, 5=observability, 6=edge, 7=robotics, 8=docs
    EXIT /B 1
)
IF "%~2"=="1" SET "STAGE_FILE=compose/00-foundation.yml"
IF "%~2"=="2" SET "STAGE_FILE=compose/10-data.yml"
IF "%~2"=="3" SET "STAGE_FILE=compose/20-messaging.yml"
IF "%~2"=="4" SET "STAGE_FILE=compose/30-apps.yml"
IF "%~2"=="5" SET "STAGE_FILE=compose/40-observability.yml"
IF "%~2"=="6" SET "STAGE_FILE=compose/50-edge.yml"
IF "%~2"=="7" SET "STAGE_FILE=compose/70-robotics.yml"
IF "%~2"=="8" SET "STAGE_FILE=compose/60-docs-build.yml"
ECHO %YELLOW%⏹️  Deteniendo etapa %~2...%NC%
%COMPOSE_CMD% -f %STAGE_FILE% down -v
ECHO %GREEN%✓ Etapa %~2 detenida%NC%
EXIT /B 0

:ps
ECHO %BLUE%📋 Estado de los servicios:%NC%
%COMPOSE% ps
EXIT /B 0

:logs
IF "%~2"=="" (
    ECHO %YELLOW%📝 Mostrando logs de todos los servicios...%NC%
    %COMPOSE% logs -f --tail=200
) ELSE (
    ECHO %YELLOW%📝 Mostrando logs de: %~2%NC%
    %COMPOSE% logs -f --tail=200 %~2
)
EXIT /B 0

:config
ECHO %BLUE%⚙️  Validando configuración...%NC%
%COMPOSE% config
EXIT /B 0

:clean-all
ECHO %YELLOW%🧹 Limpiando contenedores conflictivos...%NC%
docker stop cadvisor knot 2>nul
docker rm cadvisor knot 2>nul
FOR /F "tokens=*" %%i IN ('docker ps -aq --filter "status=exited" 2^>nul') DO docker rm %%i 2>nul
ECHO %GREEN%✓ Limpieza completada%NC%
EXIT /B 0

:fix-ports
ECHO %BLUE%🔍 Diagnosticando puertos...%NC%
ECHO.
ECHO %YELLOW%Puerto 8080 (cAdvisor):%NC%
netstat -ano | findstr :8080 || ECHO   %GREEN%✓ Libre%NC%
ECHO.
ECHO %YELLOW%Puerto 1053 (Knot DNS):%NC%
netstat -ano | findstr :1053 || ECHO   %GREEN%✓ Libre%NC%
ECHO.
ECHO %YELLOW%Puerto 3000 (Grafana):%NC%
netstat -ano | findstr :3000 || ECHO   %GREEN%✓ Libre%NC%
ECHO.
ECHO %YELLOW%Puerto 9090 (Prometheus):%NC%
netstat -ano | findstr :9090 || ECHO   %GREEN%✓ Libre%NC%
ECHO.
ECHO %YELLOW%Puerto 3100 (Loki):%NC%
netstat -ano | findstr :3100 || ECHO   %GREEN%✓ Libre%NC%
ECHO.
ECHO %YELLOW%Contenedores huérfanos:%NC%
docker ps -a --filter "status=exited" --format "table {{.Names}}\t{{.Status}}" 2>nul || ECHO   %GREEN%✓ Ninguno%NC%
ECHO.
ECHO %YELLOW%💡 Para liberar un puerto:%NC%
ECHO   1. Encuentra el PID: netstat -ano ^| findstr :PUERTO
ECHO   2. Mata el proceso: taskkill /PID XXXX /F
EXIT /B 0

:doctor
ECHO %BLUE%🔍 Diagnóstico del sistema:%NC%
ECHO.
ECHO %YELLOW%Docker:%NC%
docker --version 2>nul || ECHO %RED%✗ Docker no encontrado%NC%
ECHO.
ECHO %YELLOW%Compose (plugin):%NC%
docker compose version 2>nul || ECHO %RED%✗ No disponible%NC%
ECHO.
ECHO %YELLOW%Compose (v1):%NC%
docker-compose --version 2>nul || ECHO %RED%✗ No disponible%NC%
ECHO.
ECHO %YELLOW%Espacio en disco:%NC%
wmic logicaldisk get caption,freespace,size 2>nul
ECHO.
ECHO %YELLOW%Memoria:%NC%
wmic OS get FreePhysicalMemory,TotalVisibleMemorySize /Value 2>nul
ECHO.
ECHO %YELLOW%Contenedores activos:%NC%
docker ps --format "table {{.Names}}\t{{.Status}}" 2>nul
ECHO.
ECHO %YELLOW%Imágenes Docker:%NC%
docker images --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}" 2>nul
EXIT /B 0

:nuke
ECHO.
ECHO %RED%⚠️  ADVERTENCIA: Esta operación eliminará:%NC%
ECHO   • Todos los contenedores (incluso corriendo)
ECHO   • Todas las imágenes Docker
ECHO   • Todos los volúmenes
ECHO   • Todas las redes personalizadas
ECHO   • Toda la caché de build
ECHO.
ECHO %YELLOW%Esta acción NO se puede deshacer.%NC%
ECHO.
SET /P "confirm=¿Estás seguro? Escribe 'SI ESTOY SEGURO' para continuar: "
IF /I NOT "!confirm!"=="SI ESTOY SEGURO" (
    ECHO %GREEN%✓ Operación cancelada%NC%
    EXIT /B 0
)

ECHO.
ECHO %RED%💣 Iniciando limpieza profunda de Docker...%NC%
ECHO.

ECHO %YELLOW%1️⃣  Deteniendo servicios Compose...%NC%
%COMPOSE% down -v --remove-orphans 2>nul
ECHO %GREEN%   ✓ Servicios Compose detenidos%NC%

ECHO %YELLOW%2️⃣  Deteniendo TODOS los contenedores...%NC%
FOR /F "tokens=*" %%i IN ('docker ps -aq 2^>nul') DO docker stop %%i 2>nul
ECHO %GREEN%   ✓ Contenedores detenidos%NC%

ECHO %YELLOW%3️⃣  Eliminando TODOS los contenedores...%NC%
FOR /F "tokens=*" %%i IN ('docker ps -aq 2^>nul') DO docker rm -f %%i 2>nul
ECHO %GREEN%   ✓ Contenedores eliminados%NC%

ECHO %YELLOW%4️⃣  Eliminando TODAS las imágenes...%NC%
FOR /F "tokens=*" %%i IN ('docker images -aq 2^>nul') DO docker rmi -f %%i 2>nul
ECHO %GREEN%   ✓ Imágenes eliminadas%NC%

ECHO %YELLOW%5️⃣  Eliminando TODOS los volúmenes...%NC%
FOR /F "tokens=*" %%i IN ('docker volume ls -q 2^>nul') DO docker volume rm %%i 2>nul
ECHO %GREEN%   ✓ Volúmenes eliminados%NC%

ECHO %YELLOW%6️⃣  Eliminando redes personalizadas...%NC%
docker network prune -f 2>nul
ECHO %GREEN%   ✓ Redes eliminadas%NC%

ECHO %YELLOW%7️⃣  Limpiando caché de build...%NC%
docker builder prune -af 2>nul
ECHO %GREEN%   ✓ Caché limpiada%NC%

ECHO %YELLOW%8️⃣  Limpieza profunda del sistema...%NC%
docker system prune -af --volumes 2>nul
ECHO %GREEN%   ✓ Sistema limpio%NC%

ECHO.
ECHO %GREEN%✓ Limpieza profunda completada%NC%
ECHO %BLUE%ℹ️  El host Docker ha sido restaurado a estado limpio%NC%
ECHO %YELLOW%⚠️  Ejecuta 'trix pull' y 'trix build' para reconstruir%NC%
EXIT /B 0