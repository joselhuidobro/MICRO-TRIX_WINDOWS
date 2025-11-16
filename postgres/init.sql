-- postgres-init/init.sql

-- Extensiones
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- Tabla buildings
CREATE TABLE IF NOT EXISTS buildings (
    id SERIAL PRIMARY KEY,
    fecha DATE NOT NULL DEFAULT CURRENT_DATE,
    cel VARCHAR(20) NOT NULL,
    rfc VARCHAR(20),
    administrador VARCHAR(100) NOT NULL,
    num_predial INTEGER,
    fecha_construccion INTEGER,
    cp INTEGER,
    no_deptos INTEGER,
    no_pisos INTEGER,
    accesos_peatonales INTEGER,
    accesos_vehiculares INTEGER,
    lugares_estacionamiento INTEGER,
    lat DOUBLE PRECISION,
    lon DOUBLE PRECISION,
    direccion VARCHAR(255)
);

-- Tabla cotizaciones_historico
CREATE TABLE IF NOT EXISTS cotizaciones_historico (
    id SERIAL PRIMARY KEY,
    fecha DATE NOT NULL DEFAULT CURRENT_DATE,
    cel VARCHAR(20) NOT NULL,
    lat DOUBLE PRECISION,
    lon DOUBLE PRECISION,
    direccion VARCHAR(255),
    nombre VARCHAR(100) NOT NULL,
    concepto VARCHAR(100)
);

-- Tabla departamentos
CREATE TABLE IF NOT EXISTS departamentos (
    id SERIAL PRIMARY KEY,
    no_del_depto VARCHAR(10) NOT NULL,
    piso INTEGER NOT NULL,
    cel VARCHAR(20) NOT NULL,
    contacto VARCHAR(100) NOT NULL,
    rfc VARCHAR(20),
    correo VARCHAR(255),
    no_estacionamientos INTEGER DEFAULT 0,
    roof_garden BOOLEAN DEFAULT FALSE,
    puerta_automatica BOOLEAN DEFAULT FALSE,
    no_cuartos INTEGER DEFAULT 1,
    building_id INTEGER NOT NULL REFERENCES buildings(id) ON DELETE CASCADE
);

-- Tabla historico_ventas
CREATE TABLE IF NOT EXISTS historico_ventas (
    id SERIAL PRIMARY KEY,
    cel VARCHAR(20) NOT NULL,
    nombre VARCHAR(100) NOT NULL,
    fecha DATE NOT NULL DEFAULT CURRENT_DATE,
    direccion VARCHAR(255),
    concepto VARCHAR(100),
    categoria VARCHAR(50),
    no_semana INTEGER,
    dia_semana VARCHAR(20),
    cantidad INTEGER DEFAULT 0,
    precio_unitario NUMERIC(10, 2) DEFAULT 0.00,
    total_cotizado NUMERIC(12, 2) DEFAULT 0.00,
    total_pagado NUMERIC(12, 2) DEFAULT 0.00,
    forma_de_pago VARCHAR(50),
    estado VARCHAR(50) DEFAULT 'pendiente',
    h_llegada TIME,
    accion VARCHAR(100),
    cambiar_estado BOOLEAN DEFAULT FALSE,
    horario VARCHAR(50),
    unidad_trix VARCHAR(50),
    lat DOUBLE PRECISION,
    lon DOUBLE PRECISION
);

-- Tabla ordenes_semana
CREATE TABLE IF NOT EXISTS ordenes_semana (
    id SERIAL PRIMARY KEY,
    cel VARCHAR(20) NOT NULL,
    nombre VARCHAR(100) NOT NULL,
    fecha DATE NOT NULL DEFAULT CURRENT_DATE,
    direccion VARCHAR(255),
    concepto VARCHAR(100),
    categoria VARCHAR(50),
    no_semana INTEGER,
    dia_semana VARCHAR(20),
    cantidad INTEGER DEFAULT 0,
    precio_unitario NUMERIC(10, 2) DEFAULT 0.00,
    total_cotizado NUMERIC(12, 2) DEFAULT 0.00,
    total_pagado NUMERIC(12, 2) DEFAULT 0.00,
    forma_de_pago VARCHAR(50),
    estado VARCHAR(50) DEFAULT 'pendiente',
    h_llegada TIME,
    accion VARCHAR(100),
    cambiar_estado BOOLEAN DEFAULT FALSE,
    horario VARCHAR(50),
    unidad_trix VARCHAR(50),
    lat DOUBLE PRECISION,
    lon DOUBLE PRECISION,
    en_ruta BOOLEAN DEFAULT FALSE
);

-- Tabla usuarios
CREATE TABLE IF NOT EXISTS usuarios (
    id SERIAL PRIMARY KEY,
    nombre VARCHAR(150),
    telefono VARCHAR(50),
    nivel_seguridad INTEGER,
    correo VARCHAR(150) UNIQUE NOT NULL,
    password_hash TEXT,
    rfc VARCHAR(255),
    razon_social VARCHAR(255),
    cel VARCHAR(255)
);

-- Tabla vecinos (CORREGIDA)
CREATE TABLE IF NOT EXISTS vecinos (
    id SERIAL PRIMARY KEY,
    id_edificio INTEGER NOT NULL REFERENCES buildings(id) ON DELETE CASCADE,
    id_depto INTEGER NOT NULL REFERENCES departamentos(id) ON DELETE CASCADE,
    direccion VARCHAR(255),
    lat DOUBLE PRECISION,
    lon DOUBLE PRECISION,
    interior VARCHAR(10),
    piso INTEGER,
    cel VARCHAR(20),
    rfc VARCHAR(20),
    correo VARCHAR(255),
    nombre VARCHAR(255) NOT NULL,
    password VARCHAR(255) NOT NULL,
    nivel_seguridad INTEGER,
    razon_social VARCHAR(255)
);

-- Índices para rendimiento
CREATE INDEX IF NOT EXISTS idx_buildings_cel ON buildings(cel);
CREATE INDEX IF NOT EXISTS idx_deptos_building ON departamentos(building_id);
CREATE INDEX IF NOT EXISTS idx_ordenes_cel ON ordenes_semana(cel);
CREATE INDEX IF NOT EXISTS idx_ordenes_fecha ON ordenes_semana(fecha);