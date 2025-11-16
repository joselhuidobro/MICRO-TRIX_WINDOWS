// static/js/first_personview.js
function activateFirstPersonView(lat, lon, zoomLevel) {
    // Verifica si el objeto 'map' está disponible
    if (typeof map === 'undefined') {
        console.error("El objeto 'map' no está definido.");
        return;
    }

    // Lógica para activar la vista en primera persona
    map.easeTo({
        center: [lon, lat], // Coordenadas específicas
        zoom: 22,           // Nivel de zoom (puedes ajustar a tu gusto)
        pitch: 90,          // Inclinación para una vista en primera persona (horizontal)
        bearing: 0,         // Orientación inicial de la cámara (0 = norte)
        duration: 2000,     // Duración de la animación en milisegundos
        essential: true
    });

    console.log(`Vista en primera persona activada en [${lat}, ${lon}] con zoom ${zoomLevel}`);

    // Agregamos el listener para “caminar”
    document.addEventListener('keydown', handleFirstPersonMovement);
}

function handleFirstPersonMovement(event) {
    // Obtenemos la orientación y el centro actuales del mapa
    const bearing = map.getBearing(); // Orientación en grados
    const center = map.getCenter();   // Retorna un objeto {lng, lat}

    // Definimos la distancia a “caminar” en cada pulsación
    // Ajusta step a tu gusto.
    const step = 0.0001;

    // Convertimos el bearing a radianes para usar con Math.cos / Math.sin
    const bearingRad = bearing * Math.PI / 180;

    // Variables para la nueva posición
    let newLat = center.lat;
    let newLng = center.lng;

    if (event.key === 'ArrowUp') {
        // Avanzar hacia adelante
        newLat += step * Math.cos(bearingRad);
        newLng += step * Math.sin(bearingRad);
    } 
    else if (event.key === 'ArrowDown') {
        // Retroceder
        newLat -= step * Math.cos(bearingRad);
        newLng -= step * Math.sin(bearingRad);
    } 
    else if (event.key === 'ArrowLeft') {
        // Desplazar a la izquierda (strafe)
        const leftBearingRad = (bearing - 90) * Math.PI / 180;
        newLat += step * Math.cos(leftBearingRad);
        newLng += step * Math.sin(leftBearingRad);
    }
    else if (event.key === 'ArrowRight') {
        // Desplazar a la derecha (strafe)
        const rightBearingRad = (bearing + 90) * Math.PI / 180;
        newLat += step * Math.cos(rightBearingRad);
        newLng += step * Math.sin(rightBearingRad);
    } else {
        // Si no es ninguna flecha de movimiento, no hacemos nada
        return;
    }

    // Actualizamos la posición de la cámara
    map.easeTo({
        center: [newLng, newLat],
        bearing: bearing,      // Mantenemos la misma orientación
        pitch: 90,            // Conservamos la vista en primera persona
        zoom: map.getZoom(),   // Mantenemos el mismo nivel de zoom
        duration: 300,         // Una pequeña animación al “caminar”
        essential: true
    });
}
