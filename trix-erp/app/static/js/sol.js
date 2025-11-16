// sol.js

// Esta función asume que tendrás acceso a la `scene` de Three.js.
// Se puede ajustar según tu implementación de Three.js + Mapbox.
function agregarLuz(scene) {
    // Luz direccional
    const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
    // Ajusta la posición según la orientación que desees
    directionalLight.position.set(0, 10, 10).normalize();
  
    // También puedes añadir una luz ambiental suave
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.3);
  
    scene.add(directionalLight);
    scene.add(ambientLight);
  }
  
  // Exportamos la función para usarla en otro script
  export { agregarLuz };
  