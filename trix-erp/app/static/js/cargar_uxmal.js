// Asegurarse de que THREE y uxmalGltfUrl estén definidos
if (typeof THREE === 'undefined') {
  console.error("Three.js no está cargado.");
}

if (typeof uxmalGltfUrl === 'undefined') {
  console.error("La variable uxmalGltfUrl no está definida.");
}

// Esperar a que el mapa esté completamente cargado antes de agregar la capa personalizada
map.on('style.load', function () {
  // Inicializar el cargador GLTF (solo uno para todos los modelos)
  const loader = new THREE.GLTFLoader();

  /**
   * Función que crea un objeto de capa personalizada para Three.js.
   * @param {Object} config Configuración para el modelo 3D.
   * @param {string} config.id           - ID único para la capa en Mapbox.
   * @param {string} config.modelUrl     - URL del modelo GLTF/GLB.
   * @param {Array}  config.origin       - [lng, lat] del origen del modelo.
   * @param {number} config.altitude     - Altura del modelo.
   * @param {Array}  config.rotate       - [rotX, rotY, rotZ] en radianes.
   */
  function create3DModelLayer(config) {
    // Calcular la posición en coordenadas de Mapbox a partir de lng/lat
    const modelAsMercatorCoordinate = mapboxgl.MercatorCoordinate.fromLngLat(
      config.origin,
      config.altitude
    );

    // Empaquetar la transformación (posición, rotación y escala)
    const modelTransform = {
      translateX: modelAsMercatorCoordinate.x,
      translateY: modelAsMercatorCoordinate.y,
      translateZ: modelAsMercatorCoordinate.z,
      rotateX: config.rotate[0],
      rotateY: config.rotate[1],
      rotateZ: config.rotate[2],
      // El método .meterInMercatorCoordinateUnits() retorna la escala adecuada
      scale: modelAsMercatorCoordinate.meterInMercatorCoordinateUnits()
    };

    // Capa personalizada para Mapbox
    const customLayer = {
      id: config.id,
      type: 'custom',
      renderingMode: '3d',

      onAdd: function (map, gl) {
        this.camera = new THREE.Camera();
        this.scene = new THREE.Scene();

        

        // Luces para iluminar el modelo
        const directionalLight1 = new THREE.DirectionalLight(0xffffff);
        directionalLight1.position.set(0, -70, 100).normalize();
        this.scene.add(directionalLight1);

        const directionalLight2 = new THREE.DirectionalLight(0xffffff);
        directionalLight2.position.set(0, 70, 100).normalize();
        this.scene.add(directionalLight2);

        // Cargar el modelo 3D con GLTFLoader
        loader.load(
          config.modelUrl,
          (gltf) => {
            this.scene.add(gltf.scene);
          },
          undefined,
          (error) => {
            console.error('Error al cargar el modelo 3D:', error);
          }
        );

        // Guardamos la referencia del mapa
        this.map = map;

        // Configuramos el renderizador de Three.js usando el contexto de Mapbox
        this.renderer = new THREE.WebGLRenderer({
          canvas: map.getCanvas(),
          context: gl,
          antialias: true
        });
        this.renderer.autoClear = false;

        // Guardamos la transformación en el objeto para usarla en render()
        this.modelTransform = modelTransform;
      },

      render: function (gl, matrix) {
        // Calcular la matriz de rotación X, Y, Z
        const rotationX = new THREE.Matrix4().makeRotationAxis(
          new THREE.Vector3(1, 0, 0),
          this.modelTransform.rotateX
        );
        const rotationY = new THREE.Matrix4().makeRotationAxis(
          new THREE.Vector3(0, 1, 0),
          this.modelTransform.rotateY
        );
        const rotationZ = new THREE.Matrix4().makeRotationAxis(
          new THREE.Vector3(0, 0, 1),
          this.modelTransform.rotateZ
        );

        // Matriz que viene de Mapbox
        const m = new THREE.Matrix4().fromArray(matrix);

        // Matriz de traslación y escala del modelo
        const l = new THREE.Matrix4()
          .makeTranslation(
            this.modelTransform.translateX,
            this.modelTransform.translateY,
            this.modelTransform.translateZ
          )
          .scale(
            new THREE.Vector3(
              this.modelTransform.scale,
              -this.modelTransform.scale, // invertimos para que coincida con el eje Y de WebGL
              this.modelTransform.scale
            )
          )
          .multiply(rotationX)
          .multiply(rotationY)
          .multiply(rotationZ);

        // Multiplicamos la matriz final para la cámara
        this.camera.projectionMatrix = m.multiply(l);

        // Renderizamos la escena
        this.renderer.resetState();
        this.renderer.render(this.scene, this.camera);

        // Informamos a Mapbox que se debe repintar en el próximo cuadro
        this.map.triggerRepaint();
      }
    };

    return customLayer;
  }

  // ======= CONFIGURACIÓN DEL MODELO PRINCIPAL (ACTUAL) ======= 
  const mainModelConfig = {
    id: '3d-model', // conservamos el mismo id
    modelUrl: uxmalGltfUrl, // la URL del modelo
    origin: [-99.1602753, 19.3735765], // coordenadas [lng, lat]
    altitude: 0,
    rotate: [Math.PI / 2, 0, 0] // rotaciones en radianes
  };

  // Creamos la capa con el modelo principal
  const main3DLayer = create3DModelLayer(mainModelConfig);

  // Añadir la capa personalizada al mapa, si no existe
  if (!map.getLayer(mainModelConfig.id)) {
    map.addLayer(main3DLayer, 'waterway-label');
  } else {
    console.warn(`La capa '${mainModelConfig.id}' ya existe en el mapa.`);
  }


  // ======= CONFIGURACIÓN PARA TU SEGUNDO MODELO ======= 
  const cubeModelConfig = {
    id: '3d-cube',               // ID único
    modelUrl: bigcuboGltfUrl,       // La variable que definida en html
    origin: [-99.1602753, 19.3735765],   // Ajustar a la ubicación  [lng, lat]
    altitude: 0,
    rotate: [Math.PI / 2, 0, 0]  // Ajusta la rotación 
  };



  // Crear la nueva capa con nuestro segundo modelo
  const cube3DLayer = create3DModelLayer(cubeModelConfig);

  // Agregar la capa al mapa (por ejemplo, por encima de 'waterway-label')
  if (!map.getLayer(cubeModelConfig.id)) {
    map.addLayer(cube3DLayer, 'waterway-label');
  }
  
  // Aquí podrías crear otros modelos, pero asegúrate de definir sus configs
  // antes de crear la capa. Por ejemplo:
  /*
  const newModelConfig = {
    id: 'otro-modelo',
    modelUrl: otraUrl,
    origin: [lng, lat],
    altitude: 0,
    rotate: [0, 0, 0]
  };
  const new3DLayer = create3DModelLayer(newModelConfig);
  map.addLayer(new3DLayer, 'waterway-label');
  */
});
