document.addEventListener("DOMContentLoaded", function () {
    // Agregar el listener al botón "Usar Datos" una vez que el DOM esté cargado
    var usarDatosBtn = document.getElementById("usar-datos-btn");
    if (usarDatosBtn) {
        usarDatosBtn.addEventListener("click", function () {
            // Obtener los valores de dirección, latitud y longitud
            var direccion = document.getElementById("location-input").value;
            var latitud = document.getElementById("lat-output").value;
            var longitud = document.getElementById("lon-output").value;

            // Validar que los valores no estén vacíos
            if (!direccion || !latitud || !longitud) {
                alert("Por favor completa los campos de dirección, latitud y longitud.");
                return;
            }

            // Llenar los campos del formulario con los valores obtenidos
            var direccionFormulario = document.getElementById("direccion");
            var latFormulario = document.getElementById("lat");
            var lonFormulario = document.getElementById("lon");

            if (direccionFormulario && latFormulario && lonFormulario) {
                direccionFormulario.value = direccion;
                latFormulario.value = latitud;
                lonFormulario.value = longitud;
            } else {
                console.error("Uno o más campos del formulario no existen.");
            }
        });
    } else {
        console.error("El botón con id 'usar-datos-btn' no se encontró.");
    }
});
