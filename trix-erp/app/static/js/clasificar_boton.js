document.addEventListener("DOMContentLoaded", function () {
    document.getElementById("clasificar-dato-btn").addEventListener("click", function () {
        // Obtener los valores de los campos
        var direccion = document.getElementById("location-input").value;
        var latitud = document.getElementById("lat-output").value;
        var longitud = document.getElementById("lon-output").value;

        // Validar que todos los campos tengan datos
        if (!direccion || !latitud || !longitud) {
            alert("Por favor, completa todos los campos antes de clasificar.");
            return;
        }

        // Crear el objeto de datos
        var datos = {
            dir: direccion,
            lat: parseFloat(latitud),
            lon: parseFloat(longitud)
        };

        // Enviar los datos al servidor usando fetch
        fetch(predictUrl, { // Usar predictUrl definido en el HTML
            method: "POST",
            headers: {
                "Content-Type": "application/json",
                "X-CSRFToken": csrfToken // Usar csrfToken definido en el HTML
            },
            body: JSON.stringify(datos)
        })
        .then(response => response.json())
        .then(data => {
            // Acceder correctamente a la clave 'clasificacion' del JSON devuelto
            if (data.clasificacion) {
                document.getElementById("resultado-clasificacion").innerText = "Clasificación: " + data.clasificacion;
            } else {
                document.getElementById("resultado-clasificacion").innerText = "Error: No se encontró clasificación.";
            }
        })
        .catch(error => {
            console.error("Error:", error);
            document.getElementById("resultado-clasificacion").innerText = "Ocurrió un error al clasificar.";
        });
    });
});