
    document.getElementById("usar-datos-btn").addEventListener("click", function () {
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
        document.getElementById("direccion").value = direccion;
        document.getElementById("lat").value = latitud;
        document.getElementById("lon").value = longitud;
    });

    document.getElementById("cantidad").addEventListener("input", function () {
        var cantidad = parseFloat(document.getElementById("cantidad").value);
        var precio_unitario = parseFloat(document.getElementById("precio_unitario").value);
        document.getElementById("total_cotizado").value = cantidad * precio_unitario;
    });

    document.getElementById("precio_unitario").addEventListener("input", function () {
        var cantidad = parseFloat(document.getElementById("cantidad").value);
        var precio_unitario = parseFloat(document.getElementById("precio_unitario").value);
        document.getElementById("total_cotizado").value = cantidad * precio_unitario;
    });

    document.getElementById("total_cotizado").addEventListener("input", function () {
        var total_cotizado = parseFloat(document.getElementById("total_cotizado").value);
        var total_pagado = parseFloat(document.getElementById("total_pagado").value);
        document.getElementById("pendiente").value = total_cotizado - total_pagado;
    });

    document.getElementById("total_pagado").addEventListener("input", function () {
        var total_cotizado = parseFloat(document.getElementById("total_cotizado").value);
        var total_pagado = parseFloat(document.getElementById("total_pagado").value);
        document.getElementById("pendiente").value = total_cotizado - total_pagado;
    });

    

    document.getElementById("fecha").value = new Date().toISOString().slice(0, 10);
