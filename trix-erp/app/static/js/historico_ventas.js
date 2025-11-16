$(document).ready(function() {
    $('#actualizar-btn').click(function() {
        $.ajax({
            url: "{{ url_for('agenda.actualizar') }}",  // Llama a la ruta Flask
            type: "POST",  
            data: { 
                // Agrega cualquier dato que necesites enviar al servidor 
                // Por ejemplo, un token CSRF:
                csrf_token: "{{ csrf_token() }}" 
            },
            success: function(response) {
                console.log("Tabla actualizada:", response);
            },
            error: function(error) {
                // Maneja los errores de la solicitud AJAX
                console.error("Error al actualizar la tabla:", error);
            }
        });
    });
});
