function initAutocomplete() {
    var input = document.getElementById('location-input');
    if (!input) {
        console.error("Elemento con id 'location-input' no encontrado.");
        return;
    }

    var autocomplete = new google.maps.places.Autocomplete(input);

    autocomplete.addListener('place_changed', function () {
        var place = autocomplete.getPlace();
        if (place.geometry) {
            var lat = place.geometry.location.lat();
            var lon = place.geometry.location.lng();
            // Asignar 'lat' y 'lon' a los campos de texto
            document.getElementById('lat-output').value = lat;
            document.getElementById('lon-output').value = lon;
        } else {
            console.error("No se encontró la geometría del lugar seleccionado.");
        }
    });

    console.log("Autocomplete inicializado correctamente.");
}
