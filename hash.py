    
genera hash

docker exec -it compose-trix-erp-1 python -c "from werkzeug.security import generate_password_hash; print(generate_password_hash('trix', method='pbkdf2:sha256:600000'))"


vacio
docker exec -it postgres_trix psql -U trix -d trixdb -c "INSERT INTO usuarios (correo, password_hash, nivel_seguridad, nombre, rfc, razon_social, cel) VALUES ('joselhuidobro@gmail.com', 'HASH_AQUI', 3, 'Jose Luis', 'XAXX010101000', 'Administrador', '5550000000');"


con hash = trix (pruebas)
docker exec -it postgres_trix psql -U trix -d trixdb -c "INSERT INTO usuarios (correo, password_hash, nivel_seguridad, nombre, rfc, razon_social, cel) VALUES ('joselhuidobro@gmail.com', 'pbkdf2:sha256:600000$ZU8wT0hzjPbJnGvt$c5ba9a1f36d3621d29c4be5b356ed6f54a4349166744422dfba3022a1df8b2c0', 3, 'Jose Luis', 'XAX', 'Administrador', '5550000000');"


Validar
docker exec -it postgres_trix psql -U trix -d trixdb -c "SELECT id, correo, nombre, nivel_seguridad FROM usuarios WHERE correo='joselhuidobro@gmail.com';"