package com.trix.service;

import com.trix.model.ModelInfo;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

import java.io.*;
import java.nio.file.*;
import java.nio.file.attribute.FileTime;
import java.security.MessageDigest;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;

@Service
public class ModelInfoService {

    private final Path modelPath;
    private final Path lastHashPath;

    public ModelInfoService(@Value("${model.path}") String modelPathStr) {
        this.modelPath = Paths.get(modelPathStr).toAbsolutePath();
        this.lastHashPath = this.modelPath.resolveSibling(this.modelPath.getFileName() + ".sha256.last");
    }

    public ModelInfo readAndRemember() throws Exception {
        if (!Files.exists(modelPath)) {
            throw new FileNotFoundException("No existe el modelo: " + modelPath);
        }

        long size = Files.size(modelPath);
        FileTime mtime = Files.getLastModifiedTime(modelPath);
        String modifiedIso = DateTimeFormatter.ISO_INSTANT
                .withZone(ZoneOffset.UTC)
                .format(Instant.ofEpochMilli(mtime.toMillis()));

        String sha = sha256Of(modelPath);
        String prev = readPreviousHash();

        boolean changed = (prev != null) && !prev.equals(sha);

        // Guardamos el hash actual como “último visto” para que la próxima llamada compare contra este
        writePreviousHash(sha);

        return new ModelInfo(modelPath.toString(), modifiedIso, size, sha, prev, changed);
    }

    private String readPreviousHash() {
        try {
            if (Files.exists(lastHashPath)) {
                return Files.readString(lastHashPath).trim();
            }
        } catch (IOException ignored) {}
        return null;
    }

    private void writePreviousHash(String sha) {
        try {
            Files.writeString(lastHashPath, sha, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);
        } catch (IOException ignored) {}
    }

    private static String sha256Of(Path file) throws Exception {
        MessageDigest md = MessageDigest.getInstance("SHA-256");
        try (InputStream in = Files.newInputStream(file)) {
            byte[] buf = new byte[8192];
            int r;
            while ((r = in.read(buf)) != -1) md.update(buf, 0, r);
        }
        StringBuilder sb = new StringBuilder();
        for (byte b : md.digest()) sb.append(String.format("%02x", b));
        return sb.toString();
    }
}

