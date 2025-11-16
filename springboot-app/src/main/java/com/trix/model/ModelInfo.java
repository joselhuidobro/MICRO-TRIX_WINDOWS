package com.trix.model;

public record ModelInfo(
        String path,
        String modified,      // ISO-8601
        long sizeBytes,
        String sha256,
        String previousSha256,
        boolean changed
) {}

