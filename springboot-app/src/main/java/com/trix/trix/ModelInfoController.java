package com.trix.web;

import com.trix.model.ModelInfo;
import com.trix.service.ModelInfoService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/model")
public class ModelInfoController {

    private final ModelInfoService svc;

    public ModelInfoController(ModelInfoService svc) {
        this.svc = svc;
    }

    @GetMapping("/info")
    public ResponseEntity<ModelInfo> info() throws Exception {
        ModelInfo info = svc.readAndRemember();
        return ResponseEntity.ok()
                .eTag("\"" + info.sha256() + "\"")
                .body(info);
    }
}

