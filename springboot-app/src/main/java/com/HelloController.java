package com.example.demo;

import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.server.ResponseStatusException;
import org.springframework.http.HttpStatus;

import io.fabric8.kubernetes.client.KubernetesClient;
import io.fabric8.kubernetes.client.KubernetesClientBuilder;
import io.fabric8.kubernetes.client.KubernetesClientException;
import io.fabric8.kubernetes.api.model.Pod;
import io.fabric8.kubernetes.api.model.PodList;
import io.fabric8.kubernetes.api.model.ContainerStatus;

import java.util.*;
import java.util.stream.Collectors;

@RestController
class HelloController {

  @GetMapping("/hello")
  public String hello() {
    return "Hello, World!";
  }

  // DTO inmutable (Java 17+). Spring lo serializa a JSON automáticamente.
  record PodInfo(
      String name,
      String namespace,
      String phase,
      String reason,
      String node,
      String podIP,
      String startTime,
      int readyContainers,
      int totalContainers,
      int restartsTotal
  ) {}

  @GetMapping("/list_pods")
  public List<PodInfo> listPods(
      @RequestParam(name = "ns", required = false) String namespace // opcional: filtra por namespace
  ) {
    try (KubernetesClient client = new KubernetesClientBuilder().build()) {
      PodList podList = (namespace == null || namespace.isBlank())
          ? client.pods().inAnyNamespace().list()
          : client.pods().inNamespace(namespace).list();

      List<Pod> items = podList.getItems();
      if (items == null) return List.of();

      return items.stream().map(p -> {
        var meta = p.getMetadata();
        var status = p.getStatus();

        int total = Optional.ofNullable(p.getSpec())
            .map(s -> s.getContainers() == null ? 0 : s.getContainers().size())
            .orElse(0);

        List<ContainerStatus> css = Optional.ofNullable(status)
            .map(s -> s.getContainerStatuses() == null ? List.<ContainerStatus>of() : s.getContainerStatuses())
            .orElse(List.of());

        int ready = (int) css.stream().filter(ContainerStatus::getReady).count();
        int restarts = css.stream()
            .mapToInt(cs -> cs.getRestartCount() == null ? 0 : cs.getRestartCount())
            .sum();

        return new PodInfo(
            meta != null ? meta.getName() : null,
            meta != null ? meta.getNamespace() : null,
            status != null ? status.getPhase() : "Unknown",
            status != null ? status.getReason() : null,
            status != null ? status.getHostIP() : null,
            status != null ? status.getPodIP() : null,
            status != null ? status.getStartTime() : null,
            ready,
            total,
            restarts
        );
      }).collect(Collectors.toList());

    } catch (KubernetesClientException e) {
      throw new ResponseStatusException(
          HttpStatus.BAD_GATEWAY,
          "Error al consultar Kubernetes: " + e.getMessage(),
          e
      );
    } catch (Exception e) {
      throw new ResponseStatusException(
          HttpStatus.INTERNAL_SERVER_ERROR,
          "Fallo inesperado al listar pods",
          e
      );
    }
  }
}

