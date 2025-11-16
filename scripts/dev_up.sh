#!/usr/bin/env bash
set -euo pipefail

NAMESPACE=trix
CTX_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo ">> Crear/asegurar namespace"
kubectl create ns "$NAMESPACE" --dry-run=client -o yaml | kubectl apply -f -

echo ">> Construir imagen dentro de Minikube (si docker-runtime) usando el contexto del proyecto"
cd "$CTX_ROOT"
if minikube status | grep -q "docker-env: in-use"; then
  # Daemon Docker dentro de minikube disponible
  eval "$(minikube -p minikube docker-env)"
  docker build -t dronui:latest -f 2025_DOCKER-dronui/Dockerfile 2025_DOCKER-dronui
else
  # Runtime containerd: usar minikube image build/load
  minikube image build -t dronui:latest -f 2025_DOCKER-dronui/Dockerfile .
fi

echo ">> Desplegar overlay de minikube"
kubectl apply -k k8s/overlays/minikube

echo ">> Esperando rollout de ui..."
kubectl -n "$NAMESPACE" rollout status deploy/ui

echo ">> Recursos en $NAMESPACE"
kubectl -n "$NAMESPACE" get deploy,svc,pods -o wide
echo ">> Abre: http://$(minikube ip):30500/"

