#!/usr/bin/env bash
set -euo pipefail

NAMESPACE=trix

echo ">> Borrando stack de dev (overlay minikube)"
kubectl delete -k k8s/overlays/minikube || true
kubectl delete ns "$NAMESPACE" || true

