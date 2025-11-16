#!/usr/bin/env bash
set -euo pipefail
NAMESPACE=trix
kubectl -n "$NAMESPACE" logs deploy/ui -f

