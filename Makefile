# Detecta docker compose (plugin v2) o docker-compose (v1)
COMPOSE_CMD := $(shell docker compose version >/dev/null 2>&1 && echo "docker compose" || echo "docker-compose")

COMPOSE = $(COMPOSE_CMD) \
 -f compose/00-foundation.yml \
 -f compose/10-data.yml \
 -f compose/20-messaging.yml \
 -f compose/30-apps.yml \
 -f compose/40-observability.yml \
 -f compose/50-edge.yml \
 -f compose/60-docs-build.yml \
 -f compose/70-robotics.yml

.PHONY: up run down ps logs build pull restart docs config doctor

# Alias útil
run: up

up:
	@$(COMPOSE) up -d

down:
	@$(COMPOSE) down -v --remove-orphans

up-minimal:
	@$(COMPOSE) up -d redis postgres emqx micro_ros_agent_hmi ui springboot_api node_red trix-erp trixweb

up-edge:
	@$(COMPOSE) up -d kong caddy cloudflare_tunnel knot wireguard

build:
	@$(COMPOSE) build

pull:
	@$(COMPOSE) pull

restart:
	@$(COMPOSE) restart

docs:
	@$(COMPOSE) run --rm hugo_build
	@$(COMPOSE) run --rm doxygen_build
	@$(COMPOSE) run --rm sphinx_build

ps:
	@$(COMPOSE) ps
	

logs:
	@$(COMPOSE) logs -f --tail=200 $(SERVICE)

config:
	@$(COMPOSE) config

doctor:
	@echo "Docker:" && docker --version || true
	@echo "Compose (plugin):" && docker compose version || true
	@echo "Compose (v1):" && docker-compose --version || true




