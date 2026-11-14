# Atajos de trabajo para WSL / Ubuntu (requiere GNU Make)
SHELL := /usr/bin/env bash
IP ?= 192.168.185.3
DOMAIN ?= 67

.PHONY: help test sim check clock deploy stop docker-build docker-run

help:
	@echo "Comandos disponibles (WSL / Ubuntu):"
	@echo "  make test                - Ejecuta la suite pytest en 1_competencia/tests"
	@echo "  make sim                 - Ejecuta prueba de navegacion en simulador 2D"
	@echo "  make check               - Diagnostico de topicos, sensores y reloj"
	@echo "  make clock               - Verifica desfase /scan vs /odom"
	@echo "  make deploy IP=<ip>      - Sincroniza codigo a la Raspberry Pi por rsync"
	@echo "  make stop                - Parada de emergencia inmediata"
	@echo "  make docker-build        - Construye la imagen Docker en tb4_docker/"
	@echo "  make docker-run IP=<ip>  - Entra al contenedor conectado al robot"

test:
	python3 -m pytest 1_competencia/tests -v

sim:
	./2_laboratorio/test_sim_navegacion.sh

check:
	./tb4_check.sh

clock:
	python3 scripts/chequeo_reloj_tb4.py

deploy:
	./desplegar_en_robot.sh $(IP)

stop:
	./parar.sh

docker-build:
	cd tb4_docker && docker compose build

docker-run:
	cd tb4_docker && TB4_IP=$(IP) ROS_DOMAIN_ID=$(DOMAIN) docker compose run --rm tb4
