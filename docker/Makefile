# Makefile for Turtlebot4 sim docker image
#
# Usage(Currently no args):
#   make <target> <arg-name>=<arg-value>
#
# Examples:
#   make
#   make build-base

IMAGE_NAME = tb4_simulation
DOCKERFILE1 = ${PWD}/Dockerfile_base
DOCKERFILE2 = ${PWD}/Dockerfile_ros2

# linux/arm64 for board, amd64 for pc
DEVICE_ARCH := $(shell dpkg --print-architecture)
ifeq ($(DEVICE_ARCH),arm64)
  DEVICE_ARCH := linux/arm64
endif

#####################################################################
#						BUILD TARGETS								#
#####################################################################
# Build the base image
.PHONY: build-base
build-base:
	@echo "Building base image..."
	@docker build --platform ${DEVICE_ARCH} -f ${DOCKERFILE1} -t ${IMAGE_NAME}_base ..

# Build the overlay image & remove the base
.PHONY: build-overlay
build-overlay: build-base
	@echo "Building overlay image..."
	@docker build --platform ${DEVICE_ARCH} -f ${DOCKERFILE2} -t ${IMAGE_NAME} ..
	@docker rmi ${IMAGE_NAME}_base

# Remove old Docker containers
.PHONY: clean
clean:
	@echo "Removing old Docker containers of the image..."
	@docker ps -a -q --filter "ancestor=${IMAGE_NAME}" | xargs -r docker rm
	@docker compose down --remove-orphans

# Docker compose container running script
# Allows Docker containers to access the X server required for GUI applications
.PHONY: start_container
start_container:
	@echo "Running the container..."
	@xhost +local:docker
	@docker compose run ${IMAGE_NAME} bash

# Start the a new terminal in the container
.PHONY: terminal
terminal:
	@@container_id=$(shell docker ps -q --filter "ancestor=${IMAGE_NAME}") && \
    if [ -n "$$container_id" ]; then \
        docker exec -it $$container_id bash; \
    else \
        echo "No running container found for the ${IMAGE_NAME} image!"; \
    fi

# Build all targets and start the container if not specified
.DEFAULT_GOAL := all
all: build-base build-overlay start_container
