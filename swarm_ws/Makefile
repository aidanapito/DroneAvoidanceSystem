# Makefile for Drone Swarm System

.PHONY: help build test clean run demo setup

help: ## Show this help message
	@echo "Drone Swarm System - Available Commands:"
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

build: ## Build the ROS 2 workspace
	@echo "Building ROS 2 workspace..."
	colcon build --symlink-install

build-release: ## Build with release optimizations
	@echo "Building with release optimizations..."
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

test: ## Run tests
	@echo "Running tests..."
	colcon test --packages-select swarm_core

test-all: ## Run all tests
	@echo "Running all tests..."
	colcon test

clean: ## Clean build artifacts
	@echo "Cleaning build artifacts..."
	colcon build --cmake-clean-cache
	rm -rf build/ install/ log/

run: ## Run the swarm system (cluttered world)
	@echo "Launching swarm system..."
	ros2 launch swarm_core three_uavs.launch.py world:=cluttered

run-open: ## Run with open world
	@echo "Launching swarm system with open world..."
	ros2 launch swarm_core three_uavs.launch.py world:=open

demo: ## Run demonstration scenario
	@echo "Running demonstration scenario..."
	ros2 launch swarm_core three_uavs.launch.py world:=cluttered enable_plotjuggler:=true

setup: ## Setup development environment
	@echo "Setting up development environment..."
	chmod +x scripts/setup_ubuntu.sh
	./scripts/setup_ubuntu.sh

install-deps: ## Install Python dependencies
	@echo "Installing Python dependencies..."
	pip3 install -r src/swarm_core/requirements.txt

format: ## Format Python code
	@echo "Formatting Python code..."
	black src/swarm_core/
	flake8 src/swarm_core/

lint: ## Run linting checks
	@echo "Running linting checks..."
	flake8 src/swarm_core/
	mypy src/swarm_core/

docs: ## Generate documentation
	@echo "Generating documentation..."
	@echo "Documentation generation not yet implemented"

evaluate: ## Evaluate system performance
	@echo "Evaluating system performance..."
	python3 scripts/eval.py --rosbag latest_run.bag

# Development shortcuts
dev: build test ## Build and test
full: clean build test ## Full clean build and test
