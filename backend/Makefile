# Makefile for Physical AI & Humanoid Robotics Book Chatbot Backend

.PHONY: help install run dev test health ingest setup clean docker-build docker-run lint format

# Default target
help:
	@echo "Physical AI & Humanoid Robotics Book Chatbot Backend"
	@echo ""
	@echo "Available commands:"
	@echo "  install     - Install Python dependencies"
	@echo "  setup       - Initialize environment and Qdrant collection"
	@echo "  dev         - Run in development mode with auto-reload"
	@echo "  run         - Run in production mode"
	@echo "  test        - Run API tests"
	@echo "  health      - Check API health"
	@echo "  ingest      - Ingest book content"
	@echo "  lint        - Run code linting"
	@echo "  format      - Format code"
	@echo "  clean       - Clean cache and temporary files"
	@echo "  docker-build - Build Docker image"
	@echo "  docker-run  - Run Docker container"

# Installation
install:
	pip install -r requirements.txt

# Setup environment
setup:
	python scripts/setup_environment.py

# Development
dev:
	python -m uvicorn main:app --reload --host 0.0.0.0 --port 7860

# Production
run:
	python -m uvicorn main:app --host 0.0.0.0 --port 7860

# Testing
test:
	python test_api.py

# Health check
health:
	curl http://localhost:7860/health

# Ingest content
ingest:
	python scripts/ingest_content.py --content-dir book_content

# Ingest with overwrite
ingest-overwrite:
	python scripts/ingest_content.py --content-dir book_content --overwrite

# Code quality
lint:
	@echo "Running linters..."
	flake8 . --count --select=E9,F63,F7,F82 --show-source --statistics
	mypy rag/*.py main.py

format:
	@echo "Formatting code..."
	black .
	isort .

# Clean
clean:
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete
	rm -rf .pytest_cache
	rm -rf .mypy_cache

# Docker
docker-build:
	docker build -t humanoid-robotics-chatbot .

docker-run:
	docker run -p 7860:7860 \
		-e COHERE_API_KEY=$(COHERE_API_KEY) \
		-e QDRANT_HOST=host.docker.internal \
		humanoid-robotics-chatbot

# Docker compose
docker-up:
	docker-compose up -d

docker-down:
	docker-compose down

docker-logs:
	docker-compose logs -f