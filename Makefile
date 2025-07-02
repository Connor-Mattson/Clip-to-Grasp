# Makefile
# TODO: Comeback and do thorough testing on this.

# Name of the environment
ENV_NAME=roboCLIP
PYTHON_VERSION=3.11
ENV_FILE=environment.yml
REQUIREMENTS=requirements.txt

.PHONY: help setup install test clean lint

help:
	@echo "Available commands:"
	@echo "  make setup      - Create a conda environment and install dependencies"
	@echo "  make install    - Reinstall pip packages into the current environment"
	@echo "  make test       - Run tests (pytest)"
	@echo "  make lint       - Run linter (flake8 or black)"
	@echo "  make clean      - Remove cache and temporary files"

setup:
	conda create -n $(ENV_NAME) python=$(PYTHON_VERSION) -y
	conda activate $(ENV_NAME) && conda env update --file $(ENV_FILE) --prune
	conda activate $(ENV_NAME) && pip install -r $(REQUIREMENTS)

install:
	pip install -r $(REQUIREMENTS)

test:
	pytest tests/

lint:
	black . --check

clean:
	find . -type d -name "__pycache__" -exec rm -r {} +
	find . -type f -name "*.pyc" -delete
