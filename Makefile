.PHONY: help build run shell stop clean test

help:
	@echo "Gap Detection - Common Commands"
	@echo ""
	@echo "Docker Commands:"
	@echo "  make build         - Build Docker image"
	@echo "  make run BAG=path  - Run with rosbag file"
	@echo "  make shell         - Open Docker shell"
	@echo "  make stop          - Stop running container"
	@echo "  make clean         - Remove Docker container and image"
	@echo ""
	@echo "Native ROS Commands (requires ROS installed):"
	@echo "  make catkin        - Build with catkin_make"
	@echo "  make test          - Run Python tests"
	@echo ""
	@echo "Examples:"
	@echo "  make build"
	@echo "  make run BAG=/path/to/data.bag"
	@echo "  make run BAG=/path/to/data.bag LOOP=1"
	@echo "  make run BAG=/path/to/data.bag RATE=0.5"

build:
	./docker-run.sh build

run:
	@if [ -z "$(BAG)" ]; then \
		echo "Error: BAG file path required. Usage: make run BAG=/path/to/file.bag"; \
		exit 1; \
	fi
	@CMD="./docker-run.sh run --bag-file $(BAG)"; \
	if [ "$(LOOP)" = "1" ]; then CMD="$$CMD --loop"; fi; \
	if [ -n "$(RATE)" ]; then CMD="$$CMD --rate $(RATE)"; fi; \
	if [ "$(NOVIZ)" = "1" ]; then CMD="$$CMD --no-viz"; fi; \
	$$CMD

shell:
	./docker-run.sh shell

stop:
	./docker-run.sh stop

clean:
	./docker-run.sh clean

catkin:
	@if [ ! -d "$$HOME/catkin_ws" ]; then \
		echo "Error: catkin workspace not found at ~/catkin_ws"; \
		exit 1; \
	fi
	cd ~/catkin_ws && catkin_make
	@echo "Build complete! Don't forget to source: source ~/catkin_ws/devel/setup.bash"

test:
	python3 -m pytest tests/ -v
