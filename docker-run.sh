#!/bin/bash

# Gap Detection Docker Helper Script
# This script simplifies running the gap detection package in Docker

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
IMAGE_NAME="ugoe_gap_detection_ros:latest"
CONTAINER_NAME="gap_detection"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_help() {
    echo "Gap Detection Docker Helper"
    echo ""
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  build              Build the Docker image"
    echo "  run                Run the gap detection with rosbag"
    echo "  shell              Open a bash shell in the container"
    echo "  stop               Stop the running container"
    echo "  clean              Remove container and image"
    echo "  help               Show this help message"
    echo ""
    echo "Run Options (use with 'run' command):"
    echo "  --bag-file PATH    Path to rosbag file (required)"
    echo "  --loop             Loop the rosbag playback"
    echo "  --rate FLOAT       Playback rate (default: 1.0)"
    echo "  --no-viz           Disable visualization (RViz and rqt)"
    echo ""
    echo "Examples:"
    echo "  $0 build"
    echo "  $0 run --bag-file /path/to/data.bag"
    echo "  $0 run --bag-file /path/to/data.bag --loop --rate 0.5"
    echo "  $0 shell"
}

build_image() {
    echo -e "${GREEN}Building Docker image...${NC}"
    cd "$SCRIPT_DIR"
    docker build -t "$IMAGE_NAME" .
    echo -e "${GREEN}Build complete!${NC}"
}

run_container() {
    # Parse arguments
    BAG_FILE=""
    LOOP="false"
    RATE="1.0"
    ENABLE_VIZ="true"
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --bag-file)
                BAG_FILE="$2"
                shift 2
                ;;
            --loop)
                LOOP="true"
                shift
                ;;
            --rate)
                RATE="$2"
                shift 2
                ;;
            --no-viz)
                ENABLE_VIZ="false"
                shift
                ;;
            *)
                echo -e "${RED}Unknown option: $1${NC}"
                exit 1
                ;;
        esac
    done
    
    if [ -z "$BAG_FILE" ]; then
        echo -e "${RED}Error: --bag-file is required${NC}"
        echo "Use '$0 help' for usage information"
        exit 1
    fi
    
    if [ ! -f "$BAG_FILE" ]; then
        echo -e "${RED}Error: Bag file not found: $BAG_FILE${NC}"
        exit 1
    fi
    
    # Allow X11 forwarding
    xhost +local:docker > /dev/null 2>&1
    
    echo -e "${GREEN}Starting gap detection...${NC}"
    echo "Bag file: $BAG_FILE"
    echo "Loop: $LOOP"
    echo "Rate: $RATE"
    echo "Visualization: $ENABLE_VIZ"
    echo ""
    
    # Get absolute path and filename
    BAG_DIR=$(dirname "$(realpath "$BAG_FILE")")
    BAG_NAME=$(basename "$BAG_FILE")
    
    docker run --rm -it \
        --name "$CONTAINER_NAME" \
        --network host \
        -e DISPLAY="$DISPLAY" \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
        -v "$SCRIPT_DIR:/root/catkin_ws/src/ugoe_gap_detection_ros:rw" \
        -v "$BAG_DIR:/root/rosbags:ro" \
        --privileged \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/noetic/setup.bash && \
                 source /root/catkin_ws/devel/setup.bash && \
                 roslaunch ugoe_gap_detection_ros ugoe_gap_detection_ros.launch \
                     bag_file:=/root/rosbags/$BAG_NAME \
                     bag_loop:=$LOOP \
                     bag_rate:=$RATE \
                     enable_rviz:=$ENABLE_VIZ \
                     enable_rqt:=$ENABLE_VIZ"
}

open_shell() {
    # Allow X11 forwarding
    xhost +local:docker > /dev/null 2>&1
    
    echo -e "${GREEN}Opening shell in container...${NC}"
    docker run --rm -it \
        --name "$CONTAINER_NAME" \
        --network host \
        -e DISPLAY="$DISPLAY" \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
        -v "$SCRIPT_DIR:/root/catkin_ws/src/ugoe_gap_detection_ros:rw" \
        --privileged \
        "$IMAGE_NAME" \
        bash
}

stop_container() {
    echo -e "${YELLOW}Stopping container...${NC}"
    docker stop "$CONTAINER_NAME" 2>/dev/null || echo "Container not running"
}

clean() {
    echo -e "${YELLOW}Cleaning up...${NC}"
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
    docker rmi "$IMAGE_NAME" 2>/dev/null || true
    echo -e "${GREEN}Cleanup complete${NC}"
}

# Main script logic
case "$1" in
    build)
        build_image
        ;;
    run)
        shift
        run_container "$@"
        ;;
    shell)
        open_shell
        ;;
    stop)
        stop_container
        ;;
    clean)
        clean
        ;;
    help|--help|-h)
        print_help
        ;;
    *)
        if [ -z "$1" ]; then
            print_help
        else
            echo -e "${RED}Unknown command: $1${NC}"
            echo "Use '$0 help' for usage information"
            exit 1
        fi
        ;;
esac
