#!/bin/bash
# Helper script for AutoDrone Docker environment

set -e  # Exit on error

# Colors for better output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}=========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}=========================================${NC}"
}

check_docker() {
    if ! command -v docker &> /dev/null; then
        echo -e "${RED}Docker is not installed. Please install Docker first.${NC}"
        echo "Visit https://docs.docker.com/engine/install/ for installation instructions."
        exit 1
    fi

    if ! command -v docker-compose &> /dev/null; then
        echo -e "${RED}Docker Compose is not installed. Please install Docker Compose first.${NC}"
        echo "Visit https://docs.docker.com/compose/install/ for installation instructions."
        exit 1
    fi
}

build_container() {
    print_header "Building AutoDrone Docker Container"
    echo -e "${YELLOW}This might take a while (20-30 minutes) the first time...${NC}"
    # Try direct docker build instead of using docker-compose
    sudo docker build -t autodrone_autodrone .
    echo -e "${GREEN}Build completed successfully!${NC}"
}

run_container() {
    print_header "Starting AutoDrone Container"
    
    # Allow X server connections
    xhost + local:root
    
    # Build the container if it doesn't exist yet
    if ! sudo docker images | grep -q autodrone_autodrone; then
        build_container
    fi
    
    # Stop and remove any existing container named 'autodrone'
    print_header "Stopping existing container (if any)"
    sudo docker stop autodrone 2>/dev/null || true
    sudo docker rm autodrone 2>/dev/null || true
    
    # Run the container with GPU support using direct docker command
    print_header "Running new container"
    sudo docker run --gpus all -it --rm \
        --name autodrone \
        --network host \
        --privileged \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $(pwd):/root/AutoDrone:rw \
        autodrone_autodrone
    
    # Revoke X server access (optional, consider security implications)
    # xhost - local:root 
}

show_help() {
    echo "Usage: ./docker_autodrone.sh [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build      Build the AutoDrone Docker container"
    echo "  run        Run the AutoDrone Docker container"
    echo "  restart    Restart the AutoDrone Docker container"
    echo "  stop       Stop the AutoDrone Docker container"
    echo "  help       Show this help message"
    echo ""
    echo "Example: ./docker_autodrone.sh build"
}

# Check if Docker is installed
check_docker

# Parse command line arguments
case "$1" in
    build)
        build_container
        ;;
    run)
        run_container
        ;;
    restart)
        print_header "Restarting AutoDrone Container"
        sudo docker stop autodrone 2>/dev/null || true
        sudo docker rm autodrone 2>/dev/null || true
        run_container
        ;;
    stop)
        print_header "Stopping AutoDrone Container"
        sudo docker stop autodrone 2>/dev/null || true
        sudo docker rm autodrone 2>/dev/null || true
        echo -e "${GREEN}Container stopped and removed.${NC}"
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        if [ "$1" != "" ]; then
            echo -e "${RED}Unknown command: $1${NC}"
        fi
        show_help
        exit 1
        ;;
esac

exit 0
