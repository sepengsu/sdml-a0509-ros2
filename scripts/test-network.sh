#!/bin/bash
# Network connectivity test script for ros-control container
# Tests container-to-container, container-to-host, and container-to-external connectivity

set -o pipefail

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color
BOLD='\033[1m'

# Configuration
TIMEOUT=5
ROBOT_IP="${ROBOT_IP:-192.168.137.100}"
ROBOT_EMULATOR_HOST="robot-emulator"
ROBOT_EMULATOR_PORT="12345"
HOST_DOCKER_INTERNAL="host.docker.internal"
HOST_PORT="9999"

# Test counters
TESTS_RUN=0
TESTS_PASSED=0
TESTS_FAILED=0

# Function to print colored output
print_success() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

print_failure() {
    echo -e "${RED}[FAIL]${NC} $1"
}

print_info() {
    echo -e "${YELLOW}[INFO]${NC} $1"
}

print_header() {
    echo -e "\n${BOLD}=== $1 ===${NC}"
}

# Function to run a test command with timeout
run_test() {
    local description="$1"
    local command="$2"
    local is_critical="${3:-false}"

    TESTS_RUN=$((TESTS_RUN + 1))
    echo -n "Testing: $description... "

    if timeout "$TIMEOUT" bash -c "$command" &>/dev/null; then
        print_success "$description"
        TESTS_PASSED=$((TESTS_PASSED + 1))
        return 0
    else
        if [ "$is_critical" = "true" ]; then
            print_failure "$description"
            TESTS_FAILED=$((TESTS_FAILED + 1))
        else
            echo -e "${YELLOW}[SKIP]${NC} $description (service may not be running)"
        fi
        return 1
    fi
}

# Banner
echo -e "${BOLD}"
echo "╔═══════════════════════════════════════════════════════╗"
echo "║   Network Connectivity Test - ROS Control Container  ║"
echo "╚═══════════════════════════════════════════════════════╝"
echo -e "${NC}"

print_info "Timeout: ${TIMEOUT}s per test"
print_info "Robot IP: ${ROBOT_IP}"
echo ""

# Check if basic tools are available
print_header "Checking Required Tools"
for tool in ping nc timeout; do
    if command -v "$tool" &>/dev/null; then
        print_success "$tool is available"
    else
        print_failure "$tool is NOT available"
        echo "Please install required networking tools"
        exit 1
    fi
done

# Test 1: Container-to-Container Communication
print_header "Container-to-Container Communication"

# Ping robot-emulator
run_test "Ping robot-emulator container" \
    "ping -c 2 -W $TIMEOUT $ROBOT_EMULATOR_HOST" \
    "false"

# TCP connection to robot-emulator
run_test "TCP connection to robot-emulator:$ROBOT_EMULATOR_PORT" \
    "nc -z -w $TIMEOUT $ROBOT_EMULATOR_HOST $ROBOT_EMULATOR_PORT" \
    "false"

# Test 2: Container-to-Host Communication
print_header "Container-to-Host Communication"

# Ping host.docker.internal
run_test "Ping host.docker.internal" \
    "ping -c 2 -W $TIMEOUT $HOST_DOCKER_INTERNAL" \
    "false"

# TCP connection to host
run_test "TCP connection to host.docker.internal:$HOST_PORT" \
    "nc -z -w $TIMEOUT $HOST_DOCKER_INTERNAL $HOST_PORT" \
    "false"

# Test 3: Container-to-External Network
print_header "Container-to-External Network"

# Ping external robot IP
run_test "Ping external robot at $ROBOT_IP" \
    "ping -c 2 -W $TIMEOUT $ROBOT_IP" \
    "false"

# Test DNS resolution
run_test "DNS resolution (google.com)" \
    "ping -c 1 -W $TIMEOUT google.com" \
    "false"

# Test 4: Network Interface Information
print_header "Network Interface Information"

echo "Network interfaces:"
ip addr show 2>/dev/null || ifconfig 2>/dev/null || echo "No network interface info available"
echo ""

echo "Routing table:"
ip route show 2>/dev/null || route -n 2>/dev/null || echo "No routing info available"
echo ""

# Summary
print_header "Test Summary"

echo -e "Total tests run:    ${BOLD}$TESTS_RUN${NC}"
echo -e "Tests passed:       ${GREEN}${BOLD}$TESTS_PASSED${NC}"
echo -e "Tests failed:       ${RED}${BOLD}$TESTS_FAILED${NC}"
echo ""

if [ "$TESTS_FAILED" -eq 0 ]; then
    echo -e "${GREEN}${BOLD}All critical tests passed!${NC}"
    echo ""
    exit 0
else
    echo -e "${RED}${BOLD}Some tests failed. Please check network configuration.${NC}"
    echo ""
    echo "Troubleshooting tips:"
    echo "1. Verify Docker network configuration"
    echo "2. Check if robot-emulator container is running: docker ps"
    echo "3. Verify host services are listening on expected ports"
    echo "4. Check firewall rules on host and external robot"
    exit 1
fi
