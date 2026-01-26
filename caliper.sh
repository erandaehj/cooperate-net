#!/bin/bash

# Define the directory and the new file name
DIRECTORY="/home/eranda/cooperate-net/org1/crypto-config-ca/peerOrganizations/org1.example.com/users/User1@org1.example.com/msp/keystore"
NEW_FILE1="8b97b51627ce3d8a6ed8233eeba8d562d1b1d6897befac4d46ae171eeafc4cf0_sk"

# Navigate to the directory
echo "Navigating to the directory: $DIRECTORY"
cd "$DIRECTORY" || { echo "Failed to navigate to directory. Exiting."; exit 1; }

# Find the old file name (assuming only one file is present)
OLD_FILE1=$(ls | grep "_sk")

# Check if any file matching the pattern was found
if [ -z "$OLD_FILE1" ]; then
    echo "No file found with '_sk' in the name. Exiting."
    exit 1
fi

# Rename the file
echo "Found file: $OLD_FILE1. Renaming to $NEW_FILE1."
mv "$OLD_FILE1" "$NEW_FILE1"


# Define the directory and the new file name
DIRECTORY="/home/eranda/cooperate-net/org2/crypto-config-ca/peerOrganizations/org2.example.com/users/User1@org2.example.com/msp/keystore"
NEW_FILE4="a93d3aec25738f9fea9694d7b55972cf9139f2233e4aa015b7f8d940131ef2cf_sk"

# Navigate to the directory
echo "Navigating to the directory: $DIRECTORY"
cd "$DIRECTORY" || { echo "Failed to navigate to directory. Exiting."; exit 1; }

# Find the old file name (assuming only one file is present)
OLD_FILE4=$(ls | grep "_sk")

# Check if any file matching the pattern was found
if [ -z "$OLD_FILE4" ]; then
    echo "No file found with '_sk' in the name. Exiting."
    exit 1
fi

# Rename the file
echo "Found file: $OLD_FILE4. Renaming to $NEW_FILE4."
mv "$OLD_FILE4" "$NEW_FILE4"



# Define the directory and the new file name
DIRECTORY="/home/eranda/cooperate-net/org1/crypto-config-ca/peerOrganizations/org1.example.com/users/Admin@org1.example.com/msp/keystore"
NEW_FILE2="340fe681e051a647ded57afbb2b80ea6a5465222e356f6be208abd6b6305e6fc_sk"

# Navigate to the directory
echo "Navigating to the directory: $DIRECTORY"
cd "$DIRECTORY" || { echo "Failed to navigate to directory. Exiting."; exit 1; }

# Find the old file name (assuming only one file is present)
OLD_FILE2=$(ls | grep "_sk")

# Check if any file matching the pattern was found
if [ -z "$OLD_FILE2" ]; then
    echo "No file found with '_sk' in the name. Exiting."
    exit 1
fi

# Rename the file
echo "Found file: $OLD_FILE2. Renaming to $NEW_FILE2."
mv "$OLD_FILE2" "$NEW_FILE2"



# Define the directory and the new file name
DIRECTORY="/home/eranda/cooperate-net/org2/crypto-config-ca/peerOrganizations/org2.example.com/users/Admin@org2.example.com/msp/keystore"
NEW_FILE3="e4f5b12d4843eb134eda622ce14e002559424d9468ced66b3a671e1431bec1c6_sk"

# Navigate to the directory
echo "Navigating to the directory: $DIRECTORY"
cd "$DIRECTORY" || { echo "Failed to navigate to directory. Exiting."; exit 1; }

# Find the old file name (assuming only one file is present)
OLD_FILE3=$(ls | grep "_sk")

# Check if any file matching the pattern was found
if [ -z "$OLD_FILE3" ]; then
    echo "No file found with '_sk' in the name. Exiting."
    exit 1
fi

# Rename the file
echo "Found file: $OLD_FILE3. Renaming to $NEW_FILE3."
mv "$OLD_FILE3" "$NEW_FILE3"

cd /home/eranda/cooperate-net

npx caliper --version

# npm install --only=prod     @hyperledger/caliper-cli@0.4.2

# npx caliper launch manager     --caliper-bind-sut fabric:2.2     --caliper-workspace .     --caliper-benchconfig benchmarking/benchmarks/overtake/config.yaml     --caliper-networkconfig benchmarking/networks/fabric/v2.1/network-config_2.1.yaml  --caliper-fabric-gateway-enabled --calper-flow-only-test --caliper-fabric-gateway-discovery=false

# # Define constants
# CONFIG_FILE="/home/eranda/cooperate-net/benchmarking/benchmarks/overtake/config.yaml"
# REPORT_DIR="/home/eranda/cooperate-net/reports"
# REPORT_PREFIX="report"
# tps=400 # Keep tps constant

# # Define the specific txNumber values
# NUM_VALUES=(10000)

# # Ensure report directory exists
# mkdir -p "$REPORT_DIR"

# # Iterate over the defined txNumber values
# for NUM in "${NUM_VALUES[@]}"; do
#     echo "Running Caliper with txNumber: $NUM"
    
#     # Modify the configuration file dynamically
#     sed -i "s/txNumber: [0-9]\+/txNumber: $NUM/" "$CONFIG_FILE"

#     # Run Caliper
#     npx caliper launch manager \
#         --caliper-bind-sut fabric:2.2 \
#         --caliper-workspace . \
#         --caliper-benchconfig benchmarking/benchmarks/overtake/config.yaml \
#         --caliper-networkconfig benchmarking/networks/fabric/v2.1/network-config_2.1.yaml \
#         --caliper-fabric-gateway-enabled \
#         --caliper-flow-only-test \
#         --caliper-fabric-gateway-discovery=false \
#         --caliper-report-path "$REPORT_DIR/${REPORT_PREFIX}_txNumber_${NUM}.html"

#     # Check if the run was successful
#     if [ $? -ne 0 ]; then
#         echo "Caliper run failed for txNumber: $NUM"
#         exit 1
#     fi
# done

# echo "Caliper runs completed. Reports saved in $REPORT_DIR"


# Define constants
CONFIG_FILE="/home/eranda/cooperate-net/benchmarking/benchmarks/overtake/config.yaml"
REPORT_DIR="/home/eranda/cooperate-net/reports"
REPORT_PREFIX="report"

# Define the specific TPS values to iterate over
TPS_VALUES=(75)  # Modify these values as needed

# Ensure report directory exists
mkdir -p "$REPORT_DIR"

# Iterate over the defined TPS values
for TPS in "${TPS_VALUES[@]}"; do
    echo "Running Caliper with tps: $TPS"
    
    # Modify the configuration file dynamically by replacing the tps value.
    # This assumes that the config file has a line that starts with "tps:".
    sed -i "s/tps: [0-9]\+/tps: $TPS/" "$CONFIG_FILE"

    # Run Caliper with the updated configuration
    npx caliper launch manager \
        --caliper-bind-sut fabric:2.2 \
        --caliper-workspace . \
        --caliper-benchconfig benchmarking/benchmarks/overtake/config.yaml \
        --caliper-networkconfig benchmarking/networks/fabric/v2.1/network-config_2.1.yaml \
        --caliper-fabric-gateway-enabled \
        --caliper-flow-only-test \
        --caliper-fabric-gateway-discovery=false \
        --caliper-report-path "$REPORT_DIR/${REPORT_PREFIX}_tps_${TPS}.html"

    # Check if the run was successful
    if [ $? -ne 0 ]; then
        echo "Caliper run failed for tps: $TPS"
        exit 1
    fi
done

echo "Caliper runs completed. Reports saved in $REPORT_DIR"

# docker stats --no-stream

# cd chaincode/overtaking/
# go mod init github.com/erandaehj/cooperate-net
# go mod tidy


# docker stop grafana