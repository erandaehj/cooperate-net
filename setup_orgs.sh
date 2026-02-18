#!/bin/bash

# Check if Docker service is running
if ! systemctl is-active --quiet docker; then
    echo "Docker service is not running. Starting it now..."
    sudo systemctl start docker
else
    echo "Docker service is already running."
fi

echo -e "======Setting up fabric commands======\n"
wget "https://github.com/hyperledger/fabric/releases/download/v2.5.11/hyperledger-fabric-linux-amd64-2.5.11.tar.gz" -O "fabricbin.tar.gz"
mkdir fabricbin
tar -xvzf fabricbin.tar.gz -C ./fabricbin
mv fabricbin/bin .
rm -r fabricbin
rm fabricbin.tar.gz


wget "https://github.com/hyperledger/fabric-ca/releases/download/v1.5.15/hyperledger-fabric-ca-linux-amd64-1.5.15.tar.gz" -O "fabricca.tar.gz"
mkdir fabricca
tar -xvzf fabricca.tar.gz -C ./fabricca
mv fabricca/bin/* ./bin/
rm -r fabricca
rm fabricca.tar.gz 

export PATH=${PWD}/bin:$PATH

echo -e "======Switching to git master======\n"

git switch main

echo -e "======Setting up Org1======\n"
(cd org1; ./1_enrollOrg1AdminAndUsers.sh; ./2_generateMSPOrg1.sh)
 
echo -e "\n======Setting up Org2======\n"
(cd org2; ./1_enrollOrg2AdminAndUsers.sh; ./2_generateMSPOrg2.sh )
 
# echo -e "\n======Setting up Org3======\n"
# (cd org3; ./1_enrollOrg3AdminAndUsers.sh; ./2_generateMSPOrg3.sh )

echo -e "\n======Setting up Orderer======\n"
(cd orderer; ./1_enrollAdminAndMSP.sh; ./2_artifact.sh)
 
sleep 10

echo -e "\n======Creating Channel======\n"
(cd org1; ./3_createChannel.sh)
 
sleep 10

echo -e "\n======Org 2 Joining Channel======\n"

(cd org2; ./3_joinChannel.sh)

# echo -e "\n======Org 3 Joining Channel======\n"

# (cd org3; ./3_joinChannel.sh)

sleep 10

echo -e "\n======Deploy Chaincode ======\n"

(cd deployChaincode; ./deployOrg1_GO.sh $1;./deployOrg2_GO.sh $1;)