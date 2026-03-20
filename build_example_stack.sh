#!/bin/bash

REGISTRY="722180079256.dkr.ecr.eu-central-1.amazonaws.com"
TEAM="dspace"
DATE=$(date '+%Y-%m-%d')

# define name of the image
NAME_EXAMPLE_STACK_DEV=$TEAM/iac_uva_demo_av_stack_dev:$DATE
NAME_EXAMPLE_STACK=$TEAM/iac_uva_demo_av_stack:$DATE

# build image
echo "---------- build example stack dev version ----------"
docker build -t $NAME_EXAMPLE_STACK_DEV --target=example_driving_stack_dev -f demo_stack_uva/sample_controller/Dockerfile .
echo "---------- build example stack ----------"
docker build -t $NAME_EXAMPLE_STACK --target=example_driving_stack -f demo_stack_uva/sample_controller/Dockerfile .
