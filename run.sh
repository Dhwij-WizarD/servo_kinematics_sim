#!/bin/bash

if [ -f "myenv/bin/activate" ]; then
    source myenv/bin/activate
elif [ -f "../myenv/bin/activate" ]; then
    source ../myenv/bin/activate
else
    echo "Failed to activate virtual environment. Please check if 'myenv' exists."
    exit 1
fi

echo "Virtual environment activated successfully."

# Check if mujoco is installed
if ! python3 -c "import mujoco" &> /dev/null; then
    echo "MuJoCo not found. Installing..."
    pip install mujoco
fi

echo "Starting MuJoCo simulation with model/arm_3dof.xml..."

cd servo_kinematics_sim
# Run the controller script
python3 controller.py