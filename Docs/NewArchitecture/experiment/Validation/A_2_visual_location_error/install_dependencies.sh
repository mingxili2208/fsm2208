#!/bin/bash

echo "Installing FSM Sandbox Experiment Dependencies..."

# Check if virtual environment should be used
if [ "$1" = "--venv" ]; then
    echo "Setting up virtual environment..."
    python3 -m venv fsm_env
    source fsm_env/bin/activate
    echo "Virtual environment activated."
fi

# Upgrade pip first
python3 -m pip install --upgrade pip

# Install dependencies from requirements.txt
if [ -f "requirements.txt" ]; then
    echo "Installing packages from requirements.txt..."
    pip3 install -r requirements.txt
else
    echo "requirements.txt not found, installing packages individually..."
    pip3 install pandas numpy matplotlib seaborn transforms3d pyyaml scipy
fi

# Install ROS2 dependencies if needed
echo "Checking ROS2 dependencies..."
sudo apt update
sudo apt install -y python3-pandas python3-matplotlib python3-numpy python3-scipy python3-yaml

# Verify installation
echo "Verifying installation..."
python3 -c "import pandas, numpy, matplotlib, seaborn, transforms3d, yaml, scipy; print('All dependencies installed successfully!')"

echo "Installation complete!"
echo ""
echo "Usage:"
echo "  Normal installation: ./install_dependencies.sh"
echo "  With virtual env: ./install_dependencies.sh --venv"