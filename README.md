Če ne dela scripta za kamero:
¸¸
cd ~/SelfDriving_Project
python3 -m venv venv
source venv/bin/activate
pip install opencv-python
pip install depthai
python capture_2fps.py

echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
¸¸