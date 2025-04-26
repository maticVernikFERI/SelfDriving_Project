Pred zagonom rabiš:
```
export CYCLONEDDS_URI=file:///$HOME/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run demo_nodes_cpp listener
```

V cyclonedds.xml datoteki moreš edino spremenit PI naslov na naslov drugega (laptop nastavi na IP od Raspberry-ja in obratno)