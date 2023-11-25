# MMRS TurtleBot3 simulation in docker container

```bash
ros2 launch turtlebot_mmrs unique_multi_tb3_simulation_launch.py autostart:=True
ros2 launch turtlebot_mmrs turtlebot_mmrs_launch.py
```

architektura systemu na bazie tego co napisala profesor w swojej publikacji

najlepiej bedzie porownywac dwa modele MMRS, U-MMRS (Uncontrolled) i to co implementuje

pobawic sie z behavior tree w nav2 jesli pojawia sie problemy z replanowaniem sciezki
