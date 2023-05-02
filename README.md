# turtlebot3_Astar_driving

## Instaliranje
- git klonirati
- catkin_make.

Stablo bi trebalo izgledati: catkin_ws/src/tb3_control_interface.

## Pokretanje
- roscore
- roslaunch turtlebot3_gazebo turtlebot3_world.launch
- rosrun tb3_control_interface turtlebot3_class.

Ovo pokrece kretanje turtlebota naprijed pa okretanje i kretanje lijevo.

## Fileovi
- turtlebot3_class.cpp: glavni file za interakciju sa turtlebotom, primanje odom poruka i slanje cmd_vel poruka.
- cv_image.cpp: demo primjer kako koristiti opencv sa ROSom za rad sa slikama.
- a*.cpp: kod koji ucita grayscale sliku mape, sa zadanom pocetnom i krajnjom pozicijom pronadje optimalnu putanju (ne radi).

## Obaveze
- [ ] Napraviti komunikaciju sa turtlebotom (odom i cmd_vel)
- [ ] Gmappingom dobiti sliku mape
- [ ] Napraviti a* koji radi sa slikom i ROSom i updateati cmakelist
- [ ] Napraviti transfer_fcn koja ce prilagoditi vrijednosti world koordinata u image space i diskretizirati sliku mape- povezani graf
- [ ] Povezati sve kodove: Slika u A*, putanja iz A* u turtlebot3_class, upravljacki signal iz turtlebot3_class u transfer_fcn i odom ocitavanja nazad.
