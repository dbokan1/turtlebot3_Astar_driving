# turtlebot3_Astar_driving

## Instaliranje
- git klonirati
- catkin_make.

Stablo bi trebalo izgledati: catkin_ws/src/tb3_control_interface.

## Pokretanje
- roscore
- roslaunch turtlebot3_gazebo turtlebot3_world.launch
- rosrun tb3_control_py test.py

Ovo pokrece kretanje turtlebota naprijed pa okretanje i kretanje lijevo.

## Fileovi
- turtlebot3_class.cpp: glavni file za interakciju sa turtlebotom, primanje odom poruka i slanje cmd_vel poruka.
- cv_image.cpp: demo primjer kako koristiti opencv sa ROSom za rad sa slikama.
- a*.cpp: kod koji ucita grayscale sliku mape, sa zadanom pocetnom i krajnjom pozicijom pronadje optimalnu putanju (ne radi).

## Obaveze
- [x] Napraviti komunikaciju sa turtlebotom (odom i cmd_vel)
- [x] Gmappingom dobiti sliku mape
- [ ] Napraviti a* koji radi sa slikom i ROSom i updateati cmakelist
- [ ] Za transfer upravljacki blok: prelaz iz pixel koordinatnog sistema slike u real world koordinatni sistem simulacije
- [ ] Za transfer upravljacki blok: diskretizirati real world koordinatni da se omoguci kretanje iz jednog mjesta u drugo
- [ ] Za transfer upravljacki blok: ostvariti upravljacki algoritam za slanje ugaone i linearne brzine uz fizikalna ogranicenja
- [ ] Za transfer upravljacki blok: implementirati povratnu spregu za real time updateovanje brzine na osnovu odometrije
- [ ] Povezati sve kodove: Slika u A*, putanja iz A* u control blok, upravljacki signal iz bloka u simulaciju, odom nazad
